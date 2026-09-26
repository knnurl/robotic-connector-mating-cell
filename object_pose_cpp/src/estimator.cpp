// See estimator.hpp. Each function carries the name of the Python one it
// ports (roscam/object_pose.py); comments there explain the why.
#include "estimator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <set>
#include <utility>

namespace object_pose {

struct Estimator::Ctx {
  Pts3 S;                         // scene points
  Mat3 K;
  VecX dist;
  cv::Mat cvK, cvDist;
  const cv::Mat* depth = nullptr;
  std::array<int, 4> roi{};
  double f = 0.0;
  KdTree<2> out_tree;
  Pts2 out_uv, out_n;
};

namespace {

constexpr double kPi = 3.14159265358979323846;

inline double rad(double deg) { return deg * kPi / 180.0; }
inline double deg(double r) { return r * 180.0 / kPi; }
// numpy's round: half to even (the default rounding mode)
inline double rnd(double v) { return std::nearbyint(v); }

inline double depth_at(const cv::Mat& d, int y, int x) {
  return d.type() == CV_32F ? static_cast<double>(d.at<float>(y, x)) : d.at<double>(y, x);
}

cv::Mat to_cv(const Mat3& K) {
  cv::Mat m(3, 3, CV_64F);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) m.at<double>(i, j) = K(i, j);
  return m;
}

cv::Mat to_cv(const VecX& d) {
  cv::Mat m(static_cast<int>(d.size()), 1, CV_64F);
  for (int i = 0; i < d.size(); ++i) m.at<double>(i) = d(i);
  return m;
}

// cv2.projectPoints(P, 0, 0, K, dist)
Pts2 project(const Pts3& P, const cv::Mat& K, const cv::Mat& dist) {
  Pts2 uv(P.rows(), 2);
  if (P.rows() == 0) return uv;
  cv::Mat obj(static_cast<int>(P.rows()), 1, CV_64FC3, const_cast<double*>(P.data()));
  cv::Mat img;
  cv::projectPoints(obj, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), K, dist,
                    img);
  for (int i = 0; i < P.rows(); ++i) {
    const cv::Vec2d p = img.at<cv::Vec2d>(i);
    uv(i, 0) = p[0];
    uv(i, 1) = p[1];
  }
  return uv;
}

// cv2.undistortPoints(pts, K, dist, P=K)
Pts2 undistort_px(const Pts2& pts, const cv::Mat& K, const cv::Mat& dist) {
  Pts2 out(pts.rows(), 2);
  if (pts.rows() == 0) return out;
  cv::Mat src(static_cast<int>(pts.rows()), 1, CV_64FC2, const_cast<double*>(pts.data()));
  cv::Mat dst;
  cv::undistortPoints(src, dst, K, dist, cv::noArray(), K);
  for (int i = 0; i < pts.rows(); ++i) {
    const cv::Vec2d p = dst.at<cv::Vec2d>(i);
    out(i, 0) = p[0];
    out(i, 1) = p[1];
  }
  return out;
}

Pts3 transform(const Pts3& X, const Mat3& R, const Vec3& t) {
  Pts3 P(X.rows(), 3);
  for (int i = 0; i < X.rows(); ++i) {
    P.row(i) = (R * X.row(i).transpose() + t).transpose();
  }
  return P;
}

Mat3 skew(const Vec3& v) {
  Mat3 s;
  s << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return s;
}

// _tukey
inline double tukey(double r, double c) {
  double u = std::abs(r) / c;
  u = std::min(std::max(u, 0.0), 1.0);
  const double a = 1.0 - u * u;
  return a * a;
}

// fit_plane (plane_normal.py): least squares by SVD
struct Plane {
  Vec3 n, c;
  double rms;
};

std::optional<Plane> fit_plane(const std::vector<Vec3>& p) {
  if (p.size() < 3) return std::nullopt;
  Vec3 c = Vec3::Zero();
  for (const auto& v : p) c += v;
  c /= static_cast<double>(p.size());
  Eigen::MatrixX3d Q(p.size(), 3);
  for (std::size_t i = 0; i < p.size(); ++i) Q.row(i) = (p[i] - c).transpose();
  Eigen::JacobiSVD<Eigen::MatrixX3d> svd(Q, Eigen::ComputeThinV);
  const auto s = svd.singularValues();
  if (s(1) < 1e-12) return std::nullopt;
  Vec3 n = svd.matrixV().col(2);
  const double nn = n.norm();
  if (nn < 1e-12) return std::nullopt;
  n /= nn;
  double ss = 0.0;
  for (int i = 0; i < Q.rows(); ++i) {
    const double d = Q.row(i).dot(n);
    ss += d * d;
  }
  return Plane{n, c, std::sqrt(ss / static_cast<double>(Q.rows()))};
}

// np.quantile(d, q), method 'linear' (numpy >= 1.22: _lerp)
double quantile(std::vector<double> d, double q) {
  std::sort(d.begin(), d.end());
  const double vi = q * static_cast<double>(d.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(vi));
  const std::size_t hi = std::min(lo + 1, d.size() - 1);
  const double g = vi - static_cast<double>(lo);
  const double a = d[lo], b = d[hi], diff = b - a;
  return g >= 0.5 ? b - diff * (1.0 - g) : a + diff * g;
}

// fit_plane_robust (plane_normal.py): quantile trims, then a sigma pass
std::optional<Plane> fit_plane_robust(std::vector<Vec3> p, int trims = 3,
                                      double keep_frac = 0.8, double sigma = 2.5,
                                      std::size_t min_points = 12) {
  auto res = fit_plane(p);
  if (!res) return std::nullopt;
  auto dists = [](const std::vector<Vec3>& pts, const Plane& pl) {
    std::vector<double> d(pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) d[i] = std::abs((pts[i] - pl.c).dot(pl.n));
    return d;
  };
  for (int it = 0; it < trims; ++it) {
    if (p.size() <= min_points) break;
    const auto d = dists(p, *res);
    const double thr = quantile(d, keep_frac);
    std::vector<Vec3> kept;
    for (std::size_t i = 0; i < p.size(); ++i)
      if (d[i] <= thr) kept.push_back(p[i]);
    if (kept.size() < std::max<std::size_t>(min_points, 3)) break;
    auto nxt = fit_plane(kept);
    if (!nxt) break;
    p = std::move(kept);
    res = nxt;
  }
  if (res->rms > 0.0 && p.size() > min_points) {
    const auto d = dists(p, *res);
    std::vector<Vec3> kept;
    for (std::size_t i = 0; i < p.size(); ++i)
      if (d[i] <= sigma * res->rms) kept.push_back(p[i]);
    if (kept.size() >= std::max<std::size_t>(min_points, 3) && kept.size() < p.size()) {
      auto nxt = fit_plane(kept);
      if (nxt) res = nxt;
    }
  }
  return res;
}

template <typename T>
T clampv(T v, T lo, T hi) {
  return std::min(std::max(v, lo), hi);
}

}  // namespace

// ------------------------------------------------------------------ setup

Estimator::Estimator(Model model, Params params) : m_(std::move(model)), p_(params) {
  tree_.build(m_.samples.data(), static_cast<int>(m_.samples.rows()));
}

const cv::Mat& Estimator::norm_grid(const Mat3& K, const VecX& dist, int h, int w) {
  const bool same = h == norm_h_ && w == norm_w_ && K == norm_K_ &&
                    dist.size() == norm_dist_.size() && dist == norm_dist_;
  if (!same) {
    cv::Mat px(h * w, 1, CV_64FC2);
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) px.at<cv::Vec2d>(y * w + x) = cv::Vec2d(x, y);
    cv::Mat out;
    cv::undistortPoints(px, out, to_cv(K), to_cv(dist));
    norm_ = out.reshape(2, h).clone();
    norm_K_ = K;
    norm_dist_ = dist;
    norm_h_ = h;
    norm_w_ = w;
  }
  return norm_;
}

void Estimator::warm(const Mat3& K, const VecX& dist, int h, int w) { norm_grid(K, dist, h, w); }

// _facing
std::vector<char> Estimator::facing(const Mat3& R, const Vec3& t) const {
  const int nf = static_cast<int>(m_.F.rows());
  std::vector<char> out(nf);
  for (int i = 0; i < nf; ++i) {
    const Vec3 n = R * m_.face_n.row(i).transpose();
    const Vec3 c = R * m_.face_c.row(i).transpose() + t;
    out[i] = n.dot(c) < 0.0;
  }
  return out;
}

// _render: mask (ROI-local) of the mesh at R, t; dist null = pinhole
cv::Mat Estimator::render(const Mat3& R, const Vec3& t, const Mat3& K, const VecX* dist,
                          const std::array<int, 4>& roi,
                          const std::vector<char>& face_on) const {
  const int x0 = roi[0], y0 = roi[1], x1 = roi[2], y1 = roi[3];
  cv::Mat mask = cv::Mat::zeros(y1 - y0, x1 - x0, CV_8U);
  Idx tri;
  for (int i = 0; i < static_cast<int>(face_on.size()); ++i)
    if (face_on[i]) tri.push_back(i);
  if (tri.empty()) return mask;
  const Pts3 P = transform(m_.V, R, t);
  for (int i = 0; i < P.rows(); ++i)
    if (P(i, 2) <= 1e-4) return mask;
  Pts2 uv(P.rows(), 2);
  if (dist == nullptr) {
    for (int i = 0; i < P.rows(); ++i) {
      uv(i, 0) = P(i, 0) / P(i, 2) * K(0, 0) + K(0, 2);
      uv(i, 1) = P(i, 1) / P(i, 2) * K(1, 1) + K(1, 2);
    }
  } else {
    uv = project(P, to_cv(K), to_cv(*dist));
  }
  std::vector<std::vector<cv::Point>> polys;
  polys.reserve(tri.size());
  for (int f : tri) {
    std::vector<cv::Point> poly(3);
    for (int k = 0; k < 3; ++k) {
      const int v = m_.F(f, k);
      poly[k] = cv::Point(static_cast<int>(rnd((uv(v, 0) - x0) * 16.0)),
                          static_cast<int>(rnd((uv(v, 1) - y0) * 16.0)));
    }
    polys.push_back(std::move(poly));
  }
  cv::fillPoly(mask, polys, cv::Scalar(1), cv::LINE_8, 4);
  return mask;
}

// _rim: silhouette points at R, t, on the outer outline, not hidden
void Estimator::rim(const Mat3& R, const Vec3& t, const Mat3& K, const VecX& dist,
                    const cv::Mat& depth, Pts3& X, Pts3& P, Idx& eid) const {
  const int nf = static_cast<int>(m_.F.rows());
  std::vector<double> cos_inc(nf);
  std::vector<char> face_on(nf);
  for (int i = 0; i < nf; ++i) {
    const Vec3 n = R * m_.face_n.row(i).transpose();
    const Vec3 c = R * m_.face_c.row(i).transpose() + t;
    cos_inc[i] = -n.dot(c) / c.norm();
    face_on[i] = cos_inc[i] > 0.0;
  }
  const int ne = static_cast<int>(m_.edge_faces.rows());
  std::vector<char> sil(ne);
  for (int k = 0; k < ne; ++k) {
    const int f0 = m_.edge_faces(k, 0), f1 = m_.edge_faces(k, 1);
    const bool a = face_on[f0];
    const bool b = f1 >= 0 ? static_cast<bool>(face_on[f1]) : false;
    const int front = a ? f0 : std::max(f1, 0);
    sil[k] = ((a != b) && (cos_inc[front] > p_.rim_min_cos)) || f1 == -2;
  }
  std::vector<int> on;
  for (int i = 0; i < static_cast<int>(m_.edge_id.size()); ++i)
    if (sil[m_.edge_id[i]]) on.push_back(i);
  // P = X R^T + t, kept in front of the camera
  std::vector<int> keep;
  Pts3 Pall(on.size(), 3);
  for (std::size_t j = 0; j < on.size(); ++j) {
    Pall.row(j) = (R * m_.edge_pts.row(on[j]).transpose() + t).transpose();
    if (Pall(j, 2) > 1e-4) keep.push_back(static_cast<int>(j));
  }
  X.resize(keep.size(), 3);
  P.resize(keep.size(), 3);
  eid.resize(keep.size());
  for (std::size_t j = 0; j < keep.size(); ++j) {
    X.row(j) = m_.edge_pts.row(on[keep[j]]);
    P.row(j) = Pall.row(keep[j]);
    eid[j] = m_.edge_id[on[keep[j]]];
  }
  if (X.rows() == 0) return;
  // render just around the part at this pose (pinhole), and keep the points
  // on its outer outline
  const Pts3 Pv = transform(m_.V, R, t);
  double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
  for (int i = 0; i < Pv.rows(); ++i) {
    const double z = std::max(Pv(i, 2), 1e-4);
    const double u = Pv(i, 0) / z * K(0, 0) + K(0, 2);
    const double v = Pv(i, 1) / z * K(1, 1) + K(1, 2);
    umin = std::min(umin, u);
    umax = std::max(umax, u);
    vmin = std::min(vmin, v);
    vmax = std::max(vmax, v);
  }
  const std::array<int, 4> box = {static_cast<int>(std::floor(umin)) - 4,
                                  static_cast<int>(std::floor(vmin)) - 4,
                                  static_cast<int>(std::ceil(umax)) + 5,
                                  static_cast<int>(std::ceil(vmax)) + 5};
  const cv::Mat mask = render(R, t, K, nullptr, box, face_on);
  cv::Mat dt;
  cv::distanceTransform(mask, dt, cv::DIST_L2, 3);
  keep.clear();
  for (int i = 0; i < P.rows(); ++i) {
    const double u = P(i, 0) / P(i, 2) * K(0, 0) + K(0, 2);
    const double v = P(i, 1) / P(i, 2) * K(1, 1) + K(1, 2);
    const int ui = clampv(static_cast<int>(rnd(u - box[0])), 0, mask.cols - 1);
    const int vi = clampv(static_cast<int>(rnd(v - box[1])), 0, mask.rows - 1);
    if (dt.at<float>(vi, ui) <= 1.5f) keep.push_back(i);
  }
  auto take = [&](const Idx& idx) {
    Pts3 X2(idx.size(), 3), P2(idx.size(), 3);
    Idx e2(idx.size());
    for (std::size_t j = 0; j < idx.size(); ++j) {
      X2.row(j) = X.row(idx[j]);
      P2.row(j) = P.row(idx[j]);
      e2[j] = eid[idx[j]];
    }
    X = std::move(X2);
    P = std::move(P2);
    eid = std::move(e2);
  };
  take(keep);
  if (P.rows() == 0) return;
  const Pts2 px = project(P, to_cv(K), to_cv(dist));
  keep.clear();
  for (int i = 0; i < P.rows(); ++i) {
    const int u = static_cast<int>(rnd(px(i, 0))), v = static_cast<int>(rnd(px(i, 1)));
    double zs = 0.0;
    if (u >= 0 && u < depth.cols && v >= 0 && v < depth.rows) zs = depth_at(depth, v, u);
    const bool hidden = (zs > 0.0) && (zs < P(i, 2) - 0.003);
    if (!hidden) keep.push_back(i);
  }
  take(keep);
}

// ------------------------------------------------------------------ scene

// _segment
bool Estimator::segment(const cv::Mat& depth, const Mat3& K, const VecX& dist,
                        const Mat4& prior, Quality& q, double err, Ctx& ctx) {
  const int h = depth.rows, w = depth.cols;
  const Mat3 R0 = prior.block<3, 3>(0, 0);
  const Vec3 t0 = prior.block<3, 1>(0, 3);
  const Pts3 Pv = transform(m_.V, R0, t0);
  double zmin = 1e300;
  for (int i = 0; i < Pv.rows(); ++i) {
    if (Pv(i, 2) <= 0.01) {
      q.reason = "prior behind the camera";
      return false;
    }
    zmin = std::min(zmin, Pv(i, 2));
  }
  const Pts2 uv = project(Pv, ctx.cvK, ctx.cvDist);
  const double search = std::max(p_.search_m, 4.0 * err);
  const int margin = static_cast<int>(std::ceil(search * K(0, 0) / zmin)) + 4;
  const double umin = uv.col(0).minCoeff(), umax = uv.col(0).maxCoeff();
  const double vmin = uv.col(1).minCoeff(), vmax = uv.col(1).maxCoeff();
  const int x0 = static_cast<int>(std::max(0.0, std::floor(umin) - margin));
  const int y0 = static_cast<int>(std::max(0.0, std::floor(vmin) - margin));
  const int x1 = static_cast<int>(std::min(static_cast<double>(w), std::ceil(umax) + margin + 1));
  const int y1 = static_cast<int>(std::min(static_cast<double>(h), std::ceil(vmax) + margin + 1));
  if (x1 - x0 < 8 || y1 - y0 < 8) {
    q.reason = "prior out of view";
    return false;
  }
  const std::array<int, 4> roi = {x0, y0, x1, y1};
  const int H = y1 - y0, W = x1 - x0;
  cv::Mat z;
  depth(cv::Rect(x0, y0, W, H)).convertTo(z, CV_64F);
  cv::Mat valid(H, W, CV_8U);
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) {
      double& zz = z.at<double>(y, x);
      const bool ok = std::isfinite(zz) && zz > 0.0;
      valid.at<uchar>(y, x) = ok;
      if (!ok) zz = 0.0;
    }
  const cv::Mat& grid = norm_grid(K, dist, h, w);          // undistorted x/z, y/z
  auto nrm = [&](int y, int x) { return grid.at<cv::Vec2d>(y0 + y, x0 + x); };

  const std::vector<char> face_on = facing(R0, t0);
  const cv::Mat prior_mask = render(R0, t0, K, &dist, roi, face_on);
  int prior_n = 0, bx0 = 1 << 30, bx1 = -1, by0 = 1 << 30, by1 = -1;
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x)
      if (prior_mask.at<uchar>(y, x)) {
        ++prior_n;
        bx0 = std::min(bx0, x);
        bx1 = std::max(bx1, x);
        by0 = std::min(by0, y);
        by1 = std::max(by1, y);
      }
  if (prior_n == 0) {
    q.reason = "prior renders empty";
    return false;
  }
  bx0 += x0;
  bx1 += x0;
  by0 += y0;
  by1 += y0;

  // the support plane, from sparse depth around the prior's silhouette
  const int g = std::max(3, margin / 2);
  const int G = std::max(g + 8, static_cast<int>(0.6 * std::max(bx1 - bx0, by1 - by0)));
  const int wx0 = std::max(0, bx0 - G), wy0 = std::max(0, by0 - G);
  const int wx1 = std::min(w, bx1 + G + 1), wy1 = std::min(h, by1 + G + 1);
  std::vector<Vec3> ring;
  for (int yy = wy0; yy < wy1; yy += 4)
    for (int xx = wx0; xx < wx1; xx += 4) {
      const double zz = depth_at(depth, yy, xx);
      const bool inside = xx >= bx0 - g && xx <= bx1 + g && yy >= by0 - g && yy <= by1 + g;
      if (std::isfinite(zz) && zz > 0.0 && !inside) {
        const cv::Vec2d gg = grid.at<cv::Vec2d>(yy, xx);
        ring.emplace_back(gg[0] * zz, gg[1] * zz, zz);
      }
    }
  cv::Mat cand = valid.clone();
  if (ring.size() >= 100) {
    std::vector<Vec3> pts;
    if (ring.size() > 800) {                     // np.linspace(0, n - 1, 800).astype(int)
      const double step = static_cast<double>(ring.size() - 1) / 799.0;
      pts.reserve(800);
      for (int i = 0; i < 800; ++i)
        pts.push_back(ring[i == 799 ? ring.size() - 1
                                    : static_cast<std::size_t>(static_cast<double>(i) * step)]);
    } else {
      pts = ring;
    }
    const auto fit = fit_plane_robust(pts);
    if (fit) {
      Vec3 n = fit->n;
      const Vec3 c = fit->c;
      if (n(2) > 0.0) n = -n;                    // toward the camera
      if ((t0 - c).dot(n) > p_.support_band_m + 0.002) {
        const double nc = n.dot(c);
        for (int y = 0; y < H; ++y)
          for (int x = 0; x < W; ++x) {
            const cv::Vec2d gg = nrm(y, x);
            const double a = gg[0] * n(0) + gg[1] * n(1) + n(2);
            if (!(z.at<double>(y, x) * a - nc > p_.support_band_m)) cand.at<uchar>(y, x) = 0;
          }
        q.has_support = true;
        q.support_n = n;
        q.support_d = nc;
      }
    }
  }

  // depths the part can show at the prior, with a margin for its error
  std::set<int> vis;
  for (int f = 0; f < static_cast<int>(face_on.size()); ++f)
    if (face_on[f])
      for (int k = 0; k < 3; ++k) vis.insert(m_.F(f, k));
  double zvmin = 1e300, zvmax = -1e300;
  for (int v : vis) {
    zvmin = std::min(zvmin, Pv(v, 2));
    zvmax = std::max(zvmax, Pv(v, 2));
  }
  const double band = p_.depth_band_m + search * 0.5;
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) {
      const double zz = z.at<double>(y, x);
      if (!(zz > zvmin - band && zz < zvmax + band)) cand.at<uchar>(y, x) = 0;
    }

  // split at depth jumps, dropping the farther pixel of each
  cv::Mat jump = cv::Mat::zeros(H, W, CV_8U);
  const double lim = p_.jump_ratio / K(0, 0);
  for (int y = 0; y < H; ++y)
    for (int x = 0; x + 1 < W; ++x) {
      const double za = z.at<double>(y, x), zb = z.at<double>(y, x + 1);
      const double dx = zb - za;
      if (std::abs(dx) > lim * std::min(zb, za) && valid.at<uchar>(y, x + 1) &&
          valid.at<uchar>(y, x)) {
        if (dx > 0) jump.at<uchar>(y, x + 1) = 1;
        if (dx < 0) jump.at<uchar>(y, x) = 1;
      }
    }
  for (int y = 0; y + 1 < H; ++y)
    for (int x = 0; x < W; ++x) {
      const double za = z.at<double>(y, x), zb = z.at<double>(y + 1, x);
      const double dy = zb - za;
      if (std::abs(dy) > lim * std::min(zb, za) && valid.at<uchar>(y + 1, x) &&
          valid.at<uchar>(y, x)) {
        if (dy > 0) jump.at<uchar>(y + 1, x) = 1;
        if (dy < 0) jump.at<uchar>(y, x) = 1;
      }
    }
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x)
      if (jump.at<uchar>(y, x)) cand.at<uchar>(y, x) = 0;

  cv::Mat lab;
  const int n_lab = cv::connectedComponents(cand, lab, 8, CV_32S);
  if (n_lab < 2) {
    q.reason = "no depth at the part";
    return false;
  }
  std::vector<long> overlap(n_lab, 0);
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x)
      if (prior_mask.at<uchar>(y, x)) ++overlap[lab.at<int>(y, x)];
  overlap[0] = 0;
  const int best = static_cast<int>(std::max_element(overlap.begin(), overlap.end()) -
                                    overlap.begin());
  if (overlap[best] == 0) {
    q.reason = "nothing where the prior is";
    return false;
  }
  cv::Mat region(H, W, CV_8U);
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) region.at<uchar>(y, x) = lab.at<int>(y, x) == best;
  // fill its holes (a marker's black cells give no depth)
  cv::Mat pad = cv::Mat::zeros(H + 2, W + 2, CV_8U);
  region.copyTo(pad(cv::Rect(1, 1, W, H)));
  cv::Mat ffmask = cv::Mat::zeros(H + 4, W + 4, CV_8U);
  cv::floodFill(pad, ffmask, cv::Point(0, 0), cv::Scalar(1));
  cv::Mat filled = region.clone();
  long mask_n = 0;
  cv::Mat mask(H, W, CV_8U);
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) {
      if (pad.at<uchar>(y + 1, x + 1) == 0) filled.at<uchar>(y, x) = 1;
      const bool m = (filled.at<uchar>(y, x) && cand.at<uchar>(y, x)) || region.at<uchar>(y, x);
      mask.at<uchar>(y, x) = m;
      mask_n += m;
    }
  const double ratio = static_cast<double>(mask_n) / std::max(1, prior_n);
  q.size_ratio = ratio;
  if (!(p_.size_lo <= ratio && ratio <= p_.size_hi)) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "size %.2f of the expected", ratio);
    q.reason = buf;
    return false;
  }

  // about max_points depth points: every k-th pixel both ways
  const int k = std::max(
      1, static_cast<int>(std::ceil(std::sqrt(static_cast<double>(mask_n) / p_.max_points))));
  std::vector<Vec3> S;
  for (int y = 0; y < H; y += k)
    for (int x = 0; x < W; x += k)
      if (mask.at<uchar>(y, x)) {
        const double zz = z.at<double>(y, x);
        const cv::Vec2d gg = nrm(y, x);
        S.emplace_back(gg[0] * zz, gg[1] * zz, zz);
      }
  ctx.S.resize(S.size(), 3);
  for (std::size_t i = 0; i < S.size(); ++i) ctx.S.row(i) = S[i].transpose();

  // the outline: the outer contour, outward normals from its own shape
  std::vector<std::vector<cv::Point>> cont;
  cv::findContours(filled, cont, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  std::vector<std::array<double, 2>> cs, ns;
  for (const auto& cc : cont) {
    const int n = static_cast<int>(cc.size());
    if (n < 8) continue;
    for (int i = 0; i < n; ++i) {
      double tx = 0.0, ty = 0.0;
      for (int j = 1; j <= 3; ++j) {
        const cv::Point& a = cc[(i + j) % n];
        const cv::Point& b = cc[((i - j) % n + n) % n];
        tx = tx + (static_cast<double>(a.x) - b.x);
        ty = ty + (static_cast<double>(a.y) - b.y);
      }
      cs.push_back({static_cast<double>(cc[i].x), static_cast<double>(cc[i].y)});
      ns.push_back({ty, -tx});
    }
  }
  if (cs.empty()) {
    q.reason = "no outline";
    return false;
  }
  std::vector<std::array<double, 2>> c2, n2;
  for (std::size_t i = 0; i < cs.size(); ++i) {
    const double gn = std::sqrt(ns[i][0] * ns[i][0] + ns[i][1] * ns[i][1]);
    if (!(gn > 1e-9)) continue;
    std::array<double, 2> n = {ns[i][0] / gn, ns[i][1] / gn};
    const int ix = clampv(static_cast<int>(rnd(cs[i][0] + 1.5 * n[0])), 0, W - 1);
    const int iy = clampv(static_cast<int>(rnd(cs[i][1] + 1.5 * n[1])), 0, H - 1);
    if (filled.at<uchar>(iy, ix) > 0) {            // outward must leave the region
      n[0] *= -1.0;
      n[1] *= -1.0;
    }
    const int cx = static_cast<int>(cs[i][0]), cy = static_cast<int>(cs[i][1]);
    if (!(cx > 1 && cx < W - 2 && cy > 1 && cy < H - 2)) continue;   // not the ROI border
    const int px = clampv(static_cast<int>(rnd(cs[i][0] + 3.0 * n[0])), 0, W - 1);
    const int py = clampv(static_cast<int>(rnd(cs[i][1] + 3.0 * n[1])), 0, H - 1);
    const double z_in = z.at<double>(cy, cx), z_out = z.at<double>(py, px);
    if (valid.at<uchar>(py, px) && z_out < z_in - 0.003) continue;  // an occluder's edge
    c2.push_back(cs[i]);
    n2.push_back(n);
  }
  if (c2.size() < 20) {
    q.reason = "outline occluded";
    return false;
  }
  Pts2 full(c2.size(), 2), full2(c2.size(), 2);
  for (std::size_t i = 0; i < c2.size(); ++i) {
    full(i, 0) = c2[i][0] + x0 + p_.outline_offset_px * n2[i][0];
    full(i, 1) = c2[i][1] + y0 + p_.outline_offset_px * n2[i][1];
    full2(i, 0) = full(i, 0) + 2.0 * n2[i][0];
    full2(i, 1) = full(i, 1) + 2.0 * n2[i][1];
  }
  ctx.out_uv = undistort_px(full, ctx.cvK, ctx.cvDist);
  const Pts2 pin2 = undistort_px(full2, ctx.cvK, ctx.cvDist);
  ctx.out_n.resize(c2.size(), 2);
  for (std::size_t i = 0; i < c2.size(); ++i) {
    const double ux = pin2(i, 0) - ctx.out_uv(i, 0), uy = pin2(i, 1) - ctx.out_uv(i, 1);
    const double nn = std::max(std::sqrt(ux * ux + uy * uy), 1e-12);
    ctx.out_n(i, 0) = ux / nn;
    ctx.out_n(i, 1) = uy / nn;
  }
  ctx.roi = roi;
  return true;
}

// ------------------------------------------------------------------ solve

// _gn_step: the Gauss-Newton update, at most 5 mm / 5 deg
Vec6 Estimator::gn_step(const Rows& J, const VecX& r, const VecX& w) {
  Mat6 H = Mat6::Zero();
  Vec6 b = Vec6::Zero();
  for (int i = 0; i < J.rows(); ++i) {
    const Vec6 j = J.row(i).transpose();
    H.noalias() += w(i) * j * j.transpose();
    b += j * (w(i) * r(i));
  }
  H += Mat6::Identity() * 1e-9 * std::max(H.trace(), 1e-12);
  Vec6 x = -H.partialPivLu().solve(b);
  const double s = std::min({1.0, 0.005 / std::max(x.tail<3>().norm(), 1e-12),
                             rad(5.0) / std::max(x.head<3>().norm(), 1e-12)});
  return x * s;
}

// _apply: T <- T A^-1, A = (rotvec x[:3], x[3:])
Mat4 Estimator::apply(const Mat4& T, const Vec6& x) {
  const Vec3 w = x.head<3>();
  const double th = w.norm();
  Mat3 Ra = Mat3::Identity();
  if (th >= std::numeric_limits<double>::epsilon()) {           // cv::Rodrigues
    const Vec3 k = w / th;
    const double c = std::cos(th), s = std::sin(th), c1 = 1.0 - c;
    Ra = c * Mat3::Identity() + c1 * k * k.transpose() + s * skew(k);
  }
  Mat4 A_inv = Mat4::Identity();
  A_inv.block<3, 3>(0, 0) = Ra.transpose();
  A_inv.block<3, 1>(0, 3) = -Ra.transpose() * x.tail<3>();
  return T * A_inv;
}

// _edge_image: the colour image around the ROI, lightly smoothed, as float
Estimator::EdgeImg Estimator::edge_image(const cv::Mat& bgr, const std::array<int, 4>& roi,
                                         int margin) {
  const int h = bgr.rows, w = bgr.cols;
  const int x0 = std::max(0, roi[0] - margin), y0 = std::max(0, roi[1] - margin);
  const int x1 = std::min(w, roi[2] + margin), y1 = std::min(h, roi[3] + margin);
  cv::Mat f;
  bgr(cv::Rect(x0, y0, x1 - x0, y1 - y0)).convertTo(f, CV_32F);
  EdgeImg out;
  cv::GaussianBlur(f, out.img, cv::Size(0, 0), 0.7);
  out.ox = x0;
  out.oy = y0;
  return out;
}

// _outline_jac: rows d(ns . uv)/d(w, v) * scale
void Estimator::outline_jac(const Pts3& Xo, const Pts3& Po, const Pts2& ns, const Mat3& R,
                            const Mat3& K, const VecX& scale, Rows& J) {
  J.resize(Po.rows(), 6);
  for (int i = 0; i < Po.rows(); ++i) {
    const double zo = Po(i, 2);
    Eigen::Matrix<double, 2, 3> Jpi = Eigen::Matrix<double, 2, 3>::Zero();
    Jpi(0, 0) = K(0, 0) / zo;
    Jpi(0, 2) = -K(0, 0) * Po(i, 0) / (zo * zo);
    Jpi(1, 1) = K(1, 1) / zo;
    Jpi(1, 2) = -K(1, 1) * Po(i, 1) / (zo * zo);
    const Eigen::RowVector3d a = (ns.row(i) * Jpi) * scale(i);
    const Mat3 dPw = R * skew(Xo.row(i).transpose());
    J.block<1, 3>(i, 0) = a * dPw;
    J.block<1, 3>(i, 3) = -a * R;
  }
}

// _surface_pairs: each scene point with a camera-facing sample within gate_s
Estimator::Pairs Estimator::surface_pairs(const Ctx& ctx, const Mat3& R, const Vec3& t,
                                          double gate_s) const {
  Pairs out;
  std::vector<std::pair<int, int>> found;
  for (int i = 0; i < ctx.S.rows(); ++i) {
    const Vec3 y = R.transpose() * (ctx.S.row(i).transpose() - t);
    double d2;
    const int j = tree_.nearest(y.data(), gate_s, &d2);
    if (j < 0) continue;
    const Vec3 m = m_.samples.row(j).transpose(), nm = m_.sample_n.row(j).transpose();
    if ((R * nm).dot(R * m + t) < 0.0) found.emplace_back(i, j);
  }
  out.si.resize(found.size());
  out.m.resize(found.size(), 3);
  out.n.resize(found.size(), 3);
  for (std::size_t k = 0; k < found.size(); ++k) {
    out.si[k] = found[k].first;
    out.m.row(k) = m_.samples.row(found[k].second);
    out.n.row(k) = m_.sample_n.row(found[k].second);
  }
  return out;
}

// _surface_rows
void Estimator::surface_rows(const Ctx& ctx, const Mat3& R, const Vec3& t, const Pairs& pairs,
                             Rows& J, VecX& r) {
  const int n = static_cast<int>(pairs.si.size());
  J.resize(n, 6);
  r.resize(n);
  for (int k = 0; k < n; ++k) {
    const Vec3 y = R.transpose() * (ctx.S.row(pairs.si[k]).transpose() - t);
    const Vec3 nk = pairs.n.row(k).transpose();
    J.block<1, 3>(k, 0) = y.cross(nk).transpose();
    J.block<1, 3>(k, 3) = nk.transpose();
    r(k) = (y - pairs.m.row(k).transpose()).dot(nk);
  }
}

// _depth_outline_rows: silhouette points against the depth region's outline
void Estimator::depth_outline_rows(const Pts3& X, const Pts3& P, const Mat3& R, const Ctx& ctx,
                                   double gate_o, Rows& J, VecX& r, VecX& w) const {
  const Mat3& K = ctx.K;
  const double f = ctx.f;
  const int n = static_cast<int>(P.rows());
  Pts2 uv(n, 2);
  VecX gpx(n);
  double gmax = 0.0;
  for (int i = 0; i < n; ++i) {
    uv(i, 0) = P(i, 0) / P(i, 2) * K(0, 0) + K(0, 2);
    uv(i, 1) = P(i, 1) / P(i, 2) * K(1, 1) + K(1, 2);
    gpx(i) = gate_o * f / P(i, 2);
    gmax = std::max(gmax, gpx(i));
  }
  Idx ok, jo;
  for (int i = 0; i < n; ++i) {
    double d2;
    const int j = ctx.out_tree.nearest(uv.row(i).data(), gmax, &d2);
    if (j >= 0 && std::sqrt(d2) <= gpx(i)) {
      ok.push_back(i);
      jo.push_back(j);
    }
  }
  const int m = static_cast<int>(ok.size());
  Pts3 Xo(m, 3), Po(m, 3);
  Pts2 ns(m, 2);
  VecX scale(m);
  r.resize(m);
  w.resize(m);
  for (int k = 0; k < m; ++k) {
    Xo.row(k) = X.row(ok[k]);
    Po.row(k) = P.row(ok[k]);
    ns.row(k) = ctx.out_n.row(jo[k]);
    scale(k) = Po(k, 2) / f;
    r(k) = ((uv(ok[k], 0) - ctx.out_uv(jo[k], 0)) * ns(k, 0) +
            (uv(ok[k], 1) - ctx.out_uv(jo[k], 1)) * ns(k, 1)) * scale(k);
    w(k) = tukey(r(k), gate_o);
  }
  outline_jac(Xo, Po, ns, R, K, scale, J);
}

// _part_like: candidates whose inner colour is a major part colour or a
// blend of two of the main ones
std::vector<char> Estimator::part_like(const std::vector<std::array<float, 3>>& pool,
                                       const std::vector<float>& inner, int n_rows, int n_cols,
                                       const std::vector<char>& pk) const {
  const double share = 0.03, tol = 24.0;
  const int max_colours = 12, blend_colours = 6;
  std::vector<char> out(pk.size(), 0);
  // the colour histogram, 16 levels per channel, and its exact 3x3x3 box sum
  std::vector<long> hist(16 * 16 * 16, 0), tmp(hist.size());
  for (const auto& c : pool) {
    int b[3];
    for (int k = 0; k < 3; ++k)
      b[k] = clampv(static_cast<int>(std::floor(c[k] / 16.0f)), 0, 15);
    ++hist[b[0] * 256 + b[1] * 16 + b[2]];
  }
  const int stride[3] = {256, 16, 1};
  for (int ax = 0; ax < 3; ++ax) {
    for (int i = 0; i < 16; ++i)
      for (int j = 0; j < 16; ++j)
        for (int k = 0; k < 16; ++k) {
          const int idx = i * 256 + j * 16 + k;
          const int pos = ax == 0 ? i : ax == 1 ? j : k;
          long s = hist[idx];
          if (pos > 0) s += hist[idx - stride[ax]];
          if (pos < 15) s += hist[idx + stride[ax]];
          tmp[idx] = s;
        }
    hist.swap(tmp);
  }
  const double thr = std::max(3.0, share * static_cast<double>(pool.size()));
  std::vector<int> major;
  for (int i = 0; i < 4096; ++i)
    if (static_cast<double>(hist[i]) >= thr) major.push_back(i);
  if (major.empty()) return out;
  std::stable_sort(major.begin(), major.end(), [&](int a, int b) { return hist[a] > hist[b]; });
  if (static_cast<int>(major.size()) > max_colours) major.resize(max_colours);
  std::vector<std::array<double, 3>> C;
  for (int b : major)
    C.push_back({(b / 256 + 0.5) * 16.0, ((b / 16) % 16 + 0.5) * 16.0, (b % 16 + 0.5) * 16.0});
  const int nb = std::min(static_cast<int>(C.size()), blend_colours);
  const double tol2 = tol * tol;
  for (int rr = 0; rr < n_rows; ++rr)
    for (int cc = 0; cc < n_cols; ++cc) {
      const int pi = rr * n_cols + cc;
      if (!pk[pi]) continue;
      const float* c = &inner[static_cast<std::size_t>(pi) * 3];
      bool ok = false;
      for (const auto& Ck : C) {
        double s = 0.0;
        for (int ch = 0; ch < 3; ++ch) {
          const double e = static_cast<double>(c[ch]) - Ck[ch];
          s += e * e;
        }
        if (s <= tol2) {
          ok = true;
          break;
        }
      }
      if (!ok) {                                 // blends of two main colours
        for (int i = 0; i < nb && !ok; ++i)
          for (int j = i + 1; j < nb && !ok; ++j) {
            double ab[3], ca[3], abab = 0.0, dot = 0.0;
            for (int ch = 0; ch < 3; ++ch) {
              ab[ch] = C[j][ch] - C[i][ch];
              ca[ch] = static_cast<double>(c[ch]) - C[i][ch];
              abab += ab[ch] * ab[ch];
              dot += ca[ch] * ab[ch];
            }
            const double tt = clampv(dot / std::max(abab, 1e-9), 0.0, 1.0);
            double s = 0.0;
            for (int ch = 0; ch < 3; ++ch) {
              const double e = static_cast<double>(c[ch]) - (C[i][ch] + tt * ab[ch]);
              s += e * e;
            }
            ok = s <= tol2;
          }
      }
      out[pi] = ok;
    }
  return out;
}

// _find_edges: from each silhouette point, along its outward normal, the
// OUTERMOST clear colour edge that looks like the part on its inner side
Estimator::Obs Estimator::find_edges(const Pts3& X, const Idx& eid, const Mat3& R,
                                     const Vec3& t, const Ctx& ctx, const EdgeImg& img,
                                     int win_in, int win_out) const {
  Obs none;
  const int n = static_cast<int>(X.rows());
  if (n == 0) return none;
  const Pts3 P = transform(X, R, t);
  const Pts2 uv = project(P, ctx.cvK, ctx.cvDist);
  Pts3 P2(n, 3);
  for (int i = 0; i < n; ++i) {
    const Vec3 d = R * m_.edge_dir.row(eid[i]).transpose();
    P2.row(i) = P.row(i) + 0.001 * d.transpose();
  }
  const Pts2 uv2 = project(P2, ctx.cvK, ctx.cvDist);
  Pts2 nrm(n, 2);
  const std::vector<char> face_on = facing(R, t);
  Pts3 Fc(n, 3);
  for (int i = 0; i < n; ++i) {
    double tx = uv2(i, 0) - uv(i, 0), ty = uv2(i, 1) - uv(i, 1);
    const double tn = std::max(std::sqrt(tx * tx + ty * ty), 1e-12);
    tx /= tn;
    ty /= tn;
    nrm(i, 0) = ty;
    nrm(i, 1) = -tx;
    const int f0 = m_.edge_faces(eid[i], 0), f1 = m_.edge_faces(eid[i], 1);
    const int front = face_on[f0] ? f0 : std::max(f1, 0);
    Fc.row(i) = (R * m_.face_c.row(front).transpose() + t).transpose();
  }
  const Pts2 uvc = project(Fc, ctx.cvK, ctx.cvDist);
  for (int i = 0; i < n; ++i) {
    if ((uv(i, 0) - uvc(i, 0)) * nrm(i, 0) + (uv(i, 1) - uvc(i, 1)) * nrm(i, 1) < 0.0) {
      nrm(i, 0) *= -1.0;
      nrm(i, 1) *= -1.0;
    }
  }
  // samples along each normal: the window, and 3-7 px further in
  const int L = win_in + win_out + 10;            // s = -win_in-8 .. win_out+1
  cv::Mat mx(n, L, CV_32F), my(n, L, CV_32F);
  for (int i = 0; i < n; ++i)
    for (int k = 0; k < L; ++k) {
      const double s = static_cast<double>(-win_in - 8 + k);
      mx.at<float>(i, k) = static_cast<float>(uv(i, 0) - img.ox + nrm(i, 0) * s);
      my.at<float>(i, k) = static_cast<float>(uv(i, 1) - img.oy + nrm(i, 1) * s);
    }
  cv::Mat smp;
  cv::remap(img.img, smp, mx, my, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
  auto S = [&](int i, int k) { return smp.at<cv::Vec3f>(i, k); };
  const int nM = L - 9;                           // offsets -win_in .. win_out
  const int nP = nM - 2;
  std::vector<float> M(static_cast<std::size_t>(n) * nM);
  std::vector<char> pk(static_cast<std::size_t>(n) * nP, 0);
  for (int i = 0; i < n; ++i) {
    float mmax = -1.0f;
    for (int j = 0; j < nM; ++j) {
      const cv::Vec3f a = S(i, 7 + j + 2), b = S(i, 7 + j);
      float acc = 0.0f;
      for (int ch = 0; ch < 3; ++ch) {
        const float e = (a[ch] - b[ch]) * 0.5f;
        acc = acc + e * e;
      }
      const float v = std::sqrt(acc);
      M[static_cast<std::size_t>(i) * nM + j] = v;
      mmax = std::max(mmax, v);
    }
    const float thr = std::max(static_cast<float>(p_.edge_min_step),
                               static_cast<float>(p_.edge_rel) * mmax);
    const float* Mi = &M[static_cast<std::size_t>(i) * nM];
    for (int p = 0; p < nP; ++p) {
      const float c = Mi[p + 1];
      pk[static_cast<std::size_t>(i) * nP + p] = c >= Mi[p] && c > Mi[p + 2] && c >= thr;
    }
  }
  // the part's colours: the deep samples (offsets 1..5) all round; the inner
  // colour of each candidate: 2-3 px inside it
  std::vector<std::array<float, 3>> pool;
  pool.reserve(static_cast<std::size_t>(n) * 5);
  for (int i = 0; i < n; ++i)
    for (int k = 1; k < 6; ++k) {
      const cv::Vec3f v = S(i, k);
      pool.push_back({v[0], v[1], v[2]});
    }
  std::vector<float> inner(static_cast<std::size_t>(n) * nP * 3);
  for (int i = 0; i < n; ++i)
    for (int p = 0; p < nP; ++p) {
      const cv::Vec3f a = S(i, 6 + p), b = S(i, 7 + p);
      for (int ch = 0; ch < 3; ++ch)
        inner[(static_cast<std::size_t>(i) * nP + p) * 3 + ch] = 0.5f * (a[ch] + b[ch]);
    }
  const std::vector<char> like = part_like(pool, inner, n, nP, pk);
  Obs out;
  std::vector<std::array<double, 2>> e_uv, e_n;
  for (int i = 0; i < n; ++i) {
    int last = -1;
    for (int p = 0; p < nP; ++p)
      if (pk[static_cast<std::size_t>(i) * nP + p] && like[static_cast<std::size_t>(i) * nP + p])
        last = p;
    if (last < 0) continue;
    const int k = last + 1;                        // index into M
    const float* Mi = &M[static_cast<std::size_t>(i) * nM];
    const float m0 = Mi[k - 1], m1 = Mi[k], m2 = Mi[k + 1];
    const float den = m0 - 2.0f * m1 + m2;
    float delta = 0.0f;
    if (std::abs(den) > 1e-9f) delta = 0.5f * (m0 - m2) / den;
    delta = clampv(delta, -0.5f, 0.5f);
    const double off = static_cast<double>(k - win_in) + static_cast<double>(delta);
    out.idx.push_back(i);
    e_uv.push_back({uv(i, 0) + off * nrm(i, 0), uv(i, 1) + off * nrm(i, 1)});
    e_n.push_back({nrm(i, 0), nrm(i, 1)});
  }
  out.e_uv.resize(e_uv.size(), 2);
  out.nrm.resize(e_n.size(), 2);
  for (std::size_t j = 0; j < e_uv.size(); ++j) {
    out.e_uv(j, 0) = e_uv[j][0];
    out.e_uv(j, 1) = e_uv[j][1];
    out.nrm(j, 0) = e_n[j][0];
    out.nrm(j, 1) = e_n[j][1];
  }
  return out;
}

// _edge_obs_rows: silhouette points against FIXED edge points
void Estimator::edge_obs_rows(const Pts3& X, const Obs& obs, const Mat3& R, const Vec3& t,
                              const Ctx& ctx, double c_px, Rows& J, VecX& r, VecX& w) const {
  const int m = static_cast<int>(obs.idx.size());
  Pts3 Xo(m, 3);
  for (int k = 0; k < m; ++k) Xo.row(k) = X.row(obs.idx[k]);
  const Pts3 Po = transform(Xo, R, t);
  const Pts2 uv = project(Po, ctx.cvK, ctx.cvDist);
  VecX scale(m);
  r.resize(m);
  w.resize(m);
  for (int k = 0; k < m; ++k) {
    const double e_px = (uv(k, 0) - obs.e_uv(k, 0)) * obs.nrm(k, 0) +
                        (uv(k, 1) - obs.e_uv(k, 1)) * obs.nrm(k, 1);
    scale(k) = Po(k, 2) / ctx.f;
    r(k) = e_px * scale(k);
    w(k) = tukey(e_px, c_px);
  }
  outline_jac(Xo, Po, obs.nrm, R, ctx.K, scale, J);
}

// _fixed_rows: both terms with their correspondences held fixed
void Estimator::fixed_rows(const Ctx& ctx, const Mat4& T, const Pairs& pairs, const Pts3& X,
                           const Obs& obs, double c_px, Rows& J, VecX& r, VecX& w) const {
  const Mat3 R = T.block<3, 3>(0, 0);
  const Vec3 t = T.block<3, 1>(0, 3);
  Rows Js, Jo;
  VecX rs, ro, wo;
  surface_rows(ctx, R, t, pairs, Js, rs);
  edge_obs_rows(X, obs, R, t, ctx, c_px, Jo, ro, wo);
  J.resize(Js.rows() + Jo.rows(), 6);
  J << Js, Jo;
  r.resize(rs.size() + ro.size());
  r << rs, ro;
  w.resize(r.size());
  for (int i = 0; i < rs.size(); ++i) w(i) = tukey(rs(i), 0.003);
  for (int i = 0; i < ro.size(); ++i) w(rs.size() + i) = p_.outline_weight * wo(i);
}

// _rows: stacked rows of both terms at R, t
Estimator::RowsOut Estimator::rows(const Ctx& ctx, const Mat3& R, const Vec3& t, double gate_s,
                                   double gate_o, Quality& q, const Rim* rim_in,
                                   const EdgeImg* edges, int win_in, int win_out,
                                   const Obs* obs_in, bool final_stats, bool outline) const {
  RowsOut out;
  const Pairs pairs = surface_pairs(ctx, R, t, gate_s);
  Rows Js;
  VecX rs;
  surface_rows(ctx, R, t, pairs, Js, rs);
  Rows Jo(0, 6);
  VecX ro(0), wo(0);
  Pts3 X(0, 3);
  Idx eid;
  if (p_.use_outline && outline) {
    Pts3 P;
    if (rim_in != nullptr) {
      X = rim_in->X;
      eid = rim_in->eid;
      P = transform(X, R, t);
    } else {
      rim(R, t, ctx.K, ctx.dist, *ctx.depth, X, P, eid);
    }
    if (X.rows() > 0) {
      if (edges != nullptr) {
        Obs found;
        const Obs* obs = obs_in;
        if (obs == nullptr) {
          found = find_edges(X, eid, R, t, ctx, *edges, win_in, win_out);
          obs = &found;
        }
        edge_obs_rows(X, *obs, R, t, ctx, std::max(win_in, win_out) + 1.0, Jo, ro, wo);
      } else {
        depth_outline_rows(X, P, R, ctx, gate_o, Jo, ro, wo);
      }
    }
  }
  if (final_stats) {
    if (rs.size() > 0) {
      double ss = 0.0;
      for (int i = 0; i < rs.size(); ++i) ss += rs(i) * rs(i);
      q.rms_mm = std::sqrt(ss / static_cast<double>(rs.size())) * 1e3;
    } else {
      q.rms_mm.reset();
    }
    q.inlier_frac = static_cast<double>(rs.size()) / std::max<Eigen::Index>(1, ctx.S.rows());
    q.outline_frac = static_cast<double>(ro.size()) / std::max<Eigen::Index>(1, X.rows());
  }
  out.used.X = X;
  out.used.eid = eid;
  if (rs.size() + ro.size() < 12) {
    q.reason = "too few correspondences";
    return out;
  }
  out.J.resize(Js.rows() + Jo.rows(), 6);
  out.J << Js, Jo;
  out.r.resize(rs.size() + ro.size());
  out.r << rs, ro;
  out.w.resize(out.r.size());
  for (int i = 0; i < rs.size(); ++i) out.w(i) = tukey(rs(i), gate_s);
  for (int i = 0; i < ro.size(); ++i) out.w(rs.size() + i) = p_.outline_weight * wo(i);
  out.ok = true;
  return out;
}

// _solve: stage A, Gauss-Newton with shrinking gates
bool Estimator::solve(const Mat4& T0, const Ctx& ctx, Quality& q, bool coarse, double err,
                      bool outline, Mat4& T, std::optional<Rim>& rim_out) const {
  T = T0;
  const double tol_t = coarse ? 5e-5 : 1e-5;
  const double tol_r = coarse ? rad(0.05) : rad(0.01);
  rim_out.reset();
  const int n_it = coarse ? 8 : p_.max_iter;
  for (int it = 0; it < n_it; ++it) {
    const Mat3 R = T.block<3, 3>(0, 0);
    const Vec3 t = T.block<3, 1>(0, 3);
    const double gate_s = std::max(0.003, 2.0 * err * std::pow(0.6, it));
    const double gate_o = std::max(0.003, 6.0 * err * std::pow(0.6, it));
    const RowsOut ro = rows(ctx, R, t, gate_s, gate_o, q, rim_out ? &*rim_out : nullptr,
                            nullptr, 0, 0, nullptr, false, outline);
    if (outline && !rim_out && gate_s <= 0.003 && gate_o <= 0.003) rim_out = ro.used;
    if (!ro.ok) return false;
    const Vec6 x = gn_step(ro.J, ro.r, ro.w);
    T = apply(T, x);
    ++q.iterations;
    if ((rim_out || !outline) && x.tail<3>().norm() < tol_t && x.head<3>().norm() < tol_r) break;
  }
  return true;
}

// _stage_b: colour-edge rounds, correspondences fixed within a round
bool Estimator::stage_b(Mat4& T, const Ctx& ctx, Quality& q, const Rim& rim, const EdgeImg& img,
                        double& frac, Obs& obs) const {
  frac = 0.0;
  const int wins[2][3] = {{8, 10, 4}, {3, 4, 3}};
  for (const auto& wv : wins) {
    const int win_in = wv[0], win_out = wv[1], steps = wv[2];
    const Mat3 R = T.block<3, 3>(0, 0);
    const Vec3 t = T.block<3, 1>(0, 3);
    obs = find_edges(rim.X, rim.eid, R, t, ctx, img, win_in, win_out);
    frac = static_cast<double>(obs.idx.size()) / std::max<Eigen::Index>(1, rim.X.rows());
    const Pairs pairs = surface_pairs(ctx, R, t, 0.003);
    if (obs.idx.size() + pairs.si.size() < 12) return false;
    for (int s = 0; s < steps; ++s) {
      Rows J;
      VecX r, w;
      fixed_rows(ctx, T, pairs, rim.X, obs, std::max(win_in, win_out) + 1.0, J, r, w);
      const Vec6 x = gn_step(J, r, w);
      T = apply(T, x);
      ++q.iterations;
      if (x.tail<3>().norm() < 1e-5 && x.head<3>().norm() < rad(0.01)) break;
    }
  }
  return true;
}

// process
Result Estimator::process(const cv::Mat& depth, const cv::Mat& bgr, const Mat3& K,
                          const VecX& dist, const Mat4& prior, double err) {
  Result res;
  Quality& q = res.q;
  Ctx ctx;
  ctx.K = K;
  ctx.dist = dist;
  ctx.cvK = to_cv(K);
  ctx.cvDist = to_cv(dist);
  ctx.depth = &depth;
  ctx.f = 0.5 * (K(0, 0) + K(1, 1));
  if (!segment(depth, K, dist, prior, q, err, ctx)) return res;
  q.n_pts = static_cast<int>(ctx.S.rows());
  ctx.out_tree.build(ctx.out_uv.data(), static_cast<int>(ctx.out_uv.rows()));

  // stage A
  const bool colour = p_.colour_edges && p_.use_outline && !bgr.empty();
  const bool near = colour && err * ctx.f / std::max(prior(2, 3), 0.05) <= 6.0;
  Mat4 T;
  std::optional<Rim> rim_opt;
  if (!solve(prior, ctx, q, colour, err, !near, T, rim_opt)) return res;
  bool have_edges = false;
  Obs obs;
  EdgeImg img;
  q.edge_source = p_.use_outline ? "depth" : "";
  // stage B
  if (colour) {
    Rim rb;
    Pts3 Pb;
    rim(T.block<3, 3>(0, 0), T.block<3, 1>(0, 3), K, dist, depth, rb.X, Pb, rb.eid);
    if (rb.X.rows() > 0) {
      img = edge_image(bgr, ctx.roi);
      Mat4 T_b = T;
      double frac = 0.0;
      Obs obs_b;
      if (stage_b(T_b, ctx, q, rb, img, frac, obs_b) && frac >= p_.min_outline_frac) {
        T = T_b;
        rim_opt = rb;
        obs = std::move(obs_b);
        have_edges = true;
        q.edge_source = "colour";
      }
    }
    if (!have_edges && !p_.depth_fallback) {
      q.reason = "no colour outline (depth fallback off)";
      return res;
    }
    if (!have_edges) {                             // finish stage A properly
      Mat4 T2;
      if (!solve(T, ctx, q, false, near ? err : 0.0015, true, T2, rim_opt)) return res;
      T = T2;
    }
  }

  // final statistics at the tight gates (the last colour round's edges)
  {
    const Mat3 R = T.block<3, 3>(0, 0);
    const Vec3 t = T.block<3, 1>(0, 3);
    const RowsOut ro = rows(ctx, R, t, 0.003, 0.003, q, rim_opt ? &*rim_opt : nullptr,
                            have_edges ? &img : nullptr, 3, 4, have_edges ? &obs : nullptr,
                            true, true);
    if (ro.ok) {
      const double L = std::max(m_.size_m / 2.0, 0.005);
      Vec6 dvec;
      dvec << 1.0 / L, 1.0 / L, 1.0 / L, 1.0, 1.0, 1.0;
      Mat6 H = Mat6::Zero();
      for (int i = 0; i < ro.J.rows(); ++i) {
        const Vec6 j = ro.J.row(i).transpose();
        H.noalias() += ro.w(i) * j * j.transpose();
      }
      const Mat6 Hs = dvec.asDiagonal() * H * dvec.asDiagonal();
      Eigen::SelfAdjointEigenSolver<Mat6> es(Hs);
      const Vec6 ev = es.eigenvalues();
      const double emax = ev.maxCoeff();
      std::set<int> weak;
      for (int i = 0; i < 6; ++i)
        if (ev(i) < p_.weak_rel * emax) {
          int arg = 0;
          es.eigenvectors().col(i).cwiseAbs().maxCoeff(&arg);
          weak.insert(arg);
        }
      q.weak_dof.assign(weak.begin(), weak.end());
    }
  }

  // symmetry: the equivalent pose nearest the prior's X
  if (m_.sym_order > 1) {
    double best = -2.0;
    int best_k = 0;
    for (int k = 0; k < m_.sym_order; ++k) {
      const double a = 2.0 * kPi * k / m_.sym_order;
      Mat3 Rz;
      Rz << std::cos(a), -std::sin(a), 0.0, std::sin(a), std::cos(a), 0.0, 0.0, 0.0, 1.0;
      const double d = (T.block<3, 3>(0, 0) * Rz).col(0).dot(prior.block<3, 1>(0, 0));
      if (d > best) {
        best = d;
        best_k = k;
      }
    }
    const double a = 2.0 * kPi * best_k / m_.sym_order;
    Mat4 Rz = Mat4::Identity();
    Rz(0, 0) = std::cos(a);
    Rz(0, 1) = -std::sin(a);
    Rz(1, 0) = std::sin(a);
    Rz(1, 1) = std::cos(a);
    T = T * Rz;
    q.sym_index = best_k;
  }

  const Mat4 D = prior.inverse() * T;
  q.agree_mm = D.block<3, 1>(0, 3).norm() * 1e3;
  q.agree_deg = deg(std::acos(clampv((D.block<3, 3>(0, 0).trace() - 1.0) / 2.0, -1.0, 1.0)));
  q.agree_tilt_deg = deg(std::acos(clampv(D(2, 2), -1.0, 1.0)));
  q.agree_inplane_deg = deg(std::atan2(D(1, 0), D(0, 0)));
  q.finished = true;
  q.reason.clear();
  res.has_T = true;
  res.T = T;
  return res;
}

}  // namespace object_pose
