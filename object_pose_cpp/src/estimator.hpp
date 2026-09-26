// The object pose estimator of roscam/object_pose.py (ObjectPoseEstimator
// .process), in C++. The algorithm, its constants and the order of its
// floating-point steps follow the Python function by function, so the two
// give the same answers; roscam/test/test_object_pose_cpp.py checks it. The
// model (mesh samples, silhouette edge points) comes prepared from Python.
#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "kdtree.hpp"

namespace object_pose {

using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using VecX = Eigen::VectorXd;
using Pts3 = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using Pts2 = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;
using Rows = Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor>;
using Idx = std::vector<int>;

// ObjectPoseEstimator's constructor arguments, as its attributes hold them.
struct Params {
  double search_m = 0.010;
  double prior_err_m = 0.005;
  double depth_band_m = 0.015;
  double support_band_m = 0.003;
  double jump_ratio = 0.0;       // tan(max_incidence_deg)
  double rim_min_cos = 0.0;      // cos(rim_max_incidence_deg)
  double outline_offset_px = 0.5;
  int max_iter = 30;
  int max_points = 600;
  double outline_weight = 0.1;
  bool use_outline = true;
  bool colour_edges = true;
  bool depth_fallback = true;
  double edge_min_step = 6.0;
  double edge_rel = 0.3;
  int min_points = 200;
  double size_lo = 0.5;
  double size_hi = 1.6;
  double weak_rel = 1e-3;
  double min_outline_frac = 0.6;
};

// The mesh as ObjectPoseEstimator prepared it.
struct Model {
  Pts3 V;
  Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> F;
  Pts3 face_n, face_c;
  Pts3 samples, sample_n;
  Eigen::Matrix<int, Eigen::Dynamic, 2, Eigen::RowMajor> edge_faces;
  Pts3 edge_dir;
  Pts3 edge_pts;
  Idx edge_id;
  int sym_order = 1;
  double size_m = 0.0;
};

// process()'s numbers; Python assembles the quality dict and the gates.
struct Quality {
  std::string reason;            // why it stopped early ('' = it finished)
  bool finished = false;         // reached the gates
  int n_pts = 0;
  std::optional<double> size_ratio, rms_mm, inlier_frac, outline_frac;
  Idx weak_dof;                  // into (rx, ry, rz, tx, ty, tz), sorted
  double agree_mm = 0.0, agree_deg = 0.0, agree_tilt_deg = 0.0, agree_inplane_deg = 0.0;
  int sym_index = 0;
  int iterations = 0;
  std::string edge_source;       // '', 'depth' or 'colour'
  bool has_support = false;
  Vec3 support_n = Vec3::Zero();
  double support_d = 0.0;
};

struct Result {
  bool has_T = false;
  Mat4 T = Mat4::Identity();
  Quality q;
};

class Estimator {
 public:
  Estimator(Model model, Params params);

  // The per-camera undistorted pixel grid, built now rather than in the
  // first frame.
  void warm(const Mat3& K, const VecX& dist, int h, int w);

  // depth: CV_32F or CV_64F metres; bgr: CV_8UC3, or empty for depth only;
  // prior: T_camera<-object (4x4); err: how far off the prior may be (m).
  Result process(const cv::Mat& depth, const cv::Mat& bgr, const Mat3& K, const VecX& dist,
                 const Mat4& prior, double err);

  struct Ctx;    // one frame's scene
  struct Rim {   // silhouette points and their edge ids
    Pts3 X;
    Idx eid;
  };
  struct Obs {   // colour edges found for silhouette points
    Idx idx;
    Pts2 e_uv, nrm;
  };
  struct EdgeImg {
    cv::Mat img;  // CV_32FC3
    int ox = 0, oy = 0;
  };
  struct Pairs {
    Idx si;
    Pts3 m, n;
  };
  struct RowsOut {
    bool ok = false;
    Rows J;
    VecX r, w;
    Rim used;
  };

 private:
  // ------------------------------------------------------------- geometry
  const cv::Mat& norm_grid(const Mat3& K, const VecX& dist, int h, int w);
  std::vector<char> facing(const Mat3& R, const Vec3& t) const;
  cv::Mat render(const Mat3& R, const Vec3& t, const Mat3& K, const VecX* dist,
                 const std::array<int, 4>& roi, const std::vector<char>& face_on) const;
  void rim(const Mat3& R, const Vec3& t, const Mat3& K, const VecX& dist, const cv::Mat& depth,
           Pts3& X, Pts3& P, Idx& eid) const;
  // ---------------------------------------------------------------- scene
  bool segment(const cv::Mat& depth, const Mat3& K, const VecX& dist, const Mat4& prior,
               Quality& q, double err, Ctx& ctx);
  // ---------------------------------------------------------------- solve
  bool solve(const Mat4& T0, const Ctx& ctx, Quality& q, bool coarse, double err, bool outline,
             Mat4& T, std::optional<Rim>& rim) const;
  bool stage_b(Mat4& T, const Ctx& ctx, Quality& q, const Rim& rim, const EdgeImg& img,
               double& frac, Obs& obs) const;
  static Vec6 gn_step(const Rows& J, const VecX& r, const VecX& w);
  static Mat4 apply(const Mat4& T, const Vec6& x);
  static EdgeImg edge_image(const cv::Mat& bgr, const std::array<int, 4>& roi, int margin = 16);
  static void outline_jac(const Pts3& Xo, const Pts3& Po, const Pts2& ns, const Mat3& R,
                          const Mat3& K, const VecX& scale, Rows& J);
  Pairs surface_pairs(const Ctx& ctx, const Mat3& R, const Vec3& t, double gate_s) const;
  static void surface_rows(const Ctx& ctx, const Mat3& R, const Vec3& t, const Pairs& pairs,
                           Rows& J, VecX& r);
  void depth_outline_rows(const Pts3& X, const Pts3& P, const Mat3& R, const Ctx& ctx,
                          double gate_o, Rows& J, VecX& r, VecX& w) const;
  Obs find_edges(const Pts3& X, const Idx& eid, const Mat3& R, const Vec3& t, const Ctx& ctx,
                 const EdgeImg& img, int win_in, int win_out) const;
  std::vector<char> part_like(const std::vector<std::array<float, 3>>& pool,
                              const std::vector<float>& inner, int n_rows, int n_cols,
                              const std::vector<char>& pk) const;
  void edge_obs_rows(const Pts3& X, const Obs& obs, const Mat3& R, const Vec3& t,
                     const Ctx& ctx, double c_px, Rows& J, VecX& r, VecX& w) const;
  void fixed_rows(const Ctx& ctx, const Mat4& T, const Pairs& pairs, const Pts3& X,
                  const Obs& obs, double c_px, Rows& J, VecX& r, VecX& w) const;
  RowsOut rows(const Ctx& ctx, const Mat3& R, const Vec3& t, double gate_s, double gate_o,
               Quality& q, const Rim* rim, const EdgeImg* edges, int win_in, int win_out,
               const Obs* obs, bool final_stats, bool outline) const;

  Model m_;
  Params p_;
  KdTree<3> tree_;              // over the mesh samples
  // the norm grid cache
  cv::Mat norm_;                // CV_64FC2, h x w
  Mat3 norm_K_ = Mat3::Zero();
  VecX norm_dist_;
  int norm_h_ = 0, norm_w_ = 0;
};

}  // namespace object_pose
