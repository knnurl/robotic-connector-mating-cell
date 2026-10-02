// Exact nearest-neighbour k-d tree over D-dimensional points, with an upper
// distance bound as scipy's cKDTree.query(distance_upper_bound=...) has:
// only points strictly closer than the bound count.
#pragma once

#include <algorithm>
#include <cstddef>
#include <numeric>
#include <vector>

namespace object_pose {

template <int D>
class KdTree {
 public:
  // pts: n rows of D doubles, row-major. The tree keeps its own copy.
  void build(const double* pts, int n) {
    pts_.assign(pts, pts + static_cast<std::size_t>(n) * D);
    n_ = n;
    idx_.resize(n);
    std::iota(idx_.begin(), idx_.end(), 0);
    nodes_.clear();
    if (n > 0) {
      build_rec(0, n);
    }
  }

  int size() const { return n_; }

  // Index of the nearest point strictly within max_dist, or -1. Its
  // squared distance goes to *d2_out.
  int nearest(const double* q, double max_dist, double* d2_out) const {
    double best2 = max_dist * max_dist;
    int best = -1;
    if (n_ > 0) {
      search(0, q, best2, best);
    }
    *d2_out = best2;
    return best;
  }

 private:
  static constexpr int kLeaf = 8;

  struct Node {
    int lo, hi;      // range in idx_
    int left, right; // children, -1 for a leaf
    int dim;
    double split;
  };

  double coord(int i, int k) const { return pts_[static_cast<std::size_t>(i) * D + k]; }

  int build_rec(int lo, int hi) {
    const int id = static_cast<int>(nodes_.size());
    nodes_.push_back({lo, hi, -1, -1, 0, 0.0});
    if (hi - lo <= kLeaf) {
      return id;
    }
    double mn[D], mx[D];
    for (int k = 0; k < D; ++k) {
      mn[k] = mx[k] = coord(idx_[lo], k);
    }
    for (int i = lo + 1; i < hi; ++i) {
      for (int k = 0; k < D; ++k) {
        const double v = coord(idx_[i], k);
        mn[k] = std::min(mn[k], v);
        mx[k] = std::max(mx[k], v);
      }
    }
    int dim = 0;
    for (int k = 1; k < D; ++k) {
      if (mx[k] - mn[k] > mx[dim] - mn[dim]) {
        dim = k;
      }
    }
    if (mx[dim] - mn[dim] <= 0.0) {
      return id;  // all the same point: one leaf
    }
    const int mid = (lo + hi) / 2;
    std::nth_element(idx_.begin() + lo, idx_.begin() + mid, idx_.begin() + hi,
                     [&](int a, int b) { return coord(a, dim) < coord(b, dim); });
    const double split = coord(idx_[mid], dim);
    const int l = build_rec(lo, mid);
    const int r = build_rec(mid, hi);
    nodes_[id].left = l;
    nodes_[id].right = r;
    nodes_[id].dim = dim;
    nodes_[id].split = split;
    return id;
  }

  void search(int node, const double* q, double& best2, int& best) const {
    const Node& nd = nodes_[node];
    if (nd.left < 0) {
      for (int i = nd.lo; i < nd.hi; ++i) {
        double d2 = 0.0;
        for (int k = 0; k < D; ++k) {
          const double e = coord(idx_[i], k) - q[k];
          d2 += e * e;
        }
        if (d2 < best2) {
          best2 = d2;
          best = idx_[i];
        }
      }
      return;
    }
    // left holds values <= split, right values >= split
    const double diff = q[nd.dim] - nd.split;
    const int first = diff < 0.0 ? nd.left : nd.right;
    const int second = diff < 0.0 ? nd.right : nd.left;
    search(first, q, best2, best);
    if (diff * diff < best2) {
      search(second, q, best2, best);
    }
  }

  std::vector<double> pts_;
  std::vector<int> idx_;
  std::vector<Node> nodes_;
  int n_ = 0;
};

}  // namespace object_pose
