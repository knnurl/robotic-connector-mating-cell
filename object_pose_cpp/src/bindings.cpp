// Python binding: object_pose_cpp.Estimator(model: dict, params: dict),
// .warm(K, dist, h, w), .process(depth, bgr_or_None, K, dist, prior, err)
// -> dict (see roscam.object_pose.CppObjectPoseEstimator, which wraps it).
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <chrono>

#include "estimator.hpp"

namespace py = pybind11;
using namespace object_pose;

namespace {

using DArr = py::array_t<double, py::array::c_style | py::array::forcecast>;
using IArr = py::array_t<int, py::array::c_style | py::array::forcecast>;

template <typename M>
M rows_of(const py::handle& h, int cols) {
  DArr a = DArr::ensure(h);
  if (!a || a.ndim() != 2 || a.shape(1) != cols) throw std::runtime_error("bad array shape");
  M out(a.shape(0), cols);
  std::copy(a.data(), a.data() + a.size(), out.data());
  return out;
}

template <typename M>
M irows_of(const py::handle& h, int cols) {
  IArr a = IArr::ensure(h);
  if (!a || a.ndim() != 2 || a.shape(1) != cols) throw std::runtime_error("bad array shape");
  M out(a.shape(0), cols);
  std::copy(a.data(), a.data() + a.size(), out.data());
  return out;
}

py::object opt(const std::optional<double>& v) {
  return v ? py::object(py::float_(*v)) : py::object(py::none());
}

}  // namespace

PYBIND11_MODULE(_object_pose_cpp, m) {
  m.doc() = "roscam/object_pose.py's ObjectPoseEstimator.process in C++";
  py::class_<Estimator>(m, "Estimator")
      .def(py::init([](const py::dict& model, const py::dict& params) {
             Model md;
             md.V = rows_of<Pts3>(model["V"], 3);
             md.F = irows_of<decltype(md.F)>(model["F"], 3);
             md.face_n = rows_of<Pts3>(model["face_n"], 3);
             md.face_c = rows_of<Pts3>(model["face_c"], 3);
             md.samples = rows_of<Pts3>(model["samples"], 3);
             md.sample_n = rows_of<Pts3>(model["sample_n"], 3);
             md.edge_faces = irows_of<decltype(md.edge_faces)>(model["edge_faces"], 2);
             md.edge_dir = rows_of<Pts3>(model["edge_dir"], 3);
             md.edge_pts = rows_of<Pts3>(model["edge_pts"], 3);
             IArr eid = IArr::ensure(model["edge_id"]);
             md.edge_id.assign(eid.data(), eid.data() + eid.size());
             md.sym_order = model["sym_order"].cast<int>();
             md.size_m = model["size_m"].cast<double>();
             Params p;
             auto d = [&](const char* k) { return params[k].cast<double>(); };
             auto b = [&](const char* k) { return params[k].cast<bool>(); };
             p.search_m = d("search_m");
             p.prior_err_m = d("prior_err_m");
             p.depth_band_m = d("depth_band_m");
             p.support_band_m = d("support_band_m");
             p.jump_ratio = d("jump_ratio");
             p.rim_min_cos = d("rim_min_cos");
             p.outline_offset_px = d("outline_offset_px");
             p.max_iter = params["max_iter"].cast<int>();
             p.max_points = params["max_points"].cast<int>();
             p.outline_weight = d("outline_weight");
             p.use_outline = b("use_outline");
             p.colour_edges = b("colour_edges");
             p.depth_fallback = b("depth_fallback");
             p.edge_min_step = d("edge_min_step");
             p.edge_rel = d("edge_rel");
             p.min_points = params["min_points"].cast<int>();
             p.size_lo = d("size_lo");
             p.size_hi = d("size_hi");
             p.weak_rel = d("weak_rel");
             p.min_outline_frac = d("min_outline_frac");
             return new Estimator(std::move(md), p);
           }),
           py::arg("model"), py::arg("params"))
      .def("warm",
           [](Estimator& e, const Mat3& K, const VecX& dist, int h, int w) {
             e.warm(K, dist, h, w);
           },
           py::arg("K"), py::arg("dist"), py::arg("h"), py::arg("w"))
      .def("process",
           [](Estimator& e, py::array depth, py::object bgr, const Mat3& K, const VecX& dist,
              const Mat4& prior, double err) {
             if (depth.ndim() != 2) throw std::runtime_error("depth must be H x W");
             // depth: float32 or float64 as it comes (no copy when contiguous)
             py::array keep_depth;
             cv::Mat dm;
             if (py::isinstance<py::array_t<float>>(depth)) {
               auto a = py::array_t<float, py::array::c_style>::ensure(depth);
               keep_depth = a;
               dm = cv::Mat(static_cast<int>(a.shape(0)), static_cast<int>(a.shape(1)), CV_32F,
                            const_cast<float*>(a.data()));
             } else {
               DArr a = DArr::ensure(depth);
               keep_depth = a;
               dm = cv::Mat(static_cast<int>(a.shape(0)), static_cast<int>(a.shape(1)), CV_64F,
                            const_cast<double*>(a.data()));
             }
             py::array keep_bgr;
             cv::Mat cm;
             if (!bgr.is_none()) {
               auto a = py::array_t<uint8_t, py::array::c_style | py::array::forcecast>::ensure(bgr);
               if (!a || a.ndim() != 3 || a.shape(2) != 3)
                 throw std::runtime_error("bgr must be H x W x 3 uint8");
               keep_bgr = a;
               cm = cv::Mat(static_cast<int>(a.shape(0)), static_cast<int>(a.shape(1)), CV_8UC3,
                            const_cast<uint8_t*>(a.data()));
             }
             Result r;
             {
               py::gil_scoped_release unlocked;
               r = e.process(dm, cm, K, dist, prior, err);
             }
             const Quality& q = r.q;
             py::dict out;
             out["T"] = r.has_T ? py::cast(Mat4(r.T)) : py::object(py::none());
             out["finished"] = q.finished;
             out["reason"] = q.reason;
             out["n_pts"] = q.n_pts;
             out["size_ratio"] = opt(q.size_ratio);
             out["rms_mm"] = opt(q.rms_mm);
             out["inlier_frac"] = opt(q.inlier_frac);
             out["outline_frac"] = opt(q.outline_frac);
             out["weak_dof"] = q.weak_dof;
             out["agree_mm"] = q.agree_mm;
             out["agree_deg"] = q.agree_deg;
             out["agree_tilt_deg"] = q.agree_tilt_deg;
             out["agree_inplane_deg"] = q.agree_inplane_deg;
             out["sym_index"] = q.sym_index;
             out["iterations"] = q.iterations;
             out["edge_source"] =
                 q.edge_source.empty() ? py::object(py::none()) : py::object(py::str(q.edge_source));
             if (q.has_support) {
               py::list n;
               for (int k = 0; k < 3; ++k) n.append(q.support_n(k));
               out["support"] = py::make_tuple(n, q.support_d);
             } else {
               out["support"] = py::none();
             }
             return out;
           },
           py::arg("depth"), py::arg("bgr"), py::arg("K"), py::arg("dist"), py::arg("prior"),
           py::arg("err"));
}
