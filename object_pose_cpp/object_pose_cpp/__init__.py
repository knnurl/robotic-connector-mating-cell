"""The object pose estimator of roscam/object_pose.py in C++ (Estimator).

Use it through roscam.object_pose.CppObjectPoseEstimator, which prepares
the model (mesh samples and edges) in Python and hands it over, so both
versions run on identical data; roscam/test/test_object_pose_cpp.py holds
them to the same answers."""

from ._object_pose_cpp import Estimator  # noqa: F401
