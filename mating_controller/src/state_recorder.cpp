// Read-only 1 kHz recorder of the FR3 robot state, for diagnosing what the
// 50 Hz panel and tracking logs cannot resolve (the 2026-09-23 ~43 Hz buzz
// showed up in them only as 5-7 Hz aliases).
//
//     ros2 run mating_controller state_recorder            # until Ctrl-C
//     ros2 run mating_controller state_recorder --ros-args -p duration_s:=60.0
//
// Subscribes to /franka_robot_state_broadcaster/robot_state and writes one
// CSV row per message to $FR3_LOG_DIR/YYYY-MM-DD/state_<local time>.csv (or
// -p out:=<path>). It publishes nothing and calls nothing. C++ on purpose:
// decoding the 1 kHz state in Python cost 86% of a core (TODO.md lesson 9).
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <memory>
#include <string>

#include <franka_msgs/msg/franka_robot_state.hpp>
#include <rclcpp/rclcpp.hpp>

using franka_msgs::msg::FrankaRobotState;

namespace
{

std::string local_stamp(const char *format)
{
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    char out[32] = {0};
    std::strftime(out, sizeof(out), format, &local);
    return out;
}

std::string default_path()
{
    const char *root = std::getenv("FR3_LOG_DIR");
    const std::filesystem::path dir =
        std::filesystem::path(root ? root : ".") / local_stamp("%Y-%m-%d");
    std::filesystem::create_directories(dir);
    return (dir / ("state_" + local_stamp("%Y%m%d_%H%M%S") + ".csv")).string();
}

}  // namespace

class StateRecorder : public rclcpp::Node
{
public:
    StateRecorder() : Node("state_recorder")
    {
        const std::string topic = declare_parameter<std::string>(
            "topic", "/franka_robot_state_broadcaster/robot_state");
        std::string path = declare_parameter<std::string>("out", "");
        const double duration_s = declare_parameter<double>("duration_s", 0.0);
        if (path.empty()) {
            path = default_path();
        }
        file_ = std::fopen(path.c_str(), "w");
        if (!file_) {
            throw std::runtime_error("cannot write " + path);
        }
        std::setvbuf(file_, nullptr, _IOFBF, 1 << 20);   // write in 1 MB blocks
        std::fprintf(file_, "time,mode,success");
        for (const char *group : {"q", "dq", "tau", "dtau", "tau_ext"}) {
            for (int j = 1; j <= 7; ++j) {
                std::fprintf(file_, ",%s%d", group, j);
            }
        }
        std::fprintf(file_, ",fx,fy,fz,mx,my,mz,x,y,z,qx,qy,qz,qw\n");

        sub_ = create_subscription<FrankaRobotState>(
            topic, rclcpp::QoS(1000),
            [this](const FrankaRobotState::SharedPtr s) { write(*s); });
        if (duration_s > 0.0) {
            stop_ = create_wall_timer(std::chrono::duration<double>(duration_s),
                                      [] { rclcpp::shutdown(); });
        }
        RCLCPP_INFO(get_logger(), "Recording %s -> %s", topic.c_str(), path.c_str());
    }

    ~StateRecorder() override
    {
        std::fclose(file_);
        RCLCPP_INFO(get_logger(), "Wrote %zu rows.", rows_);
    }

private:
    void write(const FrankaRobotState &s)
    {
        std::fprintf(file_, "%.6f,%u,%.3f", s.time, s.robot_mode,
                     s.control_command_success_rate);
        const auto seven = [this](const auto &v) {
            for (size_t j = 0; j < 7; ++j) {
                std::fprintf(file_, ",%.6g", j < v.size() ? v[j] : 0.0);
            }
        };
        seven(s.measured_joint_state.position);
        seven(s.measured_joint_state.velocity);
        seven(s.measured_joint_state.effort);
        seven(s.dtau_j);
        seven(s.tau_ext_hat_filtered.effort);
        const auto &w = s.o_f_ext_hat_k.wrench;
        const auto &p = s.o_t_ee.pose;
        std::fprintf(file_, ",%.4g,%.4g,%.4g,%.4g,%.4g,%.4g,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                     w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z,
                     p.position.x, p.position.y, p.position.z, p.orientation.x,
                     p.orientation.y, p.orientation.z, p.orientation.w);
        ++rows_;
    }

    std::FILE *file_{nullptr};
    size_t rows_{0};
    rclcpp::Subscription<FrankaRobotState>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr stop_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateRecorder>());
    rclcpp::shutdown();
    return 0;
}
