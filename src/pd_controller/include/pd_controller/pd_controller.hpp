#pragma once

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Core>

#include <std_msgs/msg/detail/float64_multi_array__struct.hpp>
#include <string>
#include <vector>



namespace pd_controller {

/* ========================================================================== */
/*                                PDCONTROLLER                               */
/* ========================================================================== */

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PDController : public controller_interface::ControllerInterface {
public:
    PDController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& /*period*/
    ) override;

    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override;
    // CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    // CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
    // CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

private:
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr qj_ref_subscription_ = nullptr;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr vj_ref_subscription_ = nullptr;

    std::vector<std::string> joint_names_;

    Eigen::VectorXd qj_;
    Eigen::VectorXd vj_;

    Eigen::VectorXd qj_ref_ = Eigen::VectorXd::Zero(0);
    Eigen::VectorXd vj_ref_ = Eigen::VectorXd::Zero(0);

    /// @brief Initialization time to give the state estimator some time to get better estimates. During this time, a PD controller is used to keep the robot in q0 and the planner is paused.
    double init_time_ = 1;
    std::vector<double> init_phases_ = {1};

    Eigen::VectorXd q0_;
    Eigen::VectorXd q1_;
    Eigen::VectorXd q2_;

    std::vector<double> PD_proportional_;
    std::vector<double> PD_derivative_;
};

} // namespace pd_controller