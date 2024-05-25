#include "pd_controller/pd_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <std_msgs/msg/detail/float64_multi_array__struct.hpp>



#define QUEUE_SIZE 1



namespace pd_controller {

using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;



/* ========================================================================== */
/*                                PDCONTROLLER                               */
/* ========================================================================== */

PDController::PDController() {}


/* ================================= On_init ================================ */

CallbackReturn PDController::on_init()
{
    try {
        auto_declare<std::string>("robot_name", std::string());

        auto_declare<std::vector<std::string>>("joint_names", std::vector<std::string>());

        auto_declare<double>("initialization_time", double());
        auto_declare<std::vector<double>>("initialization_phases", {1.});

        auto_declare<std::vector<double>>("q0", {});
        auto_declare<std::vector<double>>("q1", std::vector<double>());
        auto_declare<std::vector<double>>("q2", {});

        auto_declare<std::vector<double>>("PD_proportional", {});
        auto_declare<std::vector<double>>("PD_derivative", {});
    }
    catch(const std::exception& e) {
        fprintf(stderr,"Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    
    return CallbackReturn::SUCCESS;
}


/* ===================== Command_interface_configuration ==================== */

InterfaceConfiguration PDController::command_interface_configuration() const
{
    InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
    for (const auto& joint : joint_names_) {
        command_interfaces_config.names.push_back(joint + "/" + HW_IF_EFFORT);
    }

    return command_interfaces_config;
}


/* ====================== State_interface_configuration ===================== */

InterfaceConfiguration PDController::state_interface_configuration() const
{
    InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
    for (const auto& joint : joint_names_) {
        state_interfaces_config.names.push_back(joint + "/" + HW_IF_POSITION);
        state_interfaces_config.names.push_back(joint + "/" + HW_IF_VELOCITY);
    }
    
    return state_interfaces_config;
}


/* ============================== On_configure ============================== */

CallbackReturn PDController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    joint_names_ = get_node()->get_parameter("joint_names").as_string_array();
    if (joint_names_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(),"'joint_names' is empty");
        return CallbackReturn::ERROR;
    }

    init_time_ = get_node()->get_parameter("initialization_time").as_double();
    init_phases_ = get_node()->get_parameter("initialization_phases").as_double_array();


    /* ====================================================================== */

    int nj = joint_names_.size();

    qj_ = Eigen::VectorXd::Zero(nj);
    vj_ = Eigen::VectorXd::Zero(nj);

    auto q0 = get_node()->get_parameter("q0").as_double_array();
    if (static_cast<int>(q0.size()) == 0) {
        q0 = std::vector<double>(nj, 0.0);
    } else if (static_cast<int>(q0.size()) != nj) {
        RCLCPP_ERROR(get_node()->get_logger(),"'q0' does not have nj elements");
        return CallbackReturn::ERROR;
    }
    q0_ = Eigen::VectorXd::Map(q0.data(), q0.size());

    auto q1 = get_node()->get_parameter("q1").as_double_array();
    if (static_cast<int>(q1.size()) != nj) {
        RCLCPP_ERROR(get_node()->get_logger(),"'q1' does not have nj elements");
        return CallbackReturn::ERROR;
    }
    q1_ = Eigen::VectorXd::Map(q1.data(), q1.size());

    auto q2 = get_node()->get_parameter("q2").as_double_array();
    if (static_cast<int>(q2.size()) == 0) {
        q2 = std::vector<double>(nj, 0.0);
    } else if (static_cast<int>(q2.size()) != nj
        && init_phases_.size() > 1) {
        RCLCPP_ERROR(get_node()->get_logger(),"'q2' does not have nj elements");
        return CallbackReturn::ERROR;
    }
    q2_ = Eigen::VectorXd::Map(q2.data(), q2.size());


    PD_proportional_ = get_node()->get_parameter("PD_proportional").as_double_array();
    if (static_cast<int>(PD_proportional_.size()) != nj) {
        RCLCPP_ERROR(get_node()->get_logger(),"'PD_proportional' does not have nj elements");
        return CallbackReturn::ERROR;
    }

    PD_derivative_ = get_node()->get_parameter("PD_derivative").as_double_array();
    if (static_cast<int>(PD_derivative_.size()) != nj) {
        RCLCPP_ERROR(get_node()->get_logger(),"'PD_derivative' does not have nj elements");
        return CallbackReturn::ERROR;
    }

    /* ============================= Subscribers ============================ */

    qj_ref_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/joints_position_reference", QUEUE_SIZE,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) -> void
        {
            qj_ref_ = Eigen::VectorXd::Map(msg->data.data(), msg->data.size());
        }
    );

    vj_ref_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/joints_velocity_reference", QUEUE_SIZE,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) -> void
        {
            vj_ref_ = Eigen::VectorXd::Map(msg->data.data(), msg->data.size());
        }
    );


    return CallbackReturn::SUCCESS;
}


/* =============================== On_activate ============================== */

CallbackReturn PDController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}


/* ============================== On_deactivate ============================= */

CallbackReturn PDController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}


/* ================================= Update ================================= */

controller_interface::return_type PDController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/
) {
    for (uint i=0; i<joint_names_.size(); i++) {
        qj_(i) = state_interfaces_[2*i].get_value();
        vj_(i) = state_interfaces_[2*i+1].get_value();
    }

    if (time.seconds() <= init_time_ || qj_ref_.size() == 0) {
        // The planner is not publishing messages yet. Interpolate from q0 to qi and than wait.

        Eigen::VectorXd qj_ref;
        if (static_cast<int>(init_phases_.size()) < 2) {
            qj_ref = q0_ + std::min(1., time.seconds() / init_time_) * (q1_ - q0_);
        } else {
            if (time.seconds() / init_time_ < init_phases_[0]) {
                double phi = time.seconds() / (init_time_ * init_phases_[0]);
                qj_ref = q0_ + phi * (q1_ - q0_);
            } else if ((time.seconds() - init_time_ * init_phases_[0]) / init_time_ < init_phases_[1]) {
                double phi = (time.seconds() - init_time_ * init_phases_[0]) / (init_time_ * init_phases_[1]);
                qj_ref = q1_ + phi * (q2_ - q1_);
            } else {
                qj_ref = q2_;
            }
        }

        // PD for the state estimator initialization
        for (uint i=0; i<joint_names_.size(); i++) {
            command_interfaces_[i].set_value(
                + PD_proportional_[i] * (qj_ref[i] - qj_(i))
                + PD_derivative_[i] * (- vj_(i))
            );
        }
    } else {
        // PD for the state estimator initialization
        for (uint i=0; i<joint_names_.size(); i++) {
            command_interfaces_[i].set_value(
                + PD_proportional_[i] * (qj_ref_[i] - qj_(i))
                + PD_derivative_[i] * (- vj_(i))
            );
        }
    }

    return controller_interface::return_type::OK;
}

} // namespace pd_controller



PLUGINLIB_EXPORT_CLASS(
    pd_controller::PDController,
    controller_interface::ControllerInterface
)