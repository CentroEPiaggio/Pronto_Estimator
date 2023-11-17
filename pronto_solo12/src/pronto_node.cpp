// NOT USED
// This code does not compile, alla the functions are defined in pronto_node.hpp

// THIS FILE IS NOT USED

#include "pronto_solo12/pronto_node.hpp"

namespace pronto{
namespace solo{

    template <class JointStateMsgT, class ContactStateMsgT>
    void ProntoNode<JointStateMsgT, ContactStateMsgT>::params_declaration()
    {
        try{
            // these are critical parameters, if not declared properly the code won't work        
            this->declare_parameter("pose_topic", std::string());
            this->declare_parameter("pose_frame_id", std::string());
            this->declare_parameter("twist_topic", std::string());
            this->declare_parameter("twist_frame_id", std::string());
            this->declare_parameter("filter_state_topic", std::string());
            this->declare_parameter("publish_filter_state", bool());
            this->declare_parameter("publish_pose", bool());
            this->declare_parameter("publish_tf", bool());
            this->declare_parameter("tf_child_frame_id", std::string());
            this->declare_parameter("init_sensors", std::vector<std::string>());
            this->declare_parameter("active_sensors", std::vector<std::string>());
            this->declare_parameter("republish_sensors", bool());
            this->declare_parameter("utime_history_span", double());

            this->declare_parameter("init_message.channel", std::string());
            this->declare_parameter("ins.topic", std::string());
            this->declare_parameter("ins.frame", std::string());
            this->declare_parameter("legodo.topic", std::string());

            this->declare_parameter("bias_lock.topic", std::string());
            this->declare_parameter("bias_lock.secondary_topic", std::string());
        
        } catch(const rclcpp::exceptions::InvalidParameterTypeException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Error in critical params declaration: %s", ex.what());
            RCLCPP_ERROR(this->get_logger(), "Shutting down...");
            rclcpp::shutdown();
        }

        try{
            // these are non critical parameters, default values exist
            this->declare_parameter("sigma0.vb", double());
            this->declare_parameter("sigma0.chi_xy", double());
            this->declare_parameter("sigma0.chi_z", double());
            this->declare_parameter("sigma0.gyro_bias", double());
            this->declare_parameter("sigma0.accel_bias", double());

            this->declare_parameter("x0.velocity", std::vector<double>());
            this->declare_parameter("x0.angular_velocity", std::vector<double>());
            this->declare_parameter("x0.velocity", std::vector<double>());
            this->declare_parameter("x0.velocity", std::vector<double>());

            this->declare_parameter("ins.utime_offset", double());
            this->declare_parameter("ins.downsample_factor", int());
            this->declare_parameter("ins.roll_forward_on_receive", bool());
            this->declare_parameter("ins.publish_head_on_message", bool());
            this->declare_parameter("ins.ignore_accel", bool());
            this->declare_parameter("ins.q_gyro", double());
            this->declare_parameter("ins.q_accel", double());
            this->declare_parameter("ins.q_gyro_bias", double());
            this->declare_parameter("ins.q_accel_bias", double());
            this->declare_parameter("ins.num_to_init", double());
            this->declare_parameter("ins.accel_bias_initial", std::vector<double>());
            this->declare_parameter("ins.accel_bias_recalc_at_start", bool());
            this->declare_parameter("ins.accel_bias_update_online", bool());
            this->declare_parameter("ins.gyro_bias_initial", std::vector<double>());
            this->declare_parameter("ins.gyro_bias_recalc_at_start", bool());
            this->declare_parameter("ins.gyro_bias_update_online", bool());
            this->declare_parameter("ins.timestep_dt", double());
            this->declare_parameter("ins.max_initial_gyro_bias", double());

            this->declare_parameter("legodo.time_offset", double());
            this->declare_parameter("legodo.verbose", bool());
            this->declare_parameter("legodo.debug", bool());
            this->declare_parameter("legodo.legodo_mode", int());
            this->declare_parameter("legodo.stance_mode", int());
            this->declare_parameter("legodo.stance_threshold", double());
            this->declare_parameter("legodo.stance_hysteresis_low", double());
            this->declare_parameter("legodo.stance_hysteresis_high", double());
            this->declare_parameter("legodo.stance_hysteresis_delay_low", double());
            this->declare_parameter("legodo.stance_hysteresis_delay_high", double());
            this->declare_parameter("legodo.stance_alpha", double());
            this->declare_parameter("legodo.stance_regression_beta_size", int());
            this->declare_parameter("legodo.stance_regression_beta", std::vector<double>());
            this->declare_parameter("legodo.r_vx", double());
            this->declare_parameter("legodo.r_vy", double());
            this->declare_parameter("legodo.r_vz", double());
            this->declare_parameter("legodo.downsample_factor", int());
            this->declare_parameter("legodo.utime_offset", double());
            this->declare_parameter("legodo.roll_forward_on_receive", bool());
            this->declare_parameter("legodo.publish_head_on_message", bool());

            this->declare_parameter("bias_lock.torque_threshold", double());
            this->declare_parameter("bias_lock.velocity_threshold", double());
            this->declare_parameter("bias_lock.roll_forward_on_receive", bool());
            this->declare_parameter("bias_lock.publish_head_on_message", bool());
            this->declare_parameter("bias_lock.utime_offset", double());

        } catch(const rclcpp::exceptions::InvalidParameterTypeException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Error in non critical params declaration: %s", ex.what());
        }
    }

}   // namespace solo
}   // namespace pronto