#pragma once

#include <rclcpp/rclcpp.hpp>

#include "pronto_ros/ros_frontend.hpp"
#include "pronto_ros/ins_ros_handler.hpp"
#include "pronto_ros/pose_msg_ros_handler.hpp"
#include "pronto_ros/scan_matcher_ros_handler.hpp"
#include "pronto_ros/ros_frontend.hpp"

#include "pronto_quadruped_ros/conversions.hpp"
#include "pronto_quadruped_ros/stance_estimator_ros.hpp"
#include "pronto_quadruped_ros/leg_odometer_ros.hpp"
#include "pronto_quadruped_ros/bias_lock_handler_ros.hpp"
#include "pronto_quadruped_ros/legodo_handler_ros.hpp"

#include "pronto_solo12/feet_jacobians.hpp"
#include "pronto_solo12/feet_contact_forces.hpp"
#include "pronto_solo12/forward_kinematics.hpp"


// TODO: add visual odometry
namespace pronto {
namespace solo{

template <class MsgT>
struct is_dummy_msg {
  static const bool value = false;
};

template <>
struct is_dummy_msg<std_msgs::msg::Header> {
  static const bool value = true;
};

// use the std_msgs::msg::Header as a placeholder for a dummy message type
template <class JointStateMsgT, class ContactStateMsgT = std_msgs::msg::Header>
class ProntoNode : public rclcpp::Node {
public:
    using SensorList = std::vector<std::string>;
    using SensorSet = std::set<std::string>;

    ProntoNode();

    virtual ~ProntoNode(){};

    void params_declaration();

    void init(solo::ForwardKinematics& fwd_kin,
                    solo::FeetJacobians& feet_jacs,
                    solo::FeetContactForces& feet_forces,
                    bool subscribe = true);

    void run(solo::ForwardKinematics& fwd_kin,
                        solo::FeetJacobians& feet_jacs,
                        solo::FeetContactForces& feet_forces);

protected:

    SensorList init_sensors;
    SensorList active_sensors;
    SensorSet all_sensors; // sets have unique elements

    // pointers to the modules we might want to initialize
    std::shared_ptr<InsHandlerROS> ins_handler_;
    std::shared_ptr<PoseHandlerROS> pose_handler_;
    std::shared_ptr<ScanMatcherHandler> sm2_handler_;

    // quadruped added here
    std::shared_ptr<quadruped::StanceEstimatorROS> stance_estimator;
    std::shared_ptr<quadruped::LegOdometerROS> leg_odometer;
    std::shared_ptr<quadruped::LegodoHandlerROS> legodo_handler;
    std::shared_ptr<quadruped::ImuBiasLockROS> bias_lock_handler;
    std::shared_ptr<ROSFrontEnd> front_end;

    // quadruped::StanceEstimatorROS stance_estimator;
    // quadruped::LegOdometerROS leg_odometer;
    // std::shared_ptr<quadruped::StanceEstimatorROS> stance_estimator;
    // std::shared_ptr<quadruped::LegOdometerROS> leg_odometer;
    // // quadruped::ImuBiasLockROS imu_bias_lock;


    // quadruped::LegodoHandlerROS legodo_handler;
    // quadruped::ImuBiasLockROS bias_lock_handler;
    // ROSFrontEnd front_end;
    // // SensingModule<JointStateMsgT>& legodo_handler;
    // std::shared_ptr<quadruped::LegodoHandlerROS> legodo_handler;
    // std::shared_ptr<quadruped::ImuBiasLockROS> bias_lock_handler;
    // std::shared_ptr<ROSFrontEnd> front_end;   
};

template <class JointStateMsgT, class ContactStateMsgT> 
ProntoNode<JointStateMsgT, ContactStateMsgT>::ProntoNode() : 
    
    Node("pronto_estimator")
    {

    RCLCPP_INFO(this->get_logger(), "Constructor begins here"); // print for debug
          
    params_declaration(); // declares alla the parameters defined in YAML file

    // get the list of active and init sensors from the param server
    if (!this->get_parameter("init_sensors", init_sensors)) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Not able to get init_sensors param");
    }

    if (!this->get_parameter("active_sensors", active_sensors)) {
        RCLCPP_ERROR(this->get_logger(), "Not able to get active_sensors param");
    }

    bool publish_pose = false;
    if (!this->get_parameter("publish_pose", publish_pose)) {
        RCLCPP_WARN(this->get_logger(), "Not able to get publish_pose param. Not publishing pose.");
    }

    RCLCPP_INFO(this->get_logger(), "Constructor ends here"); // print for debug
}

template <class JointStateMsgT, class ContactStateMsgT>
void ProntoNode<JointStateMsgT, ContactStateMsgT>::init(solo::ForwardKinematics& fwd_kin,
                                                        solo::FeetJacobians& feet_jacs,
                                                        solo::FeetContactForces& feet_forces,
                                                        bool subscribe) {

    RCLCPP_INFO(this->get_logger(), "Initialization begins");

    stance_estimator = std::make_shared<quadruped::StanceEstimatorROS>(shared_from_this(), feet_forces);
    leg_odometer = std::make_shared<quadruped::LegOdometerROS>(shared_from_this(), feet_jacs, fwd_kin);
    legodo_handler = std::make_shared<quadruped::LegodoHandlerROS>(shared_from_this(), *stance_estimator, *leg_odometer);
    bias_lock_handler = std::make_shared<quadruped::ImuBiasLockROS>(shared_from_this());
    front_end = std::make_shared<ROSFrontEnd>(shared_from_this(), true);

   

        // stance_estimator = std::make_shared<quadruped::StanceEstimatorROS>(this->shared_from_this(), feet_forces);
        // leg_odometer = std::make_shared<quadruped::LegOdometerROS>(this->shared_from_this(), feet_jacs, fwd_kin);
        // legodo_handler = std::make_shared<quadruped::LegodoHandlerROS>(this->shared_from_this(), stance_estimator, leg_odometer);
        // bias_lock_handler = std::make_shared<quadruped::ImuBiasLockROS>(this->shared_from_this());
        // front_end = std::make_shared<ROSFrontEnd>(this->shared_from_this());

    // parameters:
    // is the module used for init?
    // do we need to move forward the filter once computed the update?
    // do we want to publish the filter state after the message is received?
    // which topic are we listening to for this module?
    bool init = false;
    bool active = false;
    bool roll_forward = false;
    bool publish_head = false;
    std::string topic;
    std::string secondary_topic;


    for (SensorList::iterator it = init_sensors.begin(); it != init_sensors.end(); ++it) {
        all_sensors.insert(*it);
    }
    for (SensorList::iterator it = active_sensors.begin(); it != active_sensors.end(); ++it) {
        all_sensors.insert(*it);
    }

    // now all_sensors contain a list which is the union (without repetition)
    // of the init sensors and active sensors
    // iterate over the sensors
    for (SensorSet::iterator it = all_sensors.begin(); it != all_sensors.end(); ++it) {
        if (!this->get_parameter(*it + ".roll_forward_on_receive", roll_forward)) {
            RCLCPP_WARN_STREAM(this->get_logger(),"Not adding sensor \"" << *it << "\".");
            RCLCPP_WARN(this->get_logger(), "Param \"roll_forward_on_receive\" not available.");
            continue;
        }
        if (!this->get_parameter(*it + ".publish_head_on_message", publish_head)) {
            RCLCPP_WARN_STREAM(this->get_logger(),"Not adding sensor \"" << *it << "\".");
            RCLCPP_WARN(this->get_logger(), "Param \"publish_head_on_message\" not available.");
            continue;
        }
        if (!this->get_parameter(*it + ".topic", topic)) {
            RCLCPP_WARN_STREAM(this->get_logger(),"Not adding sensor \"" << *it << "\".");
            RCLCPP_WARN(this->get_logger(), "Param \"topic\" not available.");
            continue;
        }
        // check if the sensor is also used to initialize
        init = (std::find(init_sensors.begin(), init_sensors.end(), *it) != init_sensors.end());
        active = (std::find(active_sensors.begin(), active_sensors.end(), *it) != active_sensors.end());

        // is the IMU module in the list? Typically yes.
        if (it->compare("ins") == 0) {
            ins_handler_ = std::make_shared<InsHandlerROS>(this->shared_from_this());
            if (init) {
                front_end->addInitModule(*ins_handler_, *it, topic, subscribe);
            }
            if (active) {
                front_end->addSensingModule(*ins_handler_, *it, roll_forward, publish_head, topic, subscribe);
            }
        }
        // is the leg odometry module in the list?
        if(it->compare("legodo") == 0) {
            if(init){
                front_end->addInitModule(*legodo_handler, *it, topic, subscribe);
            }
            if(active){
                front_end->addSensingModule(*legodo_handler, *it, roll_forward, publish_head, topic, subscribe);
                // if secondary topic is provided, and the second message is not dummy,
                // attempt to cast the leg odometry handler as a DualHandler instead of SingleHandler
                if(!is_dummy_msg<ContactStateMsgT>::value && this->get_parameter(*it + ".secondary_topic", secondary_topic))
                {
                    try{
                        RCLCPP_INFO_STREAM(this->get_logger(),"Subscribing to secondary topic for legodo: " << secondary_topic);
                        front_end->addSecondarySensingModule(dynamic_cast<DualSensingModule<JointStateMsgT,ContactStateMsgT>&>(*legodo_handler),
                                                        *it,
                                                        secondary_topic,
                                                        subscribe);
                  } catch(const std::bad_cast & e){
                    RCLCPP_WARN_STREAM (this->get_logger(), "Could not use the provided Leg Odometry handler as DualSensingModule<"
                                    << type_name<JointStateMsgT>() << ", " << type_name<ContactStateMsgT>() << ">.");
                    RCLCPP_WARN(this->get_logger(),e.what());
                    }
                }
                else {
                    RCLCPP_WARN_STREAM(this->get_logger(), "Legodo not subscribing to secondary topic: Dummy message check: " << is_dummy_msg<ContactStateMsgT>::value);
                }
            }
        }
        if(it->compare("pose_meas") == 0){
            if(init){
                front_end->addInitModule(*pose_handler_, *it, topic, subscribe);
            }
            pose_handler_ = std::make_shared<PoseHandlerROS>(this->shared_from_this());
            if(active){
                front_end->addSensingModule(*pose_handler_, *it, roll_forward, publish_head, topic, subscribe);
            }
        }

        if(it->compare("bias_lock") == 0){
          if(!this->get_parameter(*it + ".secondary_topic", secondary_topic)){
              RCLCPP_WARN_STREAM(this->get_logger(),"Not adding sensor \"" << *it << "\".");
              RCLCPP_WARN_STREAM(this->get_logger(),"Param \"secondary_topic\" not available.");
              continue;
          }
          if(active){
            front_end->addSensingModule(*bias_lock_handler, *it, roll_forward, publish_head, topic, subscribe);
            front_end->addSecondarySensingModule(*bias_lock_handler, *it, secondary_topic, subscribe);
          }
        }

        // if(it->compare("scan_matcher") == 0 ){
        //   bool use_relative_pose = true;
        //   this->get_parameter(*it + "/relative_pose", use_relative_pose);
        //   RCLCPP_WARN_STREAM(this->get_logger(),"Scan matcher will use " << (use_relative_pose ? "relative " : "absolute ") << "pose");

        //   if(use_relative_pose){
        //     sm_handler_ = std::make_shared<LidarOdometryHandlerROS>(this->shared_from_this());
        //     if(active){
        //         front_end->addSensingModule(*sm_handler_, *it, roll_forward, publish_head, topic, subscribe);
        //     }
        //     if(init){
        //         front_end->addInitModule(*sm_handler_, *it, topic, subscribe);
        //     }
        //   } else {
        //     sm2_handler_ = std::make_shared<ScanMatcherHandler>(this->shared_from_this());
        //     if(active){
        //         front_end->addSensingModule(*sm2_handler_, *it, roll_forward, publish_head, topic, subscribe);
        //     }
        //     if(init){
        //         front_end->addInitModule(*sm2_handler_, *it, topic, subscribe);
        //     }
        //   }
        // }

        // TODO: add vicon and fovis
    }

    RCLCPP_INFO(this->get_logger(), "Node initialized"); // print for debug
}

template <class JointStateMsgT, class ContactStateMsgT>
void ProntoNode<JointStateMsgT, ContactStateMsgT>::run(solo::ForwardKinematics& fwd_kin,
                                                        solo::FeetJacobians& feet_jacs,
                                                        solo::FeetContactForces& feet_forces) 
{
    init(fwd_kin, feet_jacs, feet_forces, true);
    if (!this->get_node_base_interface() || !this->get_node_timers_interface()) {
        RCLCPP_ERROR(get_logger(), "Errore durante la creazione del nodo.");
    }
    else if(!this->get_node_topics_interface()){
        RCLCPP_ERROR(this->get_logger(), "Errore nella creazione dei topic.");
    }
    else{
        auto info = this->get_subscriptions_info_by_topic("/joint_states");
        RCLCPP_INFO_STREAM(this->get_logger(), "Funziona.");
    }
    rclcpp::spin(this->shared_from_this());
    rclcpp::shutdown();
}



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
        this->declare_parameter("base_link_name", std::string());
        this->declare_parameter("debug_mode", bool());
        this->declare_parameter("joint_names", std::vector<std::string>());

        this->declare_parameter("init_message.channel", std::string());
        this->declare_parameter("ins.topic", std::string());
        this->declare_parameter("ins.frame", std::string());

        this->declare_parameter("legodo.topic", rclcpp::PARAMETER_STRING);

        this->declare_parameter("bias_lock.topic", std::string());
        this->declare_parameter("bias_lock.secondary_topic", std::string());
    
    } catch(const rclcpp::exceptions::InvalidParameterTypeException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Error in critical params declaration: %s", ex.what());
        RCLCPP_ERROR(this->get_logger(), "Shutting down...");
        rclcpp::shutdown();
    }

    try{
        // in case one parameter is not set the program stops
        // EVERY PARAMETER HAS TO BE SET
        this->declare_parameter("sigma0.vb", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("sigma0.chi_xy", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("sigma0.chi_z", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("sigma0.Delta_xy", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("sigma0.Delta_z", rclcpp::PARAMETER_DOUBLE);        
        this->declare_parameter("sigma0.gyro_bias", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("sigma0.accel_bias", rclcpp::PARAMETER_DOUBLE);

        this->declare_parameter("x0.velocity", std::vector<double>());
        this->declare_parameter("x0.angular_velocity", std::vector<double>());
        this->declare_parameter("x0.position", std::vector<double>());
        this->declare_parameter("x0.rpy", std::vector<double>());

        this->declare_parameter("ins.utime_offset", int());
        this->declare_parameter("ins.downsample_factor", int());
        this->declare_parameter("ins.roll_forward_on_receive", bool());
        this->declare_parameter("ins.publish_head_on_message", bool());
        this->declare_parameter("ins.ignore_accel", bool());
        this->declare_parameter("ins.q_gyro", double());
        this->declare_parameter("ins.q_accel", double());
        this->declare_parameter("ins.q_gyro_bias", double());
        this->declare_parameter("ins.q_accel_bias", double());
        this->declare_parameter("ins.num_to_init", int());
        this->declare_parameter("ins.accel_bias_initial", std::vector<double>());
        this->declare_parameter("ins.accel_bias_recalc_at_start", bool());
        this->declare_parameter("ins.accel_bias_update_online", bool());
        this->declare_parameter("ins.gyro_bias_initial", std::vector<double>());
        this->declare_parameter("ins.gyro_bias_recalc_at_start", bool());
        this->declare_parameter("ins.gyro_bias_update_online", bool());
        this->declare_parameter("ins.timestep_dt", double());
        this->declare_parameter("ins.max_initial_gyro_bias", double());

        this->declare_parameter("legodo.publish_debug_topics", bool());
        this->declare_parameter("legodo.time_offset", double());
        this->declare_parameter("legodo.verbose", bool());
        this->declare_parameter("legodo.debug", bool());
        this->declare_parameter("legodo.legodo_mode", int());
        this->declare_parameter("legodo.stance_mode", int());
        this->declare_parameter("legodo.stance_threshold", double());
        this->declare_parameter("legodo.stance_hysteresis_low", double());
        this->declare_parameter("legodo.stance_hysteresis_high", double());
        this->declare_parameter("legodo.stance_hysteresis_delay_low", int());
        this->declare_parameter("legodo.stance_hysteresis_delay_high", int());
        this->declare_parameter("legodo.stance_alpha", double());
        this->declare_parameter("legodo.stance_regression_beta_size", int());
        this->declare_parameter("legodo.stance_regression_beta", std::vector<double>());
        this->declare_parameter("legodo.r_vx", double());
        this->declare_parameter("legodo.r_vy", double());
        this->declare_parameter("legodo.r_vz", double());
        this->declare_parameter("legodo.downsample_factor", int());
        this->declare_parameter("legodo.utime_offset", int());
        this->declare_parameter("legodo.roll_forward_on_receive", bool());
        this->declare_parameter("legodo.publish_head_on_message", bool());

        this->declare_parameter("pose_meas.no_corrections", int());
        this->declare_parameter("pose_meas.topic", std::string());
        this->declare_parameter("pose_meas.utime_offset", int());
        this->declare_parameter("pose_meas.downsample_factor", int());
        this->declare_parameter("pose_meas.roll_forward_on_receive", bool());
        this->declare_parameter("pose_meas.publish_head_on_message", bool());

        this->declare_parameter("bias_lock.torque_threshold", double());
        this->declare_parameter("bias_lock.velocity_threshold", double());
        this->declare_parameter("bias_lock.roll_forward_on_receive", bool());
        this->declare_parameter("bias_lock.publish_head_on_message", bool());
        this->declare_parameter("bias_lock.utime_offset", int());

    } catch(const rclcpp::exceptions::InvalidParameterTypeException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Error in non critical params declaration: %s", ex.what());
        RCLCPP_INFO(this->get_logger(), "Shutting down...");
        rclcpp::shutdown();
    }
}

} // namespace solo
}  // namespace pronto
