#pragma once

#include <memory>
#include <string>
#include <map>
#include "pronto_core/rigidbody.hpp"
#include "pronto_quadruped/ImuBiasLock.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "pronto_ros/pronto_ros_conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "pronto_msgs/msg/joint_state_with_acceleration.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace pronto {
namespace quadruped {

template <class JointStateT>
class ImuBiasLockBaseROS : public DualSensingModule<sensor_msgs::msg::Imu, JointStateT>
{
public:
    ImuBiasLockBaseROS() = delete;
    ImuBiasLockBaseROS(rclcpp::Node::SharedPtr nh);
    RBISUpdateInterface* processMessage(const sensor_msgs::msg::Imu *msg, StateEstimator *est) override;
    // tf2_ros::Buffer tfBuffer;    

    bool processMessageInit(
        const sensor_msgs::msg::Imu* msg,
        const std::map<std::string, bool> &sensor_initialized,
        const RBIS &default_state,
        const RBIM &default_cov,
        RBIS &init_state,
        RBIM &init_cov
    ) override;

protected:
    rclcpp::Node::SharedPtr nh_;
    std::unique_ptr<quadruped::ImuBiasLock> bias_lock_module_;
    pronto::JointState bias_lock_js_msg_;
    pronto::ImuMeasurement bias_lock_imu_msg_;
    visualization_msgs::msg::Marker imu_arrow_;
    visualization_msgs::msg::Marker base_arrow_;
    visualization_msgs::msg::Marker base_more_arrow_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr base_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr base_more_marker_pub_;
    visualization_msgs::msg::MarkerArray frame_markers_;
    visualization_msgs::msg::Marker axis_marker_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    bool publish_debug_topics_ = true;
    bool publish_transforms_ = true;
};

template <class JointStateT>
ImuBiasLockBaseROS<JointStateT>::ImuBiasLockBaseROS(rclcpp::Node::SharedPtr nh) : nh_(nh)
{
    tf2_ros::Buffer tfBuffer(nh_->get_clock());
    tf2_ros::TransformListener tf_imu_to_body_listener_(tfBuffer);

    std::string ins_param_prefix = "ins.";
    std::string lock_param_prefix = "bias_lock.";

    std::string imu_frame = "imu_link";

    nh_->get_parameter_or("ins.frame", imu_frame, imu_frame);
    std::string base_frame = "base";
    nh_->get_parameter_or<std::string>("base_link_name", base_frame, "base_link");
    RCLCPP_INFO_STREAM(nh_->get_logger(), "[ImuBiasLockBaseROS] Name of base_link: '" << base_frame << "'");
    Eigen::Isometry3d ins_to_body = Eigen::Isometry3d::Identity();
    while (rclcpp::ok()) {
        try {
            // lookupTransform API is : target_frame, source_frame
            geometry_msgs::msg::TransformStamped temp_transform = tfBuffer.lookupTransform(base_frame, imu_frame, tf2::TimePointZero);
            ins_to_body = tf2::transformToEigen(temp_transform.transform);
            RCLCPP_INFO_STREAM(nh_->get_logger(), "IMU (" << imu_frame << ") to base (" << base_frame << ") transform: translation=(" << ins_to_body.translation().transpose() << "),");
            RCLCPP_INFO_STREAM(nh_->get_logger(), "rotation=(" << ins_to_body.rotation() << ")");
            break;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR_SKIPFIRST(nh_->get_logger(), "%s", ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }

    quadruped::ImuBiasLockConfig cfg;
    nh_->get_parameter_or(lock_param_prefix + "torque_threshold", cfg.torque_threshold_, 0.0);
    nh_->get_parameter_or(lock_param_prefix + "velocity_threshold", cfg.velocity_threshold_, 0.0);
    nh_->get_parameter_or(lock_param_prefix + "verbose", cfg.verbose_, false);

    if (!nh_->get_parameter(ins_param_prefix + "timestep_dt", cfg.dt_)) {
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't read dt. Using default: " << cfg.dt_);
    }

    // Determine what to publish:
    nh_->get_parameter_or(lock_param_prefix + "publish_debug_topics", publish_debug_topics_, true);  // Default to 'true' to preserve previous behaviour
    nh_->get_parameter_or(lock_param_prefix + "publish_transforms", publish_transforms_, true);    // Default to 'true' to preserve previous behaviour
    RCLCPP_INFO_STREAM(nh_->get_logger(), "[ImuBiasLockBaseROS] Publishing debug topics:  " << std::boolalpha << publish_debug_topics_);
    RCLCPP_INFO_STREAM(nh_->get_logger(),"[ImuBiasLockBaseROS] Publishing transforms:    " << std::boolalpha << publish_transforms_);

    if (publish_debug_topics_) {
        status_pub_ = nh_->create_publisher<geometry_msgs::msg::PointStamped>("/state_estimator_pronto/recording_bias", 100);
        marker_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("/state_estimator_pronto/imu_arrows", 100);
        base_marker_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("/state_estimator_pronto/base_arrow", 100);
        base_more_marker_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("/state_estimator_pronto/base_more_arrow", 100);

        imu_arrow_.color.a = 1;
        imu_arrow_.color.r = 1;
        imu_arrow_.color.g = 0;
        imu_arrow_.color.b = 0;

        imu_arrow_.scale.x = 0.05;
        imu_arrow_.scale.y = 0.07;
        imu_arrow_.scale.z = 0.1;

        imu_arrow_.type = visualization_msgs::msg::Marker::ARROW;

        imu_arrow_.action = visualization_msgs::msg::Marker::ADD;
        imu_arrow_.header.frame_id = imu_frame;
        imu_arrow_.points.resize(2);
        imu_arrow_.points[0].x = 0;
        imu_arrow_.points[0].y = 0;
        imu_arrow_.points[0].z = 0;

        base_arrow_ = imu_arrow_;
        base_arrow_.header.frame_id = base_frame;
        base_arrow_.color.r = 0;
        base_arrow_.color.g = 1;
    }

    bias_lock_module_ = std::make_unique<quadruped::ImuBiasLock>(ins_to_body, cfg);
}

template <class JointStateT>
RBISUpdateInterface* ImuBiasLockBaseROS<JointStateT>::processMessage(const sensor_msgs::msg::Imu *msg, StateEstimator *est)
{
    msgToImuMeasurement(*msg, bias_lock_imu_msg_);

    if (publish_debug_topics_) {
        imu_arrow_.header.stamp = msg->header.stamp;

        imu_arrow_.points[1].x = 0.1 * msg->linear_acceleration.x;
        imu_arrow_.points[1].y = 0.1 * msg->linear_acceleration.y;
        imu_arrow_.points[1].z = 0.1 * msg->linear_acceleration.z;

        marker_pub_->publish(imu_arrow_);

        base_arrow_.points.resize(2);
        base_arrow_.points[0].x = 0;
        base_arrow_.points[0].y = 0;
        base_arrow_.points[0].z = 0;

        // Gravity vector (note, g_vec is in ENU notation so need to take the negative)
        base_arrow_.points[1].x = 0.1 * -g_vec.x();
        base_arrow_.points[1].y = 0.1 * -g_vec.y();
        base_arrow_.points[1].z = 0.1 * -g_vec.z();

        base_marker_pub_->publish(base_arrow_);

        base_more_arrow_ = base_arrow_;
        const Eigen::Vector3d& bias = bias_lock_module_->getCurrentProperAccelBias();
        base_more_arrow_.points.resize(2);
        base_more_arrow_.points[0].x = 0;
        base_more_arrow_.points[0].y = 0;
        base_more_arrow_.points[0].z = 0;

        base_more_arrow_.points[1].x = 0.1 * bias.x();
        base_more_arrow_.points[1].y = 0.1 * bias.y();
        base_more_arrow_.points[1].z = 0.1 * bias.z();

        base_more_arrow_.color.r = 0;
        base_more_arrow_.color.g = 0;
        base_more_arrow_.color.b = 1;

        base_more_marker_pub_->publish(base_more_arrow_);

        geometry_msgs::msg::PointStamped state_msg;
        state_msg.header.stamp = msg->header.stamp;
        state_msg.point.x = static_cast<double>(bias_lock_module_->getRecordStatus());
        state_msg.point.y = state_msg.point.x;
        state_msg.point.z = state_msg.point.x;
        status_pub_->publish(state_msg);
    }

    if (publish_transforms_) {

        const Eigen::Isometry3d& gravity_transform = bias_lock_module_->getGravityTransform();
        const Eigen::Isometry3d& bias_transform = bias_lock_module_->getBiasTransform();

        geometry_msgs::msg::TransformStamped msg_temp = eigToTransf(gravity_transform);
        msg_temp.child_frame_id = "gravity";
        msg_temp.header.frame_id = base_arrow_.header.frame_id;
        msg_temp.header.stamp = msg->header.stamp;

        broadcaster_->sendTransform(msg_temp);

        msg_temp = eigToTransf(bias_transform);
        msg_temp.child_frame_id = "bias";
        msg_temp.header.frame_id = base_arrow_.header.frame_id;
        msg_temp.header.stamp = msg->header.stamp;

        broadcaster_->sendTransform(msg_temp);
    }

    return bias_lock_module_->processMessage(&bias_lock_imu_msg_, est);
}

template <class JointStateT>
bool ImuBiasLockBaseROS<JointStateT>::processMessageInit(
    const sensor_msgs::msg::Imu* msg,
    const std::map<std::string, bool> &sensor_initialized,
    const RBIS &default_state,
    const RBIM &default_cov,
    RBIS &init_state,
    RBIM &init_cov
)
{
    msgToImuMeasurement(*msg, bias_lock_imu_msg_);
    return bias_lock_module_->processMessageInit(
        &bias_lock_imu_msg_,
        sensor_initialized,
        default_state,
        default_cov,
        init_state,
        init_cov
    );
}

class ImuBiasLockROS : public ImuBiasLockBaseROS<sensor_msgs::msg::JointState>
{
public:
    ImuBiasLockROS(rclcpp::Node::SharedPtr nh);
    virtual ~ImuBiasLockROS() = default;

    RBISUpdateInterface* processMessage(const sensor_msgs::msg::Imu *msg, StateEstimator *est) /*override*/; 

    bool processMessageInit(
        const sensor_msgs::msg::Imu *msg,
        const std::map<std::string, bool> &sensor_initialized,
        const RBIS &default_state,
        const RBIM &default_cov,
        RBIS &init_state,
        RBIM &init_cov
    );

    void processSecondaryMessage(const sensor_msgs::msg::JointState &msg) /*override*/;
};

class ImuBiasLockWithAccelerationROS : public ImuBiasLockBaseROS<pronto_msgs::msg::JointStateWithAcceleration>
{
public:
    ImuBiasLockWithAccelerationROS(rclcpp::Node::SharedPtr nh) : 
    nh_(nh),
    ImuBiasLockBaseROS<pronto_msgs::msg::JointStateWithAcceleration>(nh_) {}
    virtual ~ImuBiasLockWithAccelerationROS() = default;

    void processSecondaryMessage(const pronto_msgs::msg::JointStateWithAcceleration &msg);

    rclcpp::Node::SharedPtr nh_;
};

}  // namespace quadruped
}  // namespace pronto
