#include "pronto_ros2_controller/gazebo_mt.hpp"


#define SENS_NAME "gazebo_mt."
#define DEFAULT_ROBOT "anymal_c"
#define DEFAULT_LINK "base"
#define DEFAULT_R_XYZ 0.001
#define DEFAULT_R_CHI 0.1
#define DEFAULT_TOPIC "gazebo/link_states"
namespace pronto
{
    GazeboMTRosHandler::GazeboMTRosHandler(rclcpp_lifecycle::LifecycleNode::SharedPtr controller):
    controller_ptr_(controller)
    {
        ViconConfig cfg;
        std::string robot_name,link_name,sensor_name = "gazebo_mt.";
        ViconMode default_mode = ViconMode::MODE_POSITION;
        int mode;
        // set the mt to robot rigid transformation as the identity 
        cfg.body_to_vicon = pronto::Transform(Eigen::Isometry3d::Identity());

        //get the parameters of the gazebo motion tracker module 
        if(! controller_ptr_->get_parameter(sensor_name + "robot_name",robot_name))
        {
                RCLCPP_ERROR(controller_ptr_->get_logger(),"error in parsing robot_name parameter, the default name [%s] will be use",DEFAULT_ROBOT);
                robot_name = DEFAULT_ROBOT;
        }
        if(! controller_ptr_->get_parameter(sensor_name + "base_link_name",link_name))
        {
                RCLCPP_ERROR(controller_ptr_->get_logger(),"error in parsing base_link_name parameter, the default name [%s] will be use",DEFAULT_LINK);
                link_name = DEFAULT_LINK;
        }
        if(! controller_ptr_->get_parameter(sensor_name + "r_xyz",cfg.r_vicon_xyz))
        {
                RCLCPP_ERROR(controller_ptr_->get_logger(),"error in parsing base_link_name parameter, the default name [%f] will be use",DEFAULT_R_XYZ);
                cfg.r_vicon_xyz = DEFAULT_R_XYZ;
        }
        if(! controller_ptr_->get_parameter(sensor_name + "r_chi",cfg.r_vicon_chi))
        {
                RCLCPP_ERROR(controller_ptr_->get_logger(),"error in parsing base_link_name parameter, the default name [%f] will be use",DEFAULT_R_CHI);
                cfg.r_vicon_chi = DEFAULT_R_CHI;
        }
        if(! controller_ptr_->get_parameter(sensor_name + "mode",mode))
        {
                RCLCPP_ERROR(controller_ptr_->get_logger(),"error in parsing base_link_name parameter, the default name [%d] will be use",(int)default_mode);
                cfg.mode = default_mode;
        }
        else
            cfg.mode = static_cast<ViconMode>(mode);

        if(! controller_ptr_->get_parameter(sensor_name + "topic",gz_mt_topic_))
        {
                RCLCPP_ERROR(controller_ptr_->get_logger(),"error in parsing base_link_name parameter, the default name [%s] will be use",DEFAULT_TOPIC);
                gz_mt_topic_ = DEFAULT_TOPIC;
        }
    
        link_name_ = robot_name + "::" + link_name;

        mt_modules_ = ViconModule(cfg);

        RCLCPP_INFO(controller_ptr_->get_logger(),"GazeboMTRosHandler Initialization compleated");

    };
    bool GazeboMTRosHandler::processMessageInit(const gazebo_msgs::msg::LinkStates* gz_ls_msg,
                                const std::map<std::string, bool> &sensor_initialized,
                                const RBIS &default_state,
                                const RBIM &default_cov,
                                RBIS &init_state,
                                RBIM &init_cov)
    {
            gazebo_msgs::msg::LinkState link;
            pronto::RigidTransform mt_data;
            
            if(get_base_link(*gz_ls_msg,link))
            {
                Eigen::Quaterniond rot(link.pose.orientation.w,link.pose.orientation.x,link.pose.orientation.y,link.pose.orientation.z);
                Eigen::Translation3d trasl(link.pose.position.x,link.pose.position.y,link.pose.position.z);
                mt_data.transform = Eigen::Isometry3d(trasl * rot);
                mt_data.utime = (controller_ptr_->now().nanoseconds())/1000;
                return mt_modules_.processMessageInit(&mt_data,sensor_initialized,default_state,default_cov,init_state,init_cov);
            }   
            else
                return false;
              
    };
    RBISUpdateInterface* GazeboMTRosHandler::processMessage(const gazebo_msgs::msg::LinkStates* gz_ls_msg, StateEstimator *est)
    {
        gazebo_msgs::msg::LinkState link;
        pronto::RigidTransform mt_data;
        // RCLCPP_INFO(controller_ptr_->get_logger()," start process message");
        if(get_base_link(*gz_ls_msg,link))
        {
            Eigen::Quaterniond rot(link.pose.orientation.w,link.pose.orientation.x,link.pose.orientation.y,link.pose.orientation.z);
            Eigen::Translation3d trasl(link.pose.position.x,link.pose.position.y,link.pose.position.z);
            mt_data.transform = Eigen::Isometry3d(trasl * rot);
            mt_data.utime = (controller_ptr_->now().nanoseconds())/1000;
        //     RCLCPP_INFO(controller_ptr_->get_logger()," call vicon module Process message");

            return mt_modules_.processMessage(&mt_data,est);
        }   
        else
            return nullptr;
    }
}