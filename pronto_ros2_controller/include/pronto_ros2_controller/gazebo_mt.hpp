#pragma once
#include "gazebo_msgs/msg/link_states.hpp"
#include "gazebo_msgs/msg/link_state.hpp"
#include <functional>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "pronto_core/vicon_module.hpp"
#include "pronto_core/definitions.hpp"

namespace pronto
{   
    class GazeboMTRosHandler : public SensingModule<gazebo_msgs::msg::LinkStates>
    {
        public:
            GazeboMTRosHandler(rclcpp_lifecycle::LifecycleNode::SharedPtr controller);
            ~GazeboMTRosHandler(){};
            RBISUpdateInterface* processMessage(const gazebo_msgs::msg::LinkStates* gz_ls_msg, StateEstimator *est) override; 

            bool processMessageInit(const gazebo_msgs::msg::LinkStates* gz_ls_msg,
                                const std::map<std::string, bool> &sensor_initialized,
                                const RBIS &default_state,
                                const RBIM &default_cov,
                                RBIS &init_state,
                                RBIM &init_cov) override; 
            inline std::string get_sens_topic()
            {
                return gz_mt_topic_;
            }
            
        private:
            bool get_base_link(gazebo_msgs::msg::LinkStates msg, gazebo_msgs::msg::LinkState &link_stt)
            {
                for(size_t i = 0; i < msg.name.size(); i++)
                {
                    if(msg.name[i].compare(link_name_) == 0)
                    {
                        link_stt.set__link_name(msg.name[i]);
                        link_stt.set__pose(msg.pose[i]);
                        link_stt.set__twist(msg.twist[i]);
                    }
                }
                if(link_stt.link_name.empty())
                    return false;
                else
                    return true;
            }
            
            rclcpp_lifecycle::LifecycleNode::SharedPtr controller_ptr_;
            ViconModule mt_modules_;
            std::string gz_mt_topic_ = "";
            std::string link_name_ = "";
            

    };
} // namespace pronto