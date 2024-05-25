#include "pronto_ros2_controller/exteroceptive_sensor_sensor_manager.hpp"

namespace pronto_controller
{
    bool Exte_Sensor_Manager::configure_exteroceptive_module(pronto::RBIS def_stt, pronto::RBIM def_cov)
    {
        // set the default stt and cov
        default_stt_cntr_ = def_stt;
        default_cov_cntr_ = def_cov;
        // iter in active and create active module and if necessary create init module

        for(size_t i = 0; i < active_sens_.size(); i ++)
        {
            RCLCPP_INFO(filt_controller_->get_logger(),"Will be created  Exteroceptive active module %s",active_sens_[i].c_str());
            // switch case to create the sensors
            // for each case create allocate the shared_ptr, and add active module
            if(active_sens_[i].compare("gazebo_mt") == 0)
            {
                RCLCPP_INFO(filt_controller_->get_logger(),"try to create the sensing module");
                gz_motion_tracker_ = std::make_shared<pronto::GazeboMTRosHandler>(filt_controller_->shared_from_this());
                RCLCPP_INFO(filt_controller_->get_logger(),"created the sensing module");
                addSensingModule(*gz_motion_tracker_,"gazebo_mt",gz_motion_tracker_->get_sens_topic());

            }
        }

        for(size_t i = 0; i < init_sens_.size(); i++)
        {
            RCLCPP_INFO(filt_controller_->get_logger(),"Will be created Exteroceptive init module %s",init_sens_[i].c_str());
            std::vector<std::string>::iterator it;
            // switch case to create the sensors if needed, show warning in this case
            // add init module  and add pair to map initialized_sensor
            if(init_sens_[i].compare("gazebo_mt") == 0)
            {
                RCLCPP_INFO(filt_controller_->get_logger(),"asdadadsa");
                if(gz_motion_tracker_ == nullptr)
                {
                    RCLCPP_WARN(filt_controller_->get_logger(),"The sensor %s has the init module and not the active one",init_sens_[i].c_str());
                    gz_motion_tracker_ = std::make_shared<pronto::GazeboMTRosHandler>(filt_controller_->shared_from_this());
                }
                addInitModule(*gz_motion_tracker_,"gazebo_mt",gz_motion_tracker_->get_sens_topic());
            }
        }
        
        



       return true; 
    };
    void Exte_Sensor_Manager::set_exteroceptive_init_state(pronto::RBIS& init_stt, pronto::RBIM& init_cov)
    {
            init_stt = init_stt_ext_sns_;
            init_cov = init_cov_ext_sns_;
    }

}