#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "pronto_core/ins_module.hpp"
#include <vector>
#include <string>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <mutex>
#include <pronto_core/sensing_module.hpp>
#include <pronto_ros2_controller/gazebo_mt.hpp>
#include <pronto_core/state_est.hpp>

namespace pronto_controller
{
    
    using SensorId_t = std::string;
    using namespace pronto;
    class Exte_Sensor_Manager
    {
        public:
            Exte_Sensor_Manager(
                rclcpp_lifecycle::LifecycleNode::SharedPtr controller,
                
                std::vector<std::string>& active_ext_sens,
                std::vector<std::string>& init_ext_sens)
                // stt_est_ptr_(std::move(stt_est))
            {
                
                filt_controller_ = controller; 
                init_sens_ = init_ext_sens;
                active_sens_ = active_ext_sens;
            }
            ~Exte_Sensor_Manager(){};
            
            void set_filter_state(bool stt)
            {
                filter_init_ = stt;
            }
            // create the exteroceptive sensor module and the subscriber and set the 
            bool configure_exteroceptive_module(pronto::RBIS def_stt, pronto::RBIM def_cov);

            void set_exteroceptive_init_state(pronto::RBIS& init_stt, pronto::RBIM& init_cov);
            
            bool is_all_init()
            {
                for(std::map<SensorId_t,bool>::iterator it = initialize_sensors_.begin();it != initialize_sensors_.end(); ++it)
                {
                    if(!it->second)
                        return false;
                }
                return true;
            }
            void set_estimator(std::shared_ptr<pronto::StateEstimator> stt_est)
            {
                stt_est_ptr_.reset(stt_est.get());
                set_filter_state(true);
            }
            
            bool isExteroceptiveSensorInit()
            {
                
                for(auto &it: initialize_sensors_)
                {
                    if(!it.second)
                        return false;
                }
                return true;
            }
            std::map<std::string,bool> get_initialized_sens()
            {
                return initialize_sensors_;
            }     ;
        std::mutex  filt_mutex_;
        private:
        template<typename T>
        std::string type_name()
        {
            int status;
            std::string tname = typeid(T).name();
            char *demangled_name = abi::__cxa_demangle(tname.c_str(), nullptr, nullptr, &status);
            if (status == 0) {
                tname = demangled_name;
                std::free(demangled_name);
            }
            return tname;
        }
        template<typename MsgT>
        void addSensingModule(pronto::SensingModule<MsgT>& module, const SensorId_t& sensor_id,  const std::string& topic) 
            {
                
                if(active_modules_.count(sensor_id) > 0){
                        RCLCPP_WARN_STREAM(filt_controller_->get_logger(), "Sensing Module \"" << sensor_id << "\" already added. Skipping.");
                        return;
                }
                
                RCLCPP_INFO_STREAM(filt_controller_->get_logger(), "Sensor id: " << sensor_id);
                RCLCPP_INFO_STREAM(filt_controller_->get_logger(), "Topic: " << topic);

                // store the module as void*, to allow for different types of module to stay
                // in the same container. The type will be known when the message arrives
                // so we can properly cast back to the right type.
                std::pair<SensorId_t, void*> pair(sensor_id, (SensingModule<MsgT>*) &module);
                active_modules_.insert(pair);

                // subscribe the generic templated callback for all modules
               
                RCLCPP_INFO_STREAM(filt_controller_->get_logger(), sensor_id << " subscribing to " << topic);
                RCLCPP_INFO_STREAM(filt_controller_->get_logger(), " with MsgT = " << type_name<MsgT>() );

                rclcpp::QoS qos(10);  // Keep the last 10000 messages

                auto lambdaCallback = [this, sensor_id](const MsgT & msg) {
                    this->callback<MsgT>(msg, sensor_id);
                };

                // auto fcn = std::bind(ProntoNode::callback<MsgT>, std::placeholders::_1, sensor_id);
                sensor_subscribers_[sensor_id] = filt_controller_->create_subscription<MsgT>(topic, qos, lambdaCallback);
            }

            template<class MsgT>
            void callback(const MsgT & msg, const SensorId_t& sensor_id)
            {
                RCLCPP_DEBUG_STREAM(filt_controller_->get_logger(), "Callback for sensor " << sensor_id);
                bool is_init = false;
            //     // this is a generic templated callback that does the same for every module:
            //     // if the module is initialized and the filter is ready
            //     // 1) take the measurement update and pass it to the filter if valid
                    if(filter_init_)
                    {
                        
                        try
                        {
                            is_init = initialize_sensors_.at(sensor_id);
                        }
                        catch(const std::exception& e)
                        {
                            RCLCPP_DEBUG(filt_controller_->get_logger(),"%s has not to be init",sensor_id.c_str());
                            is_init = true;
                        }
                        
                        if(!is_init)
                        {
                            RCLCPP_WARN(filt_controller_->get_logger(),"Filter sensor %s is not initialized",sensor_id.c_str());
                            return;
                        }
                        // appropriate casting to the right type and call to the process message
                        // function to get the update
                        // Record start time
                        {
                            std::lock_guard<std::mutex> l_g(filt_mutex_);
                            auto start = std::chrono::high_resolution_clock::now();
                            RBISUpdateInterface* update = static_cast<pronto::SensingModule<MsgT>*>(active_modules_[sensor_id])->processMessage(&msg, stt_est_ptr_.get());
                            auto end = std::chrono::high_resolution_clock::now();
                            RCLCPP_DEBUG_STREAM(filt_controller_->get_logger(), "Time elapsed process message: " << std::chrono::duration_cast<std::chrono::microseconds>(end -start).count());
                            start = end;
                            if(update == nullptr) {

                                RCLCPP_DEBUG_STREAM(filt_controller_->get_logger(), "Invalid " << sensor_id << " update" << std::endl);
                                // special case for pose meas, it returns null when it does not want
                                // to process data anymore
                                if(sensor_id.compare("pose_meas") == 0){
                                        sensor_subscribers_["pose_meas"].reset();
                                }
                                return;

                            }
                        
                            stt_est_ptr_->addUpdate(update,true);
                        
                            end = std::chrono::high_resolution_clock::now();
                            RCLCPP_DEBUG_STREAM(filt_controller_->get_logger(), "Time elapsed process addUpdate: " << std::chrono::duration_cast<std::chrono::microseconds>(end -start).count());
                            start = end;


                            end = std::chrono::high_resolution_clock::now();
                            RCLCPP_DEBUG_STREAM(filt_controller_->get_logger(), "Time elapsed till the end: " << std::chrono::duration_cast<std::chrono::microseconds>(end -start).count());
                            
                        }
                    }
                
            }

            template<class MsgT>
            void addInitModule(pronto::SensingModule<MsgT>& module, const SensorId_t& sensor_id,
                            const std::string& topic)

            {

                if(init_modules_.count(sensor_id) > 0){
                        RCLCPP_WARN_STREAM(filt_controller_->get_logger(), "Init Module \"" << sensor_id << "\" already added. Skipping.");
                        return;
                }
                RCLCPP_INFO_STREAM(filt_controller_->get_logger(), "Sensor init id: " << sensor_id);
                RCLCPP_INFO_STREAM(filt_controller_->get_logger(), "Topic: " << topic);

                // add the sensor to the list of sensor that require initialization
                std::pair<SensorId_t, bool> init_id_pair(sensor_id, false);
                initialize_sensors_.insert(init_id_pair);
                // store the module as void*, to allow for different types of module to stay
                // in the same container. The type will be known when the message arrives
                // so we can properly cast back to the right type.
                std::pair<SensorId_t, void*> pair(sensor_id, (void*) &module);
                init_modules_.insert(pair);
            
                RCLCPP_INFO_STREAM(filt_controller_->get_logger(), sensor_id << " subscribing to " << topic);
                RCLCPP_INFO_STREAM(filt_controller_->get_logger(), " with MsgT = " << type_name<MsgT>() );
                rclcpp::QoS qos(10);  // Keep the last 10000 messages

                auto lambdaCallback = [this, sensor_id](const MsgT & msg) {
                        this->initCallback<MsgT>(msg, sensor_id);
                };

                init_subscribers_[sensor_id] = filt_controller_->create_subscription<MsgT>(topic, qos, lambdaCallback);
            

            }

            template<class MsgT>
            void initCallback(const MsgT & msg, const SensorId_t& sensor_id)
            {

                
                RCLCPP_INFO_STREAM(filt_controller_->get_logger(), "Init callback for sensor " << sensor_id);
                if(initialize_sensors_.count(sensor_id) > 0 && !initialize_sensors_[sensor_id])
                {
                        initialize_sensors_[sensor_id] = static_cast<SensingModule<MsgT>*>(init_modules_[sensor_id])->processMessageInit(&msg,
                            initialize_sensors_,
                            default_stt_cntr_,
                            default_cov_cntr_,
                            init_stt_ext_sns_,
                            init_cov_ext_sns_);

                        // if the sensor has been successfully initialized, we unsubscribe.
                        // This happens only for the sensors which are only for initialization.
                        // The sensor which are for initialization and active will continue to listen
                        if(initialize_sensors_[sensor_id]){
                                init_subscribers_[sensor_id].reset();
                                // // attempt to initialize the filter, because the value has changed
                                // // in the list
                                // initializeFilter();
                        }
                } else {
                        // if we are here it means that the module is not in the list of
                        // initialized modules or that the module is already initialized
                        // in both cases we don't want to subscribe to this topic anymore,
                        // unless there is no subscriber because we are processing a rosbag.
                        if(init_subscribers_.count(sensor_id) > 0){
                                init_subscribers_[sensor_id].reset();
                        }
                }
            }
            
            // shared pointer to the pronto core ins module, TODO check the correct initialization of it
            

            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> filt_controller_;
            std::shared_ptr<pronto::StateEstimator> stt_est_ptr_;


            // the exteroceptive sensor manager should manage the correction provided by a single subscibe module, like lidar odometry or motion trakcer correction
            std::map<SensorId_t, rclcpp::SubscriptionBase::SharedPtr> sensor_subscribers_;
            std::map<SensorId_t, rclcpp::SubscriptionBase::SharedPtr> init_subscribers_;
            std::map<SensorId_t, void*> active_modules_;
            std::map<SensorId_t, void*> init_modules_;
            std::map<SensorId_t, bool> initialize_sensors_ = {};
            std::vector<std::string> active_sens_,init_sens_;

            pronto::RBIS default_stt_cntr_,init_stt_ext_sns_;
            pronto::RBIM default_cov_cntr_,init_cov_ext_sns_;
            bool filter_init_ = false ;

            std::shared_ptr<pronto::GazeboMTRosHandler> gz_motion_tracker_ = nullptr;

            

            
    };


};