
template<typename MsgT>
void Exte_Sensor_Manager::addSensingModule(pronto::SensingModule<MsgT>& module, const SensorId_t& sensor_id, bool roll_forward,
                                            bool publish_head, const std::string& topic, bool subscribe) 
{

    if(active_modules_.count(sensor_id) > 0){
            RCLCPP_WARN_STREAM(this->get_logger(), "Sensing Module \"" << sensor_id << "\" already added. Skipping.");
            return;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Sensor id: " << sensor_id);
    RCLCPP_INFO_STREAM(this->get_logger(), "Roll forward: "<< (roll_forward? "yes" : "no"));
    RCLCPP_INFO_STREAM(this->get_logger(), "Publish head: "<< (publish_head? "yes" : "no"));
    RCLCPP_INFO_STREAM(this->get_logger(), "Topic: " << topic);


    // store the will to roll forward when the message is received
    std::pair<SensorId_t, bool> roll_pair(sensor_id, roll_forward);
    roll_forward_.insert(roll_pair);

    // store the will to publish the estimator state when the message is received
    std::pair<SensorId_t, bool> publish_pair(sensor_id, publish_head);
    publish_head_.insert(publish_pair);

    // store the module as void*, to allow for different types of module to stay
    // in the same container. The type will be known when the message arrives
    // so we can properly cast back to the right type.
    std::pair<SensorId_t, void*> pair(sensor_id, (SensingModule<MsgT>*) &module);
    active_modules_.insert(pair);

    // subscribe the generic templated callback for all modules
    if(subscribe){


            std::cerr << sensor_id << " subscribing to " << topic;
            std::cerr << " with MsgT = " << type_name<MsgT>() << std::endl;

            rclcpp::QoS qos(rclcpp::KeepLast(10000));  // Keep the last 10000 messages
            qos.transient_local();  // Use transient local QoS for fast intra-process communication
            // auto fcn = std::bind(&ProntoNode::callback<MsgT>, std::placeholders::_1, sensor_id);

            auto lambdaCallback = [this, sensor_id](const MsgT & msg) {
                this->callback<MsgT>(msg, sensor_id);
            };

            // auto fcn = std::bind(ProntoNode::callback<MsgT>, std::placeholders::_1, sensor_id);
            sensor_subscribers_[sensor_id] = this->create_subscription<MsgT>(topic, qos, lambdaCallback);
                                
    }


};

