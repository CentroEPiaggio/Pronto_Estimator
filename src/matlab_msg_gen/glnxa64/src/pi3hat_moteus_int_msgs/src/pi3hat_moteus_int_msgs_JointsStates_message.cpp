// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for pi3hat_moteus_int_msgs/JointsStates
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4100)
#pragma warning(disable : 4265)
#pragma warning(disable : 4456)
#pragma warning(disable : 4458)
#pragma warning(disable : 4946)
#pragma warning(disable : 4244)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif //_MSC_VER
#include "rclcpp/rclcpp.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_states.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class PI3HAT_MOTEUS_INT_MSGS_EXPORT ros2_pi3hat_moteus_int_msgs_msg_JointsStates_common : public MATLABROS2MsgInterface<pi3hat_moteus_int_msgs::msg::JointsStates> {
  public:
    virtual ~ros2_pi3hat_moteus_int_msgs_msg_JointsStates_common(){}
    virtual void copy_from_struct(pi3hat_moteus_int_msgs::msg::JointsStates* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const pi3hat_moteus_int_msgs::msg::JointsStates* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_pi3hat_moteus_int_msgs_msg_JointsStates_common::copy_from_struct(pi3hat_moteus_int_msgs::msg::JointsStates* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //header
        const matlab::data::StructArray header_arr = arr["header"];
        auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
        msgClassPtr_header->copy_from_struct(&msg->header,header_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'header' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'header' is wrong type; expected a struct.");
    }
    try {
        //name
        const matlab::data::CellArray name_cellarr = arr["name"];
        size_t nelem = name_cellarr.getNumberOfElements();
        for (size_t idx=0; idx < nelem; ++idx){
        	const matlab::data::CharArray name_arr = name_cellarr[idx];
        	msg->name.push_back(name_arr.toAscii());
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'name' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'name' is wrong type; expected a string.");
    }
    try {
        //position
        const matlab::data::TypedArray<double> position_arr = arr["position"];
        size_t nelem = position_arr.getNumberOfElements();
        	msg->position.resize(nelem);
        	std::copy(position_arr.begin(), position_arr.begin()+nelem, msg->position.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'position' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'position' is wrong type; expected a double.");
    }
    try {
        //velocity
        const matlab::data::TypedArray<double> velocity_arr = arr["velocity"];
        size_t nelem = velocity_arr.getNumberOfElements();
        	msg->velocity.resize(nelem);
        	std::copy(velocity_arr.begin(), velocity_arr.begin()+nelem, msg->velocity.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'velocity' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'velocity' is wrong type; expected a double.");
    }
    try {
        //effort
        const matlab::data::TypedArray<double> effort_arr = arr["effort"];
        size_t nelem = effort_arr.getNumberOfElements();
        	msg->effort.resize(nelem);
        	std::copy(effort_arr.begin(), effort_arr.begin()+nelem, msg->effort.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'effort' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'effort' is wrong type; expected a double.");
    }
    try {
        //current
        const matlab::data::TypedArray<double> current_arr = arr["current"];
        size_t nelem = current_arr.getNumberOfElements();
        	msg->current.resize(nelem);
        	std::copy(current_arr.begin(), current_arr.begin()+nelem, msg->current.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'current' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'current' is wrong type; expected a double.");
    }
    try {
        //temperature
        const matlab::data::TypedArray<double> temperature_arr = arr["temperature"];
        size_t nelem = temperature_arr.getNumberOfElements();
        	msg->temperature.resize(nelem);
        	std::copy(temperature_arr.begin(), temperature_arr.begin()+nelem, msg->temperature.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'temperature' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'temperature' is wrong type; expected a double.");
    }
    try {
        //sec_enc_pos
        const matlab::data::TypedArray<double> sec_enc_pos_arr = arr["sec_enc_pos"];
        size_t nelem = sec_enc_pos_arr.getNumberOfElements();
        	msg->sec_enc_pos.resize(nelem);
        	std::copy(sec_enc_pos_arr.begin(), sec_enc_pos_arr.begin()+nelem, msg->sec_enc_pos.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'sec_enc_pos' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'sec_enc_pos' is wrong type; expected a double.");
    }
    try {
        //sec_enc_vel
        const matlab::data::TypedArray<double> sec_enc_vel_arr = arr["sec_enc_vel"];
        size_t nelem = sec_enc_vel_arr.getNumberOfElements();
        	msg->sec_enc_vel.resize(nelem);
        	std::copy(sec_enc_vel_arr.begin(), sec_enc_vel_arr.begin()+nelem, msg->sec_enc_vel.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'sec_enc_vel' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'sec_enc_vel' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_pi3hat_moteus_int_msgs_msg_JointsStates_common::get_arr(MDFactory_T& factory, const pi3hat_moteus_int_msgs::msg::JointsStates* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","name","position","velocity","effort","current","temperature","sec_enc_pos","sec_enc_vel"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("pi3hat_moteus_int_msgs/JointsStates");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // name
    auto currentElement_name = (msg + ctr)->name;
    auto nameoutCell = factory.createCellArray({currentElement_name.size(),1});
    for(size_t idxin = 0; idxin < currentElement_name.size(); ++ idxin){
    	nameoutCell[idxin] = factory.createCharArray(currentElement_name[idxin]);
    }
    outArray[ctr]["name"] = nameoutCell;
    // position
    auto currentElement_position = (msg + ctr)->position;
    outArray[ctr]["position"] = factory.createArray<pi3hat_moteus_int_msgs::msg::JointsStates::_position_type::const_iterator, double>({currentElement_position.size(), 1}, currentElement_position.begin(), currentElement_position.end());
    // velocity
    auto currentElement_velocity = (msg + ctr)->velocity;
    outArray[ctr]["velocity"] = factory.createArray<pi3hat_moteus_int_msgs::msg::JointsStates::_velocity_type::const_iterator, double>({currentElement_velocity.size(), 1}, currentElement_velocity.begin(), currentElement_velocity.end());
    // effort
    auto currentElement_effort = (msg + ctr)->effort;
    outArray[ctr]["effort"] = factory.createArray<pi3hat_moteus_int_msgs::msg::JointsStates::_effort_type::const_iterator, double>({currentElement_effort.size(), 1}, currentElement_effort.begin(), currentElement_effort.end());
    // current
    auto currentElement_current = (msg + ctr)->current;
    outArray[ctr]["current"] = factory.createArray<pi3hat_moteus_int_msgs::msg::JointsStates::_current_type::const_iterator, double>({currentElement_current.size(), 1}, currentElement_current.begin(), currentElement_current.end());
    // temperature
    auto currentElement_temperature = (msg + ctr)->temperature;
    outArray[ctr]["temperature"] = factory.createArray<pi3hat_moteus_int_msgs::msg::JointsStates::_temperature_type::const_iterator, double>({currentElement_temperature.size(), 1}, currentElement_temperature.begin(), currentElement_temperature.end());
    // sec_enc_pos
    auto currentElement_sec_enc_pos = (msg + ctr)->sec_enc_pos;
    outArray[ctr]["sec_enc_pos"] = factory.createArray<pi3hat_moteus_int_msgs::msg::JointsStates::_sec_enc_pos_type::const_iterator, double>({currentElement_sec_enc_pos.size(), 1}, currentElement_sec_enc_pos.begin(), currentElement_sec_enc_pos.end());
    // sec_enc_vel
    auto currentElement_sec_enc_vel = (msg + ctr)->sec_enc_vel;
    outArray[ctr]["sec_enc_vel"] = factory.createArray<pi3hat_moteus_int_msgs::msg::JointsStates::_sec_enc_vel_type::const_iterator, double>({currentElement_sec_enc_vel.size(), 1}, currentElement_sec_enc_vel.begin(), currentElement_sec_enc_vel.end());
    }
    return std::move(outArray);
  } 
class PI3HAT_MOTEUS_INT_MSGS_EXPORT ros2_pi3hat_moteus_int_msgs_JointsStates_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_pi3hat_moteus_int_msgs_JointsStates_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_pi3hat_moteus_int_msgs_JointsStates_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<pi3hat_moteus_int_msgs::msg::JointsStates,ros2_pi3hat_moteus_int_msgs_msg_JointsStates_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_pi3hat_moteus_int_msgs_JointsStates_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<pi3hat_moteus_int_msgs::msg::JointsStates,ros2_pi3hat_moteus_int_msgs_msg_JointsStates_common>>();
  }
  std::shared_ptr<void> ros2_pi3hat_moteus_int_msgs_JointsStates_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<pi3hat_moteus_int_msgs::msg::JointsStates>();
    ros2_pi3hat_moteus_int_msgs_msg_JointsStates_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_pi3hat_moteus_int_msgs_JointsStates_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_pi3hat_moteus_int_msgs_msg_JointsStates_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (pi3hat_moteus_int_msgs::msg::JointsStates*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_pi3hat_moteus_int_msgs_msg_JointsStates_common, MATLABROS2MsgInterface<pi3hat_moteus_int_msgs::msg::JointsStates>)
CLASS_LOADER_REGISTER_CLASS(ros2_pi3hat_moteus_int_msgs_JointsStates_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER