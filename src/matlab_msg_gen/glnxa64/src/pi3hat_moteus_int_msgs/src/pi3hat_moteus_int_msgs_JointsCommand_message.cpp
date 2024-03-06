// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for pi3hat_moteus_int_msgs/JointsCommand
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
#include "pi3hat_moteus_int_msgs/msg/joints_command.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class PI3HAT_MOTEUS_INT_MSGS_EXPORT ros2_pi3hat_moteus_int_msgs_msg_JointsCommand_common : public MATLABROS2MsgInterface<pi3hat_moteus_int_msgs::msg::JointsCommand> {
  public:
    virtual ~ros2_pi3hat_moteus_int_msgs_msg_JointsCommand_common(){}
    virtual void copy_from_struct(pi3hat_moteus_int_msgs::msg::JointsCommand* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const pi3hat_moteus_int_msgs::msg::JointsCommand* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_pi3hat_moteus_int_msgs_msg_JointsCommand_common::copy_from_struct(pi3hat_moteus_int_msgs::msg::JointsCommand* msg, const matlab::data::Struct& arr,
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
        //kp_scale
        const matlab::data::TypedArray<double> kp_scale_arr = arr["kp_scale"];
        size_t nelem = kp_scale_arr.getNumberOfElements();
        	msg->kp_scale.resize(nelem);
        	std::copy(kp_scale_arr.begin(), kp_scale_arr.begin()+nelem, msg->kp_scale.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'kp_scale' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'kp_scale' is wrong type; expected a double.");
    }
    try {
        //kd_scale
        const matlab::data::TypedArray<double> kd_scale_arr = arr["kd_scale"];
        size_t nelem = kd_scale_arr.getNumberOfElements();
        	msg->kd_scale.resize(nelem);
        	std::copy(kd_scale_arr.begin(), kd_scale_arr.begin()+nelem, msg->kd_scale.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'kd_scale' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'kd_scale' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_pi3hat_moteus_int_msgs_msg_JointsCommand_common::get_arr(MDFactory_T& factory, const pi3hat_moteus_int_msgs::msg::JointsCommand* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","name","position","velocity","effort","kp_scale","kd_scale"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("pi3hat_moteus_int_msgs/JointsCommand");
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
    outArray[ctr]["position"] = factory.createArray<pi3hat_moteus_int_msgs::msg::JointsCommand::_position_type::const_iterator, double>({currentElement_position.size(), 1}, currentElement_position.begin(), currentElement_position.end());
    // velocity
    auto currentElement_velocity = (msg + ctr)->velocity;
    outArray[ctr]["velocity"] = factory.createArray<pi3hat_moteus_int_msgs::msg::JointsCommand::_velocity_type::const_iterator, double>({currentElement_velocity.size(), 1}, currentElement_velocity.begin(), currentElement_velocity.end());
    // effort
    auto currentElement_effort = (msg + ctr)->effort;
    outArray[ctr]["effort"] = factory.createArray<pi3hat_moteus_int_msgs::msg::JointsCommand::_effort_type::const_iterator, double>({currentElement_effort.size(), 1}, currentElement_effort.begin(), currentElement_effort.end());
    // kp_scale
    auto currentElement_kp_scale = (msg + ctr)->kp_scale;
    outArray[ctr]["kp_scale"] = factory.createArray<pi3hat_moteus_int_msgs::msg::JointsCommand::_kp_scale_type::const_iterator, double>({currentElement_kp_scale.size(), 1}, currentElement_kp_scale.begin(), currentElement_kp_scale.end());
    // kd_scale
    auto currentElement_kd_scale = (msg + ctr)->kd_scale;
    outArray[ctr]["kd_scale"] = factory.createArray<pi3hat_moteus_int_msgs::msg::JointsCommand::_kd_scale_type::const_iterator, double>({currentElement_kd_scale.size(), 1}, currentElement_kd_scale.begin(), currentElement_kd_scale.end());
    }
    return std::move(outArray);
  } 
class PI3HAT_MOTEUS_INT_MSGS_EXPORT ros2_pi3hat_moteus_int_msgs_JointsCommand_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_pi3hat_moteus_int_msgs_JointsCommand_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_pi3hat_moteus_int_msgs_JointsCommand_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<pi3hat_moteus_int_msgs::msg::JointsCommand,ros2_pi3hat_moteus_int_msgs_msg_JointsCommand_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_pi3hat_moteus_int_msgs_JointsCommand_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<pi3hat_moteus_int_msgs::msg::JointsCommand,ros2_pi3hat_moteus_int_msgs_msg_JointsCommand_common>>();
  }
  std::shared_ptr<void> ros2_pi3hat_moteus_int_msgs_JointsCommand_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<pi3hat_moteus_int_msgs::msg::JointsCommand>();
    ros2_pi3hat_moteus_int_msgs_msg_JointsCommand_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_pi3hat_moteus_int_msgs_JointsCommand_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_pi3hat_moteus_int_msgs_msg_JointsCommand_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (pi3hat_moteus_int_msgs::msg::JointsCommand*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_pi3hat_moteus_int_msgs_msg_JointsCommand_common, MATLABROS2MsgInterface<pi3hat_moteus_int_msgs::msg::JointsCommand>)
CLASS_LOADER_REGISTER_CLASS(ros2_pi3hat_moteus_int_msgs_JointsCommand_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER