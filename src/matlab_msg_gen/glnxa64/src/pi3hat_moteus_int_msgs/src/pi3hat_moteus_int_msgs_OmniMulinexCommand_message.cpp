// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for pi3hat_moteus_int_msgs/OmniMulinexCommand
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
#include "pi3hat_moteus_int_msgs/msg/omni_mulinex_command.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class PI3HAT_MOTEUS_INT_MSGS_EXPORT ros2_pi3hat_moteus_int_msgs_msg_OmniMulinexCommand_common : public MATLABROS2MsgInterface<pi3hat_moteus_int_msgs::msg::OmniMulinexCommand> {
  public:
    virtual ~ros2_pi3hat_moteus_int_msgs_msg_OmniMulinexCommand_common(){}
    virtual void copy_from_struct(pi3hat_moteus_int_msgs::msg::OmniMulinexCommand* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const pi3hat_moteus_int_msgs::msg::OmniMulinexCommand* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_pi3hat_moteus_int_msgs_msg_OmniMulinexCommand_common::copy_from_struct(pi3hat_moteus_int_msgs::msg::OmniMulinexCommand* msg, const matlab::data::Struct& arr,
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
        //v_x
        const matlab::data::TypedArray<double> v_x_arr = arr["v_x"];
        msg->v_x = v_x_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'v_x' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'v_x' is wrong type; expected a double.");
    }
    try {
        //v_y
        const matlab::data::TypedArray<double> v_y_arr = arr["v_y"];
        msg->v_y = v_y_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'v_y' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'v_y' is wrong type; expected a double.");
    }
    try {
        //omega
        const matlab::data::TypedArray<double> omega_arr = arr["omega"];
        msg->omega = omega_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'omega' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'omega' is wrong type; expected a double.");
    }
    try {
        //height_rate
        const matlab::data::TypedArray<double> height_rate_arr = arr["height_rate"];
        msg->height_rate = height_rate_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'height_rate' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'height_rate' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_pi3hat_moteus_int_msgs_msg_OmniMulinexCommand_common::get_arr(MDFactory_T& factory, const pi3hat_moteus_int_msgs::msg::OmniMulinexCommand* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","v_x","v_y","omega","height_rate"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("pi3hat_moteus_int_msgs/OmniMulinexCommand");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // v_x
    auto currentElement_v_x = (msg + ctr)->v_x;
    outArray[ctr]["v_x"] = factory.createScalar(currentElement_v_x);
    // v_y
    auto currentElement_v_y = (msg + ctr)->v_y;
    outArray[ctr]["v_y"] = factory.createScalar(currentElement_v_y);
    // omega
    auto currentElement_omega = (msg + ctr)->omega;
    outArray[ctr]["omega"] = factory.createScalar(currentElement_omega);
    // height_rate
    auto currentElement_height_rate = (msg + ctr)->height_rate;
    outArray[ctr]["height_rate"] = factory.createScalar(currentElement_height_rate);
    }
    return std::move(outArray);
  } 
class PI3HAT_MOTEUS_INT_MSGS_EXPORT ros2_pi3hat_moteus_int_msgs_OmniMulinexCommand_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_pi3hat_moteus_int_msgs_OmniMulinexCommand_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_pi3hat_moteus_int_msgs_OmniMulinexCommand_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<pi3hat_moteus_int_msgs::msg::OmniMulinexCommand,ros2_pi3hat_moteus_int_msgs_msg_OmniMulinexCommand_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_pi3hat_moteus_int_msgs_OmniMulinexCommand_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<pi3hat_moteus_int_msgs::msg::OmniMulinexCommand,ros2_pi3hat_moteus_int_msgs_msg_OmniMulinexCommand_common>>();
  }
  std::shared_ptr<void> ros2_pi3hat_moteus_int_msgs_OmniMulinexCommand_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<pi3hat_moteus_int_msgs::msg::OmniMulinexCommand>();
    ros2_pi3hat_moteus_int_msgs_msg_OmniMulinexCommand_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_pi3hat_moteus_int_msgs_OmniMulinexCommand_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_pi3hat_moteus_int_msgs_msg_OmniMulinexCommand_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (pi3hat_moteus_int_msgs::msg::OmniMulinexCommand*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_pi3hat_moteus_int_msgs_msg_OmniMulinexCommand_common, MATLABROS2MsgInterface<pi3hat_moteus_int_msgs::msg::OmniMulinexCommand>)
CLASS_LOADER_REGISTER_CLASS(ros2_pi3hat_moteus_int_msgs_OmniMulinexCommand_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER