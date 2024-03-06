// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for pronto_msgs/BipedForceTorqueSensors
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
#include "pronto_msgs/msg/biped_force_torque_sensors.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_msg_BipedForceTorqueSensors_common : public MATLABROS2MsgInterface<pronto_msgs::msg::BipedForceTorqueSensors> {
  public:
    virtual ~ros2_pronto_msgs_msg_BipedForceTorqueSensors_common(){}
    virtual void copy_from_struct(pronto_msgs::msg::BipedForceTorqueSensors* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const pronto_msgs::msg::BipedForceTorqueSensors* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_pronto_msgs_msg_BipedForceTorqueSensors_common::copy_from_struct(pronto_msgs::msg::BipedForceTorqueSensors* msg, const matlab::data::Struct& arr,
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
        //l_foot
        const matlab::data::StructArray l_foot_arr = arr["l_foot"];
        auto msgClassPtr_l_foot = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
        msgClassPtr_l_foot->copy_from_struct(&msg->l_foot,l_foot_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'l_foot' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'l_foot' is wrong type; expected a struct.");
    }
    try {
        //r_foot
        const matlab::data::StructArray r_foot_arr = arr["r_foot"];
        auto msgClassPtr_r_foot = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
        msgClassPtr_r_foot->copy_from_struct(&msg->r_foot,r_foot_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'r_foot' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'r_foot' is wrong type; expected a struct.");
    }
    try {
        //l_hand
        const matlab::data::StructArray l_hand_arr = arr["l_hand"];
        auto msgClassPtr_l_hand = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
        msgClassPtr_l_hand->copy_from_struct(&msg->l_hand,l_hand_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'l_hand' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'l_hand' is wrong type; expected a struct.");
    }
    try {
        //r_hand
        const matlab::data::StructArray r_hand_arr = arr["r_hand"];
        auto msgClassPtr_r_hand = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
        msgClassPtr_r_hand->copy_from_struct(&msg->r_hand,r_hand_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'r_hand' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'r_hand' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_pronto_msgs_msg_BipedForceTorqueSensors_common::get_arr(MDFactory_T& factory, const pronto_msgs::msg::BipedForceTorqueSensors* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","l_foot","r_foot","l_hand","r_hand"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("pronto_msgs/BipedForceTorqueSensors");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // l_foot
    auto currentElement_l_foot = (msg + ctr)->l_foot;
    auto msgClassPtr_l_foot = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
    outArray[ctr]["l_foot"] = msgClassPtr_l_foot->get_arr(factory, &currentElement_l_foot, loader);
    // r_foot
    auto currentElement_r_foot = (msg + ctr)->r_foot;
    auto msgClassPtr_r_foot = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
    outArray[ctr]["r_foot"] = msgClassPtr_r_foot->get_arr(factory, &currentElement_r_foot, loader);
    // l_hand
    auto currentElement_l_hand = (msg + ctr)->l_hand;
    auto msgClassPtr_l_hand = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
    outArray[ctr]["l_hand"] = msgClassPtr_l_hand->get_arr(factory, &currentElement_l_hand, loader);
    // r_hand
    auto currentElement_r_hand = (msg + ctr)->r_hand;
    auto msgClassPtr_r_hand = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
    outArray[ctr]["r_hand"] = msgClassPtr_r_hand->get_arr(factory, &currentElement_r_hand, loader);
    }
    return std::move(outArray);
  } 
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_BipedForceTorqueSensors_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_pronto_msgs_BipedForceTorqueSensors_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_pronto_msgs_BipedForceTorqueSensors_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<pronto_msgs::msg::BipedForceTorqueSensors,ros2_pronto_msgs_msg_BipedForceTorqueSensors_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_pronto_msgs_BipedForceTorqueSensors_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<pronto_msgs::msg::BipedForceTorqueSensors,ros2_pronto_msgs_msg_BipedForceTorqueSensors_common>>();
  }
  std::shared_ptr<void> ros2_pronto_msgs_BipedForceTorqueSensors_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<pronto_msgs::msg::BipedForceTorqueSensors>();
    ros2_pronto_msgs_msg_BipedForceTorqueSensors_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_pronto_msgs_BipedForceTorqueSensors_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_pronto_msgs_msg_BipedForceTorqueSensors_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (pronto_msgs::msg::BipedForceTorqueSensors*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_msg_BipedForceTorqueSensors_common, MATLABROS2MsgInterface<pronto_msgs::msg::BipedForceTorqueSensors>)
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_BipedForceTorqueSensors_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER