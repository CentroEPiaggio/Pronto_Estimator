// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for pronto_msgs/QuadrupedForceTorqueSensors
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
#include "pronto_msgs/msg/quadruped_force_torque_sensors.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_msg_QuadrupedForceTorqueSensors_common : public MATLABROS2MsgInterface<pronto_msgs::msg::QuadrupedForceTorqueSensors> {
  public:
    virtual ~ros2_pronto_msgs_msg_QuadrupedForceTorqueSensors_common(){}
    virtual void copy_from_struct(pronto_msgs::msg::QuadrupedForceTorqueSensors* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const pronto_msgs::msg::QuadrupedForceTorqueSensors* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_pronto_msgs_msg_QuadrupedForceTorqueSensors_common::copy_from_struct(pronto_msgs::msg::QuadrupedForceTorqueSensors* msg, const matlab::data::Struct& arr,
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
        //lf
        const matlab::data::StructArray lf_arr = arr["lf"];
        auto msgClassPtr_lf = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
        msgClassPtr_lf->copy_from_struct(&msg->lf,lf_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'lf' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'lf' is wrong type; expected a struct.");
    }
    try {
        //rf
        const matlab::data::StructArray rf_arr = arr["rf"];
        auto msgClassPtr_rf = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
        msgClassPtr_rf->copy_from_struct(&msg->rf,rf_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'rf' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'rf' is wrong type; expected a struct.");
    }
    try {
        //lh
        const matlab::data::StructArray lh_arr = arr["lh"];
        auto msgClassPtr_lh = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
        msgClassPtr_lh->copy_from_struct(&msg->lh,lh_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'lh' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'lh' is wrong type; expected a struct.");
    }
    try {
        //rh
        const matlab::data::StructArray rh_arr = arr["rh"];
        auto msgClassPtr_rh = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
        msgClassPtr_rh->copy_from_struct(&msg->rh,rh_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'rh' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'rh' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_pronto_msgs_msg_QuadrupedForceTorqueSensors_common::get_arr(MDFactory_T& factory, const pronto_msgs::msg::QuadrupedForceTorqueSensors* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","lf","rf","lh","rh"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("pronto_msgs/QuadrupedForceTorqueSensors");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // lf
    auto currentElement_lf = (msg + ctr)->lf;
    auto msgClassPtr_lf = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
    outArray[ctr]["lf"] = msgClassPtr_lf->get_arr(factory, &currentElement_lf, loader);
    // rf
    auto currentElement_rf = (msg + ctr)->rf;
    auto msgClassPtr_rf = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
    outArray[ctr]["rf"] = msgClassPtr_rf->get_arr(factory, &currentElement_rf, loader);
    // lh
    auto currentElement_lh = (msg + ctr)->lh;
    auto msgClassPtr_lh = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
    outArray[ctr]["lh"] = msgClassPtr_lh->get_arr(factory, &currentElement_lh, loader);
    // rh
    auto currentElement_rh = (msg + ctr)->rh;
    auto msgClassPtr_rh = getCommonObject<geometry_msgs::msg::Wrench>("ros2_geometry_msgs_msg_Wrench_common",loader);
    outArray[ctr]["rh"] = msgClassPtr_rh->get_arr(factory, &currentElement_rh, loader);
    }
    return std::move(outArray);
  } 
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_QuadrupedForceTorqueSensors_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_pronto_msgs_QuadrupedForceTorqueSensors_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_pronto_msgs_QuadrupedForceTorqueSensors_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<pronto_msgs::msg::QuadrupedForceTorqueSensors,ros2_pronto_msgs_msg_QuadrupedForceTorqueSensors_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_pronto_msgs_QuadrupedForceTorqueSensors_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<pronto_msgs::msg::QuadrupedForceTorqueSensors,ros2_pronto_msgs_msg_QuadrupedForceTorqueSensors_common>>();
  }
  std::shared_ptr<void> ros2_pronto_msgs_QuadrupedForceTorqueSensors_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<pronto_msgs::msg::QuadrupedForceTorqueSensors>();
    ros2_pronto_msgs_msg_QuadrupedForceTorqueSensors_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_pronto_msgs_QuadrupedForceTorqueSensors_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_pronto_msgs_msg_QuadrupedForceTorqueSensors_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (pronto_msgs::msg::QuadrupedForceTorqueSensors*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_msg_QuadrupedForceTorqueSensors_common, MATLABROS2MsgInterface<pronto_msgs::msg::QuadrupedForceTorqueSensors>)
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_QuadrupedForceTorqueSensors_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER