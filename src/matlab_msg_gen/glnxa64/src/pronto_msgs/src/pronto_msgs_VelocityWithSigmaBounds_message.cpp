// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for pronto_msgs/VelocityWithSigmaBounds
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
#include "pronto_msgs/msg/velocity_with_sigma_bounds.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_msg_VelocityWithSigmaBounds_common : public MATLABROS2MsgInterface<pronto_msgs::msg::VelocityWithSigmaBounds> {
  public:
    virtual ~ros2_pronto_msgs_msg_VelocityWithSigmaBounds_common(){}
    virtual void copy_from_struct(pronto_msgs::msg::VelocityWithSigmaBounds* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const pronto_msgs::msg::VelocityWithSigmaBounds* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_pronto_msgs_msg_VelocityWithSigmaBounds_common::copy_from_struct(pronto_msgs::msg::VelocityWithSigmaBounds* msg, const matlab::data::Struct& arr,
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
        //velocity_plus_one_sigma
        const matlab::data::StructArray velocity_plus_one_sigma_arr = arr["velocity_plus_one_sigma"];
        auto msgClassPtr_velocity_plus_one_sigma = getCommonObject<geometry_msgs::msg::Vector3>("ros2_geometry_msgs_msg_Vector3_common",loader);
        msgClassPtr_velocity_plus_one_sigma->copy_from_struct(&msg->velocity_plus_one_sigma,velocity_plus_one_sigma_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'velocity_plus_one_sigma' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'velocity_plus_one_sigma' is wrong type; expected a struct.");
    }
    try {
        //velocity_minus_one_sigma
        const matlab::data::StructArray velocity_minus_one_sigma_arr = arr["velocity_minus_one_sigma"];
        auto msgClassPtr_velocity_minus_one_sigma = getCommonObject<geometry_msgs::msg::Vector3>("ros2_geometry_msgs_msg_Vector3_common",loader);
        msgClassPtr_velocity_minus_one_sigma->copy_from_struct(&msg->velocity_minus_one_sigma,velocity_minus_one_sigma_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'velocity_minus_one_sigma' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'velocity_minus_one_sigma' is wrong type; expected a struct.");
    }
    try {
        //plus_one_sigma
        const matlab::data::StructArray plus_one_sigma_arr = arr["plus_one_sigma"];
        auto msgClassPtr_plus_one_sigma = getCommonObject<geometry_msgs::msg::Vector3>("ros2_geometry_msgs_msg_Vector3_common",loader);
        msgClassPtr_plus_one_sigma->copy_from_struct(&msg->plus_one_sigma,plus_one_sigma_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'plus_one_sigma' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'plus_one_sigma' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_pronto_msgs_msg_VelocityWithSigmaBounds_common::get_arr(MDFactory_T& factory, const pronto_msgs::msg::VelocityWithSigmaBounds* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","velocity_plus_one_sigma","velocity_minus_one_sigma","plus_one_sigma"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("pronto_msgs/VelocityWithSigmaBounds");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // velocity_plus_one_sigma
    auto currentElement_velocity_plus_one_sigma = (msg + ctr)->velocity_plus_one_sigma;
    auto msgClassPtr_velocity_plus_one_sigma = getCommonObject<geometry_msgs::msg::Vector3>("ros2_geometry_msgs_msg_Vector3_common",loader);
    outArray[ctr]["velocity_plus_one_sigma"] = msgClassPtr_velocity_plus_one_sigma->get_arr(factory, &currentElement_velocity_plus_one_sigma, loader);
    // velocity_minus_one_sigma
    auto currentElement_velocity_minus_one_sigma = (msg + ctr)->velocity_minus_one_sigma;
    auto msgClassPtr_velocity_minus_one_sigma = getCommonObject<geometry_msgs::msg::Vector3>("ros2_geometry_msgs_msg_Vector3_common",loader);
    outArray[ctr]["velocity_minus_one_sigma"] = msgClassPtr_velocity_minus_one_sigma->get_arr(factory, &currentElement_velocity_minus_one_sigma, loader);
    // plus_one_sigma
    auto currentElement_plus_one_sigma = (msg + ctr)->plus_one_sigma;
    auto msgClassPtr_plus_one_sigma = getCommonObject<geometry_msgs::msg::Vector3>("ros2_geometry_msgs_msg_Vector3_common",loader);
    outArray[ctr]["plus_one_sigma"] = msgClassPtr_plus_one_sigma->get_arr(factory, &currentElement_plus_one_sigma, loader);
    }
    return std::move(outArray);
  } 
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_VelocityWithSigmaBounds_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_pronto_msgs_VelocityWithSigmaBounds_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_pronto_msgs_VelocityWithSigmaBounds_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<pronto_msgs::msg::VelocityWithSigmaBounds,ros2_pronto_msgs_msg_VelocityWithSigmaBounds_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_pronto_msgs_VelocityWithSigmaBounds_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<pronto_msgs::msg::VelocityWithSigmaBounds,ros2_pronto_msgs_msg_VelocityWithSigmaBounds_common>>();
  }
  std::shared_ptr<void> ros2_pronto_msgs_VelocityWithSigmaBounds_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<pronto_msgs::msg::VelocityWithSigmaBounds>();
    ros2_pronto_msgs_msg_VelocityWithSigmaBounds_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_pronto_msgs_VelocityWithSigmaBounds_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_pronto_msgs_msg_VelocityWithSigmaBounds_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (pronto_msgs::msg::VelocityWithSigmaBounds*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_msg_VelocityWithSigmaBounds_common, MATLABROS2MsgInterface<pronto_msgs::msg::VelocityWithSigmaBounds>)
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_VelocityWithSigmaBounds_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER