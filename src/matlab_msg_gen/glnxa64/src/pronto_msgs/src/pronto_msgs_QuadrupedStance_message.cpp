// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for pronto_msgs/QuadrupedStance
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
#include "pronto_msgs/msg/quadruped_stance.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_msg_QuadrupedStance_common : public MATLABROS2MsgInterface<pronto_msgs::msg::QuadrupedStance> {
  public:
    virtual ~ros2_pronto_msgs_msg_QuadrupedStance_common(){}
    virtual void copy_from_struct(pronto_msgs::msg::QuadrupedStance* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const pronto_msgs::msg::QuadrupedStance* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_pronto_msgs_msg_QuadrupedStance_common::copy_from_struct(pronto_msgs::msg::QuadrupedStance* msg, const matlab::data::Struct& arr,
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
        const matlab::data::TypedArray<float> lf_arr = arr["lf"];
        msg->lf = lf_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'lf' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'lf' is wrong type; expected a single.");
    }
    try {
        //rf
        const matlab::data::TypedArray<float> rf_arr = arr["rf"];
        msg->rf = rf_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'rf' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'rf' is wrong type; expected a single.");
    }
    try {
        //lh
        const matlab::data::TypedArray<float> lh_arr = arr["lh"];
        msg->lh = lh_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'lh' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'lh' is wrong type; expected a single.");
    }
    try {
        //rh
        const matlab::data::TypedArray<float> rh_arr = arr["rh"];
        msg->rh = rh_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'rh' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'rh' is wrong type; expected a single.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_pronto_msgs_msg_QuadrupedStance_common::get_arr(MDFactory_T& factory, const pronto_msgs::msg::QuadrupedStance* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","lf","rf","lh","rh"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("pronto_msgs/QuadrupedStance");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // lf
    auto currentElement_lf = (msg + ctr)->lf;
    outArray[ctr]["lf"] = factory.createScalar(currentElement_lf);
    // rf
    auto currentElement_rf = (msg + ctr)->rf;
    outArray[ctr]["rf"] = factory.createScalar(currentElement_rf);
    // lh
    auto currentElement_lh = (msg + ctr)->lh;
    outArray[ctr]["lh"] = factory.createScalar(currentElement_lh);
    // rh
    auto currentElement_rh = (msg + ctr)->rh;
    outArray[ctr]["rh"] = factory.createScalar(currentElement_rh);
    }
    return std::move(outArray);
  } 
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_QuadrupedStance_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_pronto_msgs_QuadrupedStance_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_pronto_msgs_QuadrupedStance_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<pronto_msgs::msg::QuadrupedStance,ros2_pronto_msgs_msg_QuadrupedStance_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_pronto_msgs_QuadrupedStance_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<pronto_msgs::msg::QuadrupedStance,ros2_pronto_msgs_msg_QuadrupedStance_common>>();
  }
  std::shared_ptr<void> ros2_pronto_msgs_QuadrupedStance_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<pronto_msgs::msg::QuadrupedStance>();
    ros2_pronto_msgs_msg_QuadrupedStance_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_pronto_msgs_QuadrupedStance_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_pronto_msgs_msg_QuadrupedStance_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (pronto_msgs::msg::QuadrupedStance*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_msg_QuadrupedStance_common, MATLABROS2MsgInterface<pronto_msgs::msg::QuadrupedStance>)
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_QuadrupedStance_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER