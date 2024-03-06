// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for pronto_msgs/FilterState
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
#include "pronto_msgs/msg/filter_state.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_msg_FilterState_common : public MATLABROS2MsgInterface<pronto_msgs::msg::FilterState> {
  public:
    virtual ~ros2_pronto_msgs_msg_FilterState_common(){}
    virtual void copy_from_struct(pronto_msgs::msg::FilterState* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const pronto_msgs::msg::FilterState* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_pronto_msgs_msg_FilterState_common::copy_from_struct(pronto_msgs::msg::FilterState* msg, const matlab::data::Struct& arr,
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
        //quat
        const matlab::data::StructArray quat_arr = arr["quat"];
        auto msgClassPtr_quat = getCommonObject<geometry_msgs::msg::Quaternion>("ros2_geometry_msgs_msg_Quaternion_common",loader);
        msgClassPtr_quat->copy_from_struct(&msg->quat,quat_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'quat' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'quat' is wrong type; expected a struct.");
    }
    try {
        //state
        const matlab::data::TypedArray<double> state_arr = arr["state"];
        size_t nelem = state_arr.getNumberOfElements();
        	msg->state.resize(nelem);
        	std::copy(state_arr.begin(), state_arr.begin()+nelem, msg->state.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'state' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'state' is wrong type; expected a double.");
    }
    try {
        //cov
        const matlab::data::TypedArray<double> cov_arr = arr["cov"];
        size_t nelem = cov_arr.getNumberOfElements();
        	msg->cov.resize(nelem);
        	std::copy(cov_arr.begin(), cov_arr.begin()+nelem, msg->cov.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'cov' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'cov' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_pronto_msgs_msg_FilterState_common::get_arr(MDFactory_T& factory, const pronto_msgs::msg::FilterState* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","quat","state","cov"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("pronto_msgs/FilterState");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // quat
    auto currentElement_quat = (msg + ctr)->quat;
    auto msgClassPtr_quat = getCommonObject<geometry_msgs::msg::Quaternion>("ros2_geometry_msgs_msg_Quaternion_common",loader);
    outArray[ctr]["quat"] = msgClassPtr_quat->get_arr(factory, &currentElement_quat, loader);
    // state
    auto currentElement_state = (msg + ctr)->state;
    outArray[ctr]["state"] = factory.createArray<pronto_msgs::msg::FilterState::_state_type::const_iterator, double>({currentElement_state.size(), 1}, currentElement_state.begin(), currentElement_state.end());
    // cov
    auto currentElement_cov = (msg + ctr)->cov;
    outArray[ctr]["cov"] = factory.createArray<pronto_msgs::msg::FilterState::_cov_type::const_iterator, double>({currentElement_cov.size(), 1}, currentElement_cov.begin(), currentElement_cov.end());
    }
    return std::move(outArray);
  } 
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_FilterState_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_pronto_msgs_FilterState_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_pronto_msgs_FilterState_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<pronto_msgs::msg::FilterState,ros2_pronto_msgs_msg_FilterState_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_pronto_msgs_FilterState_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<pronto_msgs::msg::FilterState,ros2_pronto_msgs_msg_FilterState_common>>();
  }
  std::shared_ptr<void> ros2_pronto_msgs_FilterState_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<pronto_msgs::msg::FilterState>();
    ros2_pronto_msgs_msg_FilterState_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_pronto_msgs_FilterState_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_pronto_msgs_msg_FilterState_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (pronto_msgs::msg::FilterState*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_msg_FilterState_common, MATLABROS2MsgInterface<pronto_msgs::msg::FilterState>)
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_FilterState_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER