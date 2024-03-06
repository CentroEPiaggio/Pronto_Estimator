// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for pronto_msgs/IndexedMeasurement
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
#include "pronto_msgs/msg/indexed_measurement.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_msg_IndexedMeasurement_common : public MATLABROS2MsgInterface<pronto_msgs::msg::IndexedMeasurement> {
  public:
    virtual ~ros2_pronto_msgs_msg_IndexedMeasurement_common(){}
    virtual void copy_from_struct(pronto_msgs::msg::IndexedMeasurement* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const pronto_msgs::msg::IndexedMeasurement* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_pronto_msgs_msg_IndexedMeasurement_common::copy_from_struct(pronto_msgs::msg::IndexedMeasurement* msg, const matlab::data::Struct& arr,
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
        //utime
        const matlab::data::TypedArray<uint64_t> utime_arr = arr["utime"];
        msg->utime = utime_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'utime' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'utime' is wrong type; expected a uint64.");
    }
    try {
        //state_utime
        const matlab::data::TypedArray<uint64_t> state_utime_arr = arr["state_utime"];
        msg->state_utime = state_utime_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'state_utime' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'state_utime' is wrong type; expected a uint64.");
    }
    try {
        //z_effective
        const matlab::data::TypedArray<double> z_effective_arr = arr["z_effective"];
        size_t nelem = z_effective_arr.getNumberOfElements();
        	msg->z_effective.resize(nelem);
        	std::copy(z_effective_arr.begin(), z_effective_arr.begin()+nelem, msg->z_effective.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'z_effective' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'z_effective' is wrong type; expected a double.");
    }
    try {
        //z_indices
        const matlab::data::TypedArray<int32_t> z_indices_arr = arr["z_indices"];
        size_t nelem = z_indices_arr.getNumberOfElements();
        	msg->z_indices.resize(nelem);
        	std::copy(z_indices_arr.begin(), z_indices_arr.begin()+nelem, msg->z_indices.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'z_indices' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'z_indices' is wrong type; expected a int32.");
    }
    try {
        //r_effective
        const matlab::data::TypedArray<double> r_effective_arr = arr["r_effective"];
        size_t nelem = r_effective_arr.getNumberOfElements();
        	msg->r_effective.resize(nelem);
        	std::copy(r_effective_arr.begin(), r_effective_arr.begin()+nelem, msg->r_effective.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'r_effective' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'r_effective' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_pronto_msgs_msg_IndexedMeasurement_common::get_arr(MDFactory_T& factory, const pronto_msgs::msg::IndexedMeasurement* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","utime","state_utime","z_effective","z_indices","r_effective"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("pronto_msgs/IndexedMeasurement");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // utime
    auto currentElement_utime = (msg + ctr)->utime;
    outArray[ctr]["utime"] = factory.createScalar(currentElement_utime);
    // state_utime
    auto currentElement_state_utime = (msg + ctr)->state_utime;
    outArray[ctr]["state_utime"] = factory.createScalar(currentElement_state_utime);
    // z_effective
    auto currentElement_z_effective = (msg + ctr)->z_effective;
    outArray[ctr]["z_effective"] = factory.createArray<pronto_msgs::msg::IndexedMeasurement::_z_effective_type::const_iterator, double>({currentElement_z_effective.size(), 1}, currentElement_z_effective.begin(), currentElement_z_effective.end());
    // z_indices
    auto currentElement_z_indices = (msg + ctr)->z_indices;
    outArray[ctr]["z_indices"] = factory.createArray<pronto_msgs::msg::IndexedMeasurement::_z_indices_type::const_iterator, int32_t>({currentElement_z_indices.size(), 1}, currentElement_z_indices.begin(), currentElement_z_indices.end());
    // r_effective
    auto currentElement_r_effective = (msg + ctr)->r_effective;
    outArray[ctr]["r_effective"] = factory.createArray<pronto_msgs::msg::IndexedMeasurement::_r_effective_type::const_iterator, double>({currentElement_r_effective.size(), 1}, currentElement_r_effective.begin(), currentElement_r_effective.end());
    }
    return std::move(outArray);
  } 
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_IndexedMeasurement_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_pronto_msgs_IndexedMeasurement_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_pronto_msgs_IndexedMeasurement_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<pronto_msgs::msg::IndexedMeasurement,ros2_pronto_msgs_msg_IndexedMeasurement_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_pronto_msgs_IndexedMeasurement_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<pronto_msgs::msg::IndexedMeasurement,ros2_pronto_msgs_msg_IndexedMeasurement_common>>();
  }
  std::shared_ptr<void> ros2_pronto_msgs_IndexedMeasurement_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<pronto_msgs::msg::IndexedMeasurement>();
    ros2_pronto_msgs_msg_IndexedMeasurement_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_pronto_msgs_IndexedMeasurement_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_pronto_msgs_msg_IndexedMeasurement_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (pronto_msgs::msg::IndexedMeasurement*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_msg_IndexedMeasurement_common, MATLABROS2MsgInterface<pronto_msgs::msg::IndexedMeasurement>)
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_IndexedMeasurement_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER