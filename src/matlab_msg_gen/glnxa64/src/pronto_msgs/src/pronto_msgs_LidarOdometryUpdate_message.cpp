// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for pronto_msgs/LidarOdometryUpdate
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
#include "pronto_msgs/msg/lidar_odometry_update.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_msg_LidarOdometryUpdate_common : public MATLABROS2MsgInterface<pronto_msgs::msg::LidarOdometryUpdate> {
  public:
    virtual ~ros2_pronto_msgs_msg_LidarOdometryUpdate_common(){}
    virtual void copy_from_struct(pronto_msgs::msg::LidarOdometryUpdate* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const pronto_msgs::msg::LidarOdometryUpdate* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_pronto_msgs_msg_LidarOdometryUpdate_common::copy_from_struct(pronto_msgs::msg::LidarOdometryUpdate* msg, const matlab::data::Struct& arr,
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
        //curr_timestamp
        const matlab::data::StructArray curr_timestamp_arr = arr["curr_timestamp"];
        auto msgClassPtr_curr_timestamp = getCommonObject<builtin_interfaces::msg::Time>("ros2_builtin_interfaces_msg_Time_common",loader);
        msgClassPtr_curr_timestamp->copy_from_struct(&msg->curr_timestamp,curr_timestamp_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'curr_timestamp' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'curr_timestamp' is wrong type; expected a struct.");
    }
    try {
        //prev_timestamp
        const matlab::data::StructArray prev_timestamp_arr = arr["prev_timestamp"];
        auto msgClassPtr_prev_timestamp = getCommonObject<builtin_interfaces::msg::Time>("ros2_builtin_interfaces_msg_Time_common",loader);
        msgClassPtr_prev_timestamp->copy_from_struct(&msg->prev_timestamp,prev_timestamp_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'prev_timestamp' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'prev_timestamp' is wrong type; expected a struct.");
    }
    try {
        //relative_transform
        const matlab::data::StructArray relative_transform_arr = arr["relative_transform"];
        auto msgClassPtr_relative_transform = getCommonObject<geometry_msgs::msg::Transform>("ros2_geometry_msgs_msg_Transform_common",loader);
        msgClassPtr_relative_transform->copy_from_struct(&msg->relative_transform,relative_transform_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'relative_transform' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'relative_transform' is wrong type; expected a struct.");
    }
    try {
        //covariance
        const matlab::data::TypedArray<double> covariance_arr = arr["covariance"];
        size_t nelem = 36;
        	std::copy(covariance_arr.begin(), covariance_arr.begin()+nelem, msg->covariance.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'covariance' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'covariance' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_pronto_msgs_msg_LidarOdometryUpdate_common::get_arr(MDFactory_T& factory, const pronto_msgs::msg::LidarOdometryUpdate* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","curr_timestamp","prev_timestamp","relative_transform","covariance"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("pronto_msgs/LidarOdometryUpdate");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // curr_timestamp
    auto currentElement_curr_timestamp = (msg + ctr)->curr_timestamp;
    auto msgClassPtr_curr_timestamp = getCommonObject<builtin_interfaces::msg::Time>("ros2_builtin_interfaces_msg_Time_common",loader);
    outArray[ctr]["curr_timestamp"] = msgClassPtr_curr_timestamp->get_arr(factory, &currentElement_curr_timestamp, loader);
    // prev_timestamp
    auto currentElement_prev_timestamp = (msg + ctr)->prev_timestamp;
    auto msgClassPtr_prev_timestamp = getCommonObject<builtin_interfaces::msg::Time>("ros2_builtin_interfaces_msg_Time_common",loader);
    outArray[ctr]["prev_timestamp"] = msgClassPtr_prev_timestamp->get_arr(factory, &currentElement_prev_timestamp, loader);
    // relative_transform
    auto currentElement_relative_transform = (msg + ctr)->relative_transform;
    auto msgClassPtr_relative_transform = getCommonObject<geometry_msgs::msg::Transform>("ros2_geometry_msgs_msg_Transform_common",loader);
    outArray[ctr]["relative_transform"] = msgClassPtr_relative_transform->get_arr(factory, &currentElement_relative_transform, loader);
    // covariance
    auto currentElement_covariance = (msg + ctr)->covariance;
    outArray[ctr]["covariance"] = factory.createArray<pronto_msgs::msg::LidarOdometryUpdate::_covariance_type::const_iterator, double>({currentElement_covariance.size(), 1}, currentElement_covariance.begin(), currentElement_covariance.end());
    }
    return std::move(outArray);
  } 
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_LidarOdometryUpdate_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_pronto_msgs_LidarOdometryUpdate_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_pronto_msgs_LidarOdometryUpdate_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<pronto_msgs::msg::LidarOdometryUpdate,ros2_pronto_msgs_msg_LidarOdometryUpdate_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_pronto_msgs_LidarOdometryUpdate_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<pronto_msgs::msg::LidarOdometryUpdate,ros2_pronto_msgs_msg_LidarOdometryUpdate_common>>();
  }
  std::shared_ptr<void> ros2_pronto_msgs_LidarOdometryUpdate_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<pronto_msgs::msg::LidarOdometryUpdate>();
    ros2_pronto_msgs_msg_LidarOdometryUpdate_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_pronto_msgs_LidarOdometryUpdate_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_pronto_msgs_msg_LidarOdometryUpdate_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (pronto_msgs::msg::LidarOdometryUpdate*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_msg_LidarOdometryUpdate_common, MATLABROS2MsgInterface<pronto_msgs::msg::LidarOdometryUpdate>)
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_LidarOdometryUpdate_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER