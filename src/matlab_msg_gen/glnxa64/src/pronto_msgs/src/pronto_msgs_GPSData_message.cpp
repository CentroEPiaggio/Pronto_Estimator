// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for pronto_msgs/GPSData
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
#include "pronto_msgs/msg/gps_data.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_msg_GPSData_common : public MATLABROS2MsgInterface<pronto_msgs::msg::GPSData> {
  public:
    virtual ~ros2_pronto_msgs_msg_GPSData_common(){}
    virtual void copy_from_struct(pronto_msgs::msg::GPSData* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const pronto_msgs::msg::GPSData* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_pronto_msgs_msg_GPSData_common::copy_from_struct(pronto_msgs::msg::GPSData* msg, const matlab::data::Struct& arr,
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
        //gps_lock
        const matlab::data::TypedArray<int32_t> gps_lock_arr = arr["gps_lock"];
        msg->gps_lock = gps_lock_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'gps_lock' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'gps_lock' is wrong type; expected a int32.");
    }
    try {
        //longitude
        const matlab::data::TypedArray<double> longitude_arr = arr["longitude"];
        msg->longitude = longitude_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'longitude' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'longitude' is wrong type; expected a double.");
    }
    try {
        //latitude
        const matlab::data::TypedArray<double> latitude_arr = arr["latitude"];
        msg->latitude = latitude_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'latitude' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'latitude' is wrong type; expected a double.");
    }
    try {
        //elev
        const matlab::data::TypedArray<double> elev_arr = arr["elev"];
        msg->elev = elev_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'elev' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'elev' is wrong type; expected a double.");
    }
    try {
        //horizontal_accuracy
        const matlab::data::TypedArray<double> horizontal_accuracy_arr = arr["horizontal_accuracy"];
        msg->horizontal_accuracy = horizontal_accuracy_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'horizontal_accuracy' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'horizontal_accuracy' is wrong type; expected a double.");
    }
    try {
        //vertical_accuracy
        const matlab::data::TypedArray<double> vertical_accuracy_arr = arr["vertical_accuracy"];
        msg->vertical_accuracy = vertical_accuracy_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'vertical_accuracy' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'vertical_accuracy' is wrong type; expected a double.");
    }
    try {
        //num_satellites
        const matlab::data::TypedArray<uint32_t> num_satellites_arr = arr["num_satellites"];
        msg->num_satellites = num_satellites_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'num_satellites' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'num_satellites' is wrong type; expected a uint32.");
    }
    try {
        //speed
        const matlab::data::TypedArray<double> speed_arr = arr["speed"];
        msg->speed = speed_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'speed' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'speed' is wrong type; expected a double.");
    }
    try {
        //heading
        const matlab::data::TypedArray<double> heading_arr = arr["heading"];
        msg->heading = heading_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'heading' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'heading' is wrong type; expected a double.");
    }
    try {
        //xyz_pos
        const matlab::data::TypedArray<double> xyz_pos_arr = arr["xyz_pos"];
        size_t nelem = 3;
        	std::copy(xyz_pos_arr.begin(), xyz_pos_arr.begin()+nelem, msg->xyz_pos.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'xyz_pos' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'xyz_pos' is wrong type; expected a double.");
    }
    try {
        //gps_time
        const matlab::data::TypedArray<double> gps_time_arr = arr["gps_time"];
        msg->gps_time = gps_time_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'gps_time' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'gps_time' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_pronto_msgs_msg_GPSData_common::get_arr(MDFactory_T& factory, const pronto_msgs::msg::GPSData* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","utime","gps_lock","longitude","latitude","elev","horizontal_accuracy","vertical_accuracy","num_satellites","speed","heading","xyz_pos","gps_time"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("pronto_msgs/GPSData");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // utime
    auto currentElement_utime = (msg + ctr)->utime;
    outArray[ctr]["utime"] = factory.createScalar(currentElement_utime);
    // gps_lock
    auto currentElement_gps_lock = (msg + ctr)->gps_lock;
    outArray[ctr]["gps_lock"] = factory.createScalar(currentElement_gps_lock);
    // longitude
    auto currentElement_longitude = (msg + ctr)->longitude;
    outArray[ctr]["longitude"] = factory.createScalar(currentElement_longitude);
    // latitude
    auto currentElement_latitude = (msg + ctr)->latitude;
    outArray[ctr]["latitude"] = factory.createScalar(currentElement_latitude);
    // elev
    auto currentElement_elev = (msg + ctr)->elev;
    outArray[ctr]["elev"] = factory.createScalar(currentElement_elev);
    // horizontal_accuracy
    auto currentElement_horizontal_accuracy = (msg + ctr)->horizontal_accuracy;
    outArray[ctr]["horizontal_accuracy"] = factory.createScalar(currentElement_horizontal_accuracy);
    // vertical_accuracy
    auto currentElement_vertical_accuracy = (msg + ctr)->vertical_accuracy;
    outArray[ctr]["vertical_accuracy"] = factory.createScalar(currentElement_vertical_accuracy);
    // num_satellites
    auto currentElement_num_satellites = (msg + ctr)->num_satellites;
    outArray[ctr]["num_satellites"] = factory.createScalar(currentElement_num_satellites);
    // speed
    auto currentElement_speed = (msg + ctr)->speed;
    outArray[ctr]["speed"] = factory.createScalar(currentElement_speed);
    // heading
    auto currentElement_heading = (msg + ctr)->heading;
    outArray[ctr]["heading"] = factory.createScalar(currentElement_heading);
    // xyz_pos
    auto currentElement_xyz_pos = (msg + ctr)->xyz_pos;
    outArray[ctr]["xyz_pos"] = factory.createArray<pronto_msgs::msg::GPSData::_xyz_pos_type::const_iterator, double>({currentElement_xyz_pos.size(), 1}, currentElement_xyz_pos.begin(), currentElement_xyz_pos.end());
    // gps_time
    auto currentElement_gps_time = (msg + ctr)->gps_time;
    outArray[ctr]["gps_time"] = factory.createScalar(currentElement_gps_time);
    }
    return std::move(outArray);
  } 
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_GPSData_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_pronto_msgs_GPSData_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_pronto_msgs_GPSData_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<pronto_msgs::msg::GPSData,ros2_pronto_msgs_msg_GPSData_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_pronto_msgs_GPSData_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<pronto_msgs::msg::GPSData,ros2_pronto_msgs_msg_GPSData_common>>();
  }
  std::shared_ptr<void> ros2_pronto_msgs_GPSData_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<pronto_msgs::msg::GPSData>();
    ros2_pronto_msgs_msg_GPSData_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_pronto_msgs_GPSData_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_pronto_msgs_msg_GPSData_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (pronto_msgs::msg::GPSData*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_msg_GPSData_common, MATLABROS2MsgInterface<pronto_msgs::msg::GPSData>)
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_GPSData_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER