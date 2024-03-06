// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for pronto_msgs/ControllerFootContact
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
#include "pronto_msgs/msg/controller_foot_contact.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_msg_ControllerFootContact_common : public MATLABROS2MsgInterface<pronto_msgs::msg::ControllerFootContact> {
  public:
    virtual ~ros2_pronto_msgs_msg_ControllerFootContact_common(){}
    virtual void copy_from_struct(pronto_msgs::msg::ControllerFootContact* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const pronto_msgs::msg::ControllerFootContact* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_pronto_msgs_msg_ControllerFootContact_common::copy_from_struct(pronto_msgs::msg::ControllerFootContact* msg, const matlab::data::Struct& arr,
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
        //num_right_foot_contacts
        const matlab::data::TypedArray<int32_t> num_right_foot_contacts_arr = arr["num_right_foot_contacts"];
        msg->num_right_foot_contacts = num_right_foot_contacts_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'num_right_foot_contacts' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'num_right_foot_contacts' is wrong type; expected a int32.");
    }
    try {
        //right_foot_contacts
        const matlab::data::TypedArray<double> right_foot_contacts_arr = arr["right_foot_contacts"];
        size_t nelem = right_foot_contacts_arr.getNumberOfElements();
        	msg->right_foot_contacts.resize(nelem);
        	std::copy(right_foot_contacts_arr.begin(), right_foot_contacts_arr.begin()+nelem, msg->right_foot_contacts.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'right_foot_contacts' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'right_foot_contacts' is wrong type; expected a double.");
    }
    try {
        //num_left_foot_contacts
        const matlab::data::TypedArray<int32_t> num_left_foot_contacts_arr = arr["num_left_foot_contacts"];
        msg->num_left_foot_contacts = num_left_foot_contacts_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'num_left_foot_contacts' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'num_left_foot_contacts' is wrong type; expected a int32.");
    }
    try {
        //left_foot_contacts
        const matlab::data::TypedArray<double> left_foot_contacts_arr = arr["left_foot_contacts"];
        size_t nelem = left_foot_contacts_arr.getNumberOfElements();
        	msg->left_foot_contacts.resize(nelem);
        	std::copy(left_foot_contacts_arr.begin(), left_foot_contacts_arr.begin()+nelem, msg->left_foot_contacts.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'left_foot_contacts' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'left_foot_contacts' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_pronto_msgs_msg_ControllerFootContact_common::get_arr(MDFactory_T& factory, const pronto_msgs::msg::ControllerFootContact* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","num_right_foot_contacts","right_foot_contacts","num_left_foot_contacts","left_foot_contacts"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("pronto_msgs/ControllerFootContact");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // num_right_foot_contacts
    auto currentElement_num_right_foot_contacts = (msg + ctr)->num_right_foot_contacts;
    outArray[ctr]["num_right_foot_contacts"] = factory.createScalar(currentElement_num_right_foot_contacts);
    // right_foot_contacts
    auto currentElement_right_foot_contacts = (msg + ctr)->right_foot_contacts;
    outArray[ctr]["right_foot_contacts"] = factory.createArray<pronto_msgs::msg::ControllerFootContact::_right_foot_contacts_type::const_iterator, double>({currentElement_right_foot_contacts.size(), 1}, currentElement_right_foot_contacts.begin(), currentElement_right_foot_contacts.end());
    // num_left_foot_contacts
    auto currentElement_num_left_foot_contacts = (msg + ctr)->num_left_foot_contacts;
    outArray[ctr]["num_left_foot_contacts"] = factory.createScalar(currentElement_num_left_foot_contacts);
    // left_foot_contacts
    auto currentElement_left_foot_contacts = (msg + ctr)->left_foot_contacts;
    outArray[ctr]["left_foot_contacts"] = factory.createArray<pronto_msgs::msg::ControllerFootContact::_left_foot_contacts_type::const_iterator, double>({currentElement_left_foot_contacts.size(), 1}, currentElement_left_foot_contacts.begin(), currentElement_left_foot_contacts.end());
    }
    return std::move(outArray);
  } 
class PRONTO_MSGS_EXPORT ros2_pronto_msgs_ControllerFootContact_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_pronto_msgs_ControllerFootContact_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_pronto_msgs_ControllerFootContact_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<pronto_msgs::msg::ControllerFootContact,ros2_pronto_msgs_msg_ControllerFootContact_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_pronto_msgs_ControllerFootContact_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<pronto_msgs::msg::ControllerFootContact,ros2_pronto_msgs_msg_ControllerFootContact_common>>();
  }
  std::shared_ptr<void> ros2_pronto_msgs_ControllerFootContact_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<pronto_msgs::msg::ControllerFootContact>();
    ros2_pronto_msgs_msg_ControllerFootContact_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_pronto_msgs_ControllerFootContact_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_pronto_msgs_msg_ControllerFootContact_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (pronto_msgs::msg::ControllerFootContact*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_msg_ControllerFootContact_common, MATLABROS2MsgInterface<pronto_msgs::msg::ControllerFootContact>)
CLASS_LOADER_REGISTER_CLASS(ros2_pronto_msgs_ControllerFootContact_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER