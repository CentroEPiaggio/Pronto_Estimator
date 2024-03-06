// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for pi3hat_moteus_int_msgs/PacketPass
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
#include "pi3hat_moteus_int_msgs/msg/packet_pass.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class PI3HAT_MOTEUS_INT_MSGS_EXPORT ros2_pi3hat_moteus_int_msgs_msg_PacketPass_common : public MATLABROS2MsgInterface<pi3hat_moteus_int_msgs::msg::PacketPass> {
  public:
    virtual ~ros2_pi3hat_moteus_int_msgs_msg_PacketPass_common(){}
    virtual void copy_from_struct(pi3hat_moteus_int_msgs::msg::PacketPass* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const pi3hat_moteus_int_msgs::msg::PacketPass* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_pi3hat_moteus_int_msgs_msg_PacketPass_common::copy_from_struct(pi3hat_moteus_int_msgs::msg::PacketPass* msg, const matlab::data::Struct& arr,
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
        //valid
        const matlab::data::TypedArray<double> valid_arr = arr["valid"];
        msg->valid = valid_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'valid' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'valid' is wrong type; expected a double.");
    }
    try {
        //cycle_dur
        const matlab::data::TypedArray<double> cycle_dur_arr = arr["cycle_dur"];
        msg->cycle_dur = cycle_dur_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'cycle_dur' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'cycle_dur' is wrong type; expected a double.");
    }
    try {
        //name
        const matlab::data::CellArray name_cellarr = arr["name"];
        size_t nelem = name_cellarr.getNumberOfElements();
        for (size_t idx=0; idx < nelem; ++idx){
        	const matlab::data::CharArray name_arr = name_cellarr[idx];
        	msg->name.push_back(name_arr.toAscii());
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'name' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'name' is wrong type; expected a string.");
    }
    try {
        //pack_loss
        const matlab::data::TypedArray<double> pack_loss_arr = arr["pack_loss"];
        size_t nelem = pack_loss_arr.getNumberOfElements();
        	msg->pack_loss.resize(nelem);
        	std::copy(pack_loss_arr.begin(), pack_loss_arr.begin()+nelem, msg->pack_loss.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'pack_loss' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'pack_loss' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_pi3hat_moteus_int_msgs_msg_PacketPass_common::get_arr(MDFactory_T& factory, const pi3hat_moteus_int_msgs::msg::PacketPass* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","valid","cycle_dur","name","pack_loss"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("pi3hat_moteus_int_msgs/PacketPass");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // valid
    auto currentElement_valid = (msg + ctr)->valid;
    outArray[ctr]["valid"] = factory.createScalar(currentElement_valid);
    // cycle_dur
    auto currentElement_cycle_dur = (msg + ctr)->cycle_dur;
    outArray[ctr]["cycle_dur"] = factory.createScalar(currentElement_cycle_dur);
    // name
    auto currentElement_name = (msg + ctr)->name;
    auto nameoutCell = factory.createCellArray({currentElement_name.size(),1});
    for(size_t idxin = 0; idxin < currentElement_name.size(); ++ idxin){
    	nameoutCell[idxin] = factory.createCharArray(currentElement_name[idxin]);
    }
    outArray[ctr]["name"] = nameoutCell;
    // pack_loss
    auto currentElement_pack_loss = (msg + ctr)->pack_loss;
    outArray[ctr]["pack_loss"] = factory.createArray<pi3hat_moteus_int_msgs::msg::PacketPass::_pack_loss_type::const_iterator, double>({currentElement_pack_loss.size(), 1}, currentElement_pack_loss.begin(), currentElement_pack_loss.end());
    }
    return std::move(outArray);
  } 
class PI3HAT_MOTEUS_INT_MSGS_EXPORT ros2_pi3hat_moteus_int_msgs_PacketPass_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_pi3hat_moteus_int_msgs_PacketPass_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_pi3hat_moteus_int_msgs_PacketPass_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<pi3hat_moteus_int_msgs::msg::PacketPass,ros2_pi3hat_moteus_int_msgs_msg_PacketPass_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_pi3hat_moteus_int_msgs_PacketPass_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<pi3hat_moteus_int_msgs::msg::PacketPass,ros2_pi3hat_moteus_int_msgs_msg_PacketPass_common>>();
  }
  std::shared_ptr<void> ros2_pi3hat_moteus_int_msgs_PacketPass_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<pi3hat_moteus_int_msgs::msg::PacketPass>();
    ros2_pi3hat_moteus_int_msgs_msg_PacketPass_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_pi3hat_moteus_int_msgs_PacketPass_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_pi3hat_moteus_int_msgs_msg_PacketPass_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (pi3hat_moteus_int_msgs::msg::PacketPass*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_pi3hat_moteus_int_msgs_msg_PacketPass_common, MATLABROS2MsgInterface<pi3hat_moteus_int_msgs::msg::PacketPass>)
CLASS_LOADER_REGISTER_CLASS(ros2_pi3hat_moteus_int_msgs_PacketPass_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER