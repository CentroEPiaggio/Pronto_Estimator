function ros2msg = pi3hat_moteus_int_msgs_msg_PacketPass_1To2_Converter(message,ros2msg)
%pi3hat_moteus_int_msgs_msg_PacketPass_1To2_Converter passes data of ROS message to ROS 2 message.
% Copyright 2019 The MathWorks, Inc.
ros2msg.header.stamp.sec = message.Header.Stamp.Sec;
ros2msg.header.stamp.nanosec = message.Header.Stamp.Nsec;
ros2msg.header.frame_id = message.Header.FrameId;
ros2msg.valid = message.Valid;
ros2msg.cycle_dur = message.CycleDur;
ros2msg.name = message.Name;
end