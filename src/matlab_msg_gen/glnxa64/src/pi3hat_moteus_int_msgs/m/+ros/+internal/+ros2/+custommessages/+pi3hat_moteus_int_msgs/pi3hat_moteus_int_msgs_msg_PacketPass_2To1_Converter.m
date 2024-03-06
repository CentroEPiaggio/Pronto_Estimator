function ros1msg = pi3hat_moteus_int_msgs_msg_PacketPass_2To1_Converter(message,ros1msg)
%pi3hat_moteus_int_msgs_msg_PacketPass_2To1_Converter passes data of ROS 2 message to ROS message.
% Copyright 2019 The MathWorks, Inc.    
ros1msg.Header.Stamp.Sec = message.header.stamp.sec;
ros1msg.Header.Stamp.Nsec = message.header.stamp.nanosec;
ros1msg.Header.FrameId = message.header.frame_id{1};
ros1msg.Valid = message.valid;
ros1msg.CycleDur = message.cycle_dur;
ros1msg.Name = message.name{1};
end