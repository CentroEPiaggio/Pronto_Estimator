function ros1msg = pi3hat_moteus_int_msgs_msg_OmniMulinexCommand_2To1_Converter(message,ros1msg)
%pi3hat_moteus_int_msgs_msg_OmniMulinexCommand_2To1_Converter passes data of ROS 2 message to ROS message.
% Copyright 2019 The MathWorks, Inc.    
ros1msg.Header.Stamp.Sec = message.header.stamp.sec;
ros1msg.Header.Stamp.Nsec = message.header.stamp.nanosec;
ros1msg.Header.FrameId = message.header.frame_id{1};
ros1msg.VX = message.v_x;
ros1msg.VY = message.v_y;
ros1msg.Omega = message.omega;
ros1msg.HeightRate = message.height_rate;
end