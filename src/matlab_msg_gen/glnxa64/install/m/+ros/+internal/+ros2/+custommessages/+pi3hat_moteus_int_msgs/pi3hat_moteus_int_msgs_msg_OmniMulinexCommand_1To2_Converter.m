function ros2msg = pi3hat_moteus_int_msgs_msg_OmniMulinexCommand_1To2_Converter(message,ros2msg)
%pi3hat_moteus_int_msgs_msg_OmniMulinexCommand_1To2_Converter passes data of ROS message to ROS 2 message.
% Copyright 2019 The MathWorks, Inc.
ros2msg.header.stamp.sec = message.Header.Stamp.Sec;
ros2msg.header.stamp.nanosec = message.Header.Stamp.Nsec;
ros2msg.header.frame_id = message.Header.FrameId;
ros2msg.v_x = message.VX;
ros2msg.v_y = message.VY;
ros2msg.omega = message.Omega;
ros2msg.height_rate = message.HeightRate;
end