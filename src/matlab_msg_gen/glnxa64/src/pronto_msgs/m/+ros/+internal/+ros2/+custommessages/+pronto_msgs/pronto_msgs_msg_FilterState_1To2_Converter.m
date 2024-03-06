function ros2msg = pronto_msgs_msg_FilterState_1To2_Converter(message,ros2msg)
%pronto_msgs_msg_FilterState_1To2_Converter passes data of ROS message to ROS 2 message.
% Copyright 2019 The MathWorks, Inc.
ros2msg.header.stamp.sec = message.Header.Stamp.Sec;
ros2msg.header.stamp.nanosec = message.Header.Stamp.Nsec;
ros2msg.header.frame_id = message.Header.FrameId;
ros2msg.quat.x = message.Quat.X;
ros2msg.quat.y = message.Quat.Y;
ros2msg.quat.z = message.Quat.Z;
ros2msg.quat.w = message.Quat.W;
end