function ros2msg = pronto_msgs_msg_VelocityWithSigmaBounds_1To2_Converter(message,ros2msg)
%pronto_msgs_msg_VelocityWithSigmaBounds_1To2_Converter passes data of ROS message to ROS 2 message.
% Copyright 2019 The MathWorks, Inc.
ros2msg.header.stamp.sec = message.Header.Stamp.Sec;
ros2msg.header.stamp.nanosec = message.Header.Stamp.Nsec;
ros2msg.header.frame_id = message.Header.FrameId;
ros2msg.velocity_plus_one_sigma.x = message.VelocityPlusOneSigma.X;
ros2msg.velocity_plus_one_sigma.y = message.VelocityPlusOneSigma.Y;
ros2msg.velocity_plus_one_sigma.z = message.VelocityPlusOneSigma.Z;
ros2msg.velocity_minus_one_sigma.x = message.VelocityMinusOneSigma.X;
ros2msg.velocity_minus_one_sigma.y = message.VelocityMinusOneSigma.Y;
ros2msg.velocity_minus_one_sigma.z = message.VelocityMinusOneSigma.Z;
ros2msg.plus_one_sigma.x = message.PlusOneSigma.X;
ros2msg.plus_one_sigma.y = message.PlusOneSigma.Y;
ros2msg.plus_one_sigma.z = message.PlusOneSigma.Z;
end