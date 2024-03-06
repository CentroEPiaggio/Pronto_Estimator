function ros1msg = pronto_msgs_msg_VelocityWithSigmaBounds_2To1_Converter(message,ros1msg)
%pronto_msgs_msg_VelocityWithSigmaBounds_2To1_Converter passes data of ROS 2 message to ROS message.
% Copyright 2019 The MathWorks, Inc.    
ros1msg.Header.Stamp.Sec = message.header.stamp.sec;
ros1msg.Header.Stamp.Nsec = message.header.stamp.nanosec;
ros1msg.Header.FrameId = message.header.frame_id{1};
ros1msg.VelocityPlusOneSigma.X = message.velocity_plus_one_sigma.x;
ros1msg.VelocityPlusOneSigma.Y = message.velocity_plus_one_sigma.y;
ros1msg.VelocityPlusOneSigma.Z = message.velocity_plus_one_sigma.z;
ros1msg.VelocityMinusOneSigma.X = message.velocity_minus_one_sigma.x;
ros1msg.VelocityMinusOneSigma.Y = message.velocity_minus_one_sigma.y;
ros1msg.VelocityMinusOneSigma.Z = message.velocity_minus_one_sigma.z;
ros1msg.PlusOneSigma.X = message.plus_one_sigma.x;
ros1msg.PlusOneSigma.Y = message.plus_one_sigma.y;
ros1msg.PlusOneSigma.Z = message.plus_one_sigma.z;
end