function ros1msg = pronto_msgs_msg_LidarOdometryUpdate_2To1_Converter(message,ros1msg)
%pronto_msgs_msg_LidarOdometryUpdate_2To1_Converter passes data of ROS 2 message to ROS message.
% Copyright 2019 The MathWorks, Inc.    
ros1msg.Header.Stamp.Sec = message.header.stamp.sec;
ros1msg.Header.Stamp.Nsec = message.header.stamp.nanosec;
ros1msg.Header.FrameId = message.header.frame_id{1};
ros1msg.CurrTimestamp.Sec = message.curr_timestamp.sec;
ros1msg.CurrTimestamp.Nsec = message.curr_timestamp.nanosec;
ros1msg.PrevTimestamp.Sec = message.prev_timestamp.sec;
ros1msg.PrevTimestamp.Nsec = message.prev_timestamp.nanosec;
ros1msg.RelativeTransform.Translation.X = message.relative_transform.translation.x;
ros1msg.RelativeTransform.Translation.Y = message.relative_transform.translation.y;
ros1msg.RelativeTransform.Translation.Z = message.relative_transform.translation.z;
ros1msg.RelativeTransform.Rotation.X = message.relative_transform.rotation.x;
ros1msg.RelativeTransform.Rotation.Y = message.relative_transform.rotation.y;
ros1msg.RelativeTransform.Rotation.Z = message.relative_transform.rotation.z;
ros1msg.RelativeTransform.Rotation.W = message.relative_transform.rotation.w;
ros1msg.Covariance = message.covariance;
end