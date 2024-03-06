function ros2msg = pronto_msgs_msg_VisualOdometryUpdate_1To2_Converter(message,ros2msg)
%pronto_msgs_msg_VisualOdometryUpdate_1To2_Converter passes data of ROS message to ROS 2 message.
% Copyright 2019 The MathWorks, Inc.
ros2msg.header.stamp.sec = message.Header.Stamp.Sec;
ros2msg.header.stamp.nanosec = message.Header.Stamp.Nsec;
ros2msg.header.frame_id = message.Header.FrameId;
ros2msg.curr_timestamp.sec = message.CurrTimestamp.Sec;
ros2msg.curr_timestamp.nanosec = message.CurrTimestamp.Nsec;
ros2msg.prev_timestamp.sec = message.PrevTimestamp.Sec;
ros2msg.prev_timestamp.nanosec = message.PrevTimestamp.Nsec;
ros2msg.relative_transform.translation.x = message.RelativeTransform.Translation.X;
ros2msg.relative_transform.translation.y = message.RelativeTransform.Translation.Y;
ros2msg.relative_transform.translation.z = message.RelativeTransform.Translation.Z;
ros2msg.relative_transform.rotation.x = message.RelativeTransform.Rotation.X;
ros2msg.relative_transform.rotation.y = message.RelativeTransform.Rotation.Y;
ros2msg.relative_transform.rotation.z = message.RelativeTransform.Rotation.Z;
ros2msg.relative_transform.rotation.w = message.RelativeTransform.Rotation.W;
ros2msg.covariance = message.Covariance;
ros2msg.estimate_status = message.EstimateStatus;
end