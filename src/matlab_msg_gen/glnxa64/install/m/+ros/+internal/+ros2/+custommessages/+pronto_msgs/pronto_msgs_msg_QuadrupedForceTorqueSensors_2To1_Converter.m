function ros1msg = pronto_msgs_msg_QuadrupedForceTorqueSensors_2To1_Converter(message,ros1msg)
%pronto_msgs_msg_QuadrupedForceTorqueSensors_2To1_Converter passes data of ROS 2 message to ROS message.
% Copyright 2019 The MathWorks, Inc.    
ros1msg.Header.Stamp.Sec = message.header.stamp.sec;
ros1msg.Header.Stamp.Nsec = message.header.stamp.nanosec;
ros1msg.Header.FrameId = message.header.frame_id{1};
ros1msg.Lf.Force.X = message.lf.force.x;
ros1msg.Lf.Force.Y = message.lf.force.y;
ros1msg.Lf.Force.Z = message.lf.force.z;
ros1msg.Lf.Torque.X = message.lf.torque.x;
ros1msg.Lf.Torque.Y = message.lf.torque.y;
ros1msg.Lf.Torque.Z = message.lf.torque.z;
ros1msg.Rf.Force.X = message.rf.force.x;
ros1msg.Rf.Force.Y = message.rf.force.y;
ros1msg.Rf.Force.Z = message.rf.force.z;
ros1msg.Rf.Torque.X = message.rf.torque.x;
ros1msg.Rf.Torque.Y = message.rf.torque.y;
ros1msg.Rf.Torque.Z = message.rf.torque.z;
ros1msg.Lh.Force.X = message.lh.force.x;
ros1msg.Lh.Force.Y = message.lh.force.y;
ros1msg.Lh.Force.Z = message.lh.force.z;
ros1msg.Lh.Torque.X = message.lh.torque.x;
ros1msg.Lh.Torque.Y = message.lh.torque.y;
ros1msg.Lh.Torque.Z = message.lh.torque.z;
ros1msg.Rh.Force.X = message.rh.force.x;
ros1msg.Rh.Force.Y = message.rh.force.y;
ros1msg.Rh.Force.Z = message.rh.force.z;
ros1msg.Rh.Torque.X = message.rh.torque.x;
ros1msg.Rh.Torque.Y = message.rh.torque.y;
ros1msg.Rh.Torque.Z = message.rh.torque.z;
end