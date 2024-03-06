function ros2msg = pronto_msgs_msg_QuadrupedForceTorqueSensors_1To2_Converter(message,ros2msg)
%pronto_msgs_msg_QuadrupedForceTorqueSensors_1To2_Converter passes data of ROS message to ROS 2 message.
% Copyright 2019 The MathWorks, Inc.
ros2msg.header.stamp.sec = message.Header.Stamp.Sec;
ros2msg.header.stamp.nanosec = message.Header.Stamp.Nsec;
ros2msg.header.frame_id = message.Header.FrameId;
ros2msg.lf.force.x = message.Lf.Force.X;
ros2msg.lf.force.y = message.Lf.Force.Y;
ros2msg.lf.force.z = message.Lf.Force.Z;
ros2msg.lf.torque.x = message.Lf.Torque.X;
ros2msg.lf.torque.y = message.Lf.Torque.Y;
ros2msg.lf.torque.z = message.Lf.Torque.Z;
ros2msg.rf.force.x = message.Rf.Force.X;
ros2msg.rf.force.y = message.Rf.Force.Y;
ros2msg.rf.force.z = message.Rf.Force.Z;
ros2msg.rf.torque.x = message.Rf.Torque.X;
ros2msg.rf.torque.y = message.Rf.Torque.Y;
ros2msg.rf.torque.z = message.Rf.Torque.Z;
ros2msg.lh.force.x = message.Lh.Force.X;
ros2msg.lh.force.y = message.Lh.Force.Y;
ros2msg.lh.force.z = message.Lh.Force.Z;
ros2msg.lh.torque.x = message.Lh.Torque.X;
ros2msg.lh.torque.y = message.Lh.Torque.Y;
ros2msg.lh.torque.z = message.Lh.Torque.Z;
ros2msg.rh.force.x = message.Rh.Force.X;
ros2msg.rh.force.y = message.Rh.Force.Y;
ros2msg.rh.force.z = message.Rh.Force.Z;
ros2msg.rh.torque.x = message.Rh.Torque.X;
ros2msg.rh.torque.y = message.Rh.Torque.Y;
ros2msg.rh.torque.z = message.Rh.Torque.Z;
end