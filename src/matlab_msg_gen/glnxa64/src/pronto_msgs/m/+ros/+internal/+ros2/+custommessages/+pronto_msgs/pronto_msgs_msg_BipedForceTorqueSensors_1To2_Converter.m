function ros2msg = pronto_msgs_msg_BipedForceTorqueSensors_1To2_Converter(message,ros2msg)
%pronto_msgs_msg_BipedForceTorqueSensors_1To2_Converter passes data of ROS message to ROS 2 message.
% Copyright 2019 The MathWorks, Inc.
ros2msg.header.stamp.sec = message.Header.Stamp.Sec;
ros2msg.header.stamp.nanosec = message.Header.Stamp.Nsec;
ros2msg.header.frame_id = message.Header.FrameId;
ros2msg.l_foot.force.x = message.LFoot.Force.X;
ros2msg.l_foot.force.y = message.LFoot.Force.Y;
ros2msg.l_foot.force.z = message.LFoot.Force.Z;
ros2msg.l_foot.torque.x = message.LFoot.Torque.X;
ros2msg.l_foot.torque.y = message.LFoot.Torque.Y;
ros2msg.l_foot.torque.z = message.LFoot.Torque.Z;
ros2msg.r_foot.force.x = message.RFoot.Force.X;
ros2msg.r_foot.force.y = message.RFoot.Force.Y;
ros2msg.r_foot.force.z = message.RFoot.Force.Z;
ros2msg.r_foot.torque.x = message.RFoot.Torque.X;
ros2msg.r_foot.torque.y = message.RFoot.Torque.Y;
ros2msg.r_foot.torque.z = message.RFoot.Torque.Z;
ros2msg.l_hand.force.x = message.LHand.Force.X;
ros2msg.l_hand.force.y = message.LHand.Force.Y;
ros2msg.l_hand.force.z = message.LHand.Force.Z;
ros2msg.l_hand.torque.x = message.LHand.Torque.X;
ros2msg.l_hand.torque.y = message.LHand.Torque.Y;
ros2msg.l_hand.torque.z = message.LHand.Torque.Z;
ros2msg.r_hand.force.x = message.RHand.Force.X;
ros2msg.r_hand.force.y = message.RHand.Force.Y;
ros2msg.r_hand.force.z = message.RHand.Force.Z;
ros2msg.r_hand.torque.x = message.RHand.Torque.X;
ros2msg.r_hand.torque.y = message.RHand.Torque.Y;
ros2msg.r_hand.torque.z = message.RHand.Torque.Z;
end