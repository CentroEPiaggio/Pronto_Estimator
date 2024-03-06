function ros1msg = pronto_msgs_msg_BipedForceTorqueSensors_2To1_Converter(message,ros1msg)
%pronto_msgs_msg_BipedForceTorqueSensors_2To1_Converter passes data of ROS 2 message to ROS message.
% Copyright 2019 The MathWorks, Inc.    
ros1msg.Header.Stamp.Sec = message.header.stamp.sec;
ros1msg.Header.Stamp.Nsec = message.header.stamp.nanosec;
ros1msg.Header.FrameId = message.header.frame_id{1};
ros1msg.LFoot.Force.X = message.l_foot.force.x;
ros1msg.LFoot.Force.Y = message.l_foot.force.y;
ros1msg.LFoot.Force.Z = message.l_foot.force.z;
ros1msg.LFoot.Torque.X = message.l_foot.torque.x;
ros1msg.LFoot.Torque.Y = message.l_foot.torque.y;
ros1msg.LFoot.Torque.Z = message.l_foot.torque.z;
ros1msg.RFoot.Force.X = message.r_foot.force.x;
ros1msg.RFoot.Force.Y = message.r_foot.force.y;
ros1msg.RFoot.Force.Z = message.r_foot.force.z;
ros1msg.RFoot.Torque.X = message.r_foot.torque.x;
ros1msg.RFoot.Torque.Y = message.r_foot.torque.y;
ros1msg.RFoot.Torque.Z = message.r_foot.torque.z;
ros1msg.LHand.Force.X = message.l_hand.force.x;
ros1msg.LHand.Force.Y = message.l_hand.force.y;
ros1msg.LHand.Force.Z = message.l_hand.force.z;
ros1msg.LHand.Torque.X = message.l_hand.torque.x;
ros1msg.LHand.Torque.Y = message.l_hand.torque.y;
ros1msg.LHand.Torque.Z = message.l_hand.torque.z;
ros1msg.RHand.Force.X = message.r_hand.force.x;
ros1msg.RHand.Force.Y = message.r_hand.force.y;
ros1msg.RHand.Force.Z = message.r_hand.force.z;
ros1msg.RHand.Torque.X = message.r_hand.torque.x;
ros1msg.RHand.Torque.Y = message.r_hand.torque.y;
ros1msg.RHand.Torque.Z = message.r_hand.torque.z;
end