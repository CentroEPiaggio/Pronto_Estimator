function [data, info] = bipedForceTorqueSensors
%BipedForceTorqueSensors gives an empty data for pronto_msgs/BipedForceTorqueSensors
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'pronto_msgs/BipedForceTorqueSensors';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.l_foot, info.l_foot] = ros.internal.ros2.messages.geometry_msgs.wrench;
info.l_foot.MLdataType = 'struct';
[data.r_foot, info.r_foot] = ros.internal.ros2.messages.geometry_msgs.wrench;
info.r_foot.MLdataType = 'struct';
[data.l_hand, info.l_hand] = ros.internal.ros2.messages.geometry_msgs.wrench;
info.l_hand.MLdataType = 'struct';
[data.r_hand, info.r_hand] = ros.internal.ros2.messages.geometry_msgs.wrench;
info.r_hand.MLdataType = 'struct';
info.MessageType = 'pronto_msgs/BipedForceTorqueSensors';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,41);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'l_foot';
info.MatPath{7} = 'l_foot.force';
info.MatPath{8} = 'l_foot.force.x';
info.MatPath{9} = 'l_foot.force.y';
info.MatPath{10} = 'l_foot.force.z';
info.MatPath{11} = 'l_foot.torque';
info.MatPath{12} = 'l_foot.torque.x';
info.MatPath{13} = 'l_foot.torque.y';
info.MatPath{14} = 'l_foot.torque.z';
info.MatPath{15} = 'r_foot';
info.MatPath{16} = 'r_foot.force';
info.MatPath{17} = 'r_foot.force.x';
info.MatPath{18} = 'r_foot.force.y';
info.MatPath{19} = 'r_foot.force.z';
info.MatPath{20} = 'r_foot.torque';
info.MatPath{21} = 'r_foot.torque.x';
info.MatPath{22} = 'r_foot.torque.y';
info.MatPath{23} = 'r_foot.torque.z';
info.MatPath{24} = 'l_hand';
info.MatPath{25} = 'l_hand.force';
info.MatPath{26} = 'l_hand.force.x';
info.MatPath{27} = 'l_hand.force.y';
info.MatPath{28} = 'l_hand.force.z';
info.MatPath{29} = 'l_hand.torque';
info.MatPath{30} = 'l_hand.torque.x';
info.MatPath{31} = 'l_hand.torque.y';
info.MatPath{32} = 'l_hand.torque.z';
info.MatPath{33} = 'r_hand';
info.MatPath{34} = 'r_hand.force';
info.MatPath{35} = 'r_hand.force.x';
info.MatPath{36} = 'r_hand.force.y';
info.MatPath{37} = 'r_hand.force.z';
info.MatPath{38} = 'r_hand.torque';
info.MatPath{39} = 'r_hand.torque.x';
info.MatPath{40} = 'r_hand.torque.y';
info.MatPath{41} = 'r_hand.torque.z';