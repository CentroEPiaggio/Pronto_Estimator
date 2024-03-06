function [data, info] = controllerFootContact
%ControllerFootContact gives an empty data for pronto_msgs/ControllerFootContact
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'pronto_msgs/ControllerFootContact';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.num_right_foot_contacts, info.num_right_foot_contacts] = ros.internal.ros2.messages.ros2.default_type('int32',1,0);
[data.right_foot_contacts, info.right_foot_contacts] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
[data.num_left_foot_contacts, info.num_left_foot_contacts] = ros.internal.ros2.messages.ros2.default_type('int32',1,0);
[data.left_foot_contacts, info.left_foot_contacts] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
info.MessageType = 'pronto_msgs/ControllerFootContact';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'num_right_foot_contacts';
info.MatPath{7} = 'right_foot_contacts';
info.MatPath{8} = 'num_left_foot_contacts';
info.MatPath{9} = 'left_foot_contacts';
