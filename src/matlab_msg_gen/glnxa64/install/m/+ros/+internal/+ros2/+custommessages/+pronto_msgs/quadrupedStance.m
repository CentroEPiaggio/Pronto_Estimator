function [data, info] = quadrupedStance
%QuadrupedStance gives an empty data for pronto_msgs/QuadrupedStance
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'pronto_msgs/QuadrupedStance';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.lf, info.lf] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.rf, info.rf] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.lh, info.lh] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.rh, info.rh] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
info.MessageType = 'pronto_msgs/QuadrupedStance';
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
info.MatPath{6} = 'lf';
info.MatPath{7} = 'rf';
info.MatPath{8} = 'lh';
info.MatPath{9} = 'rh';
