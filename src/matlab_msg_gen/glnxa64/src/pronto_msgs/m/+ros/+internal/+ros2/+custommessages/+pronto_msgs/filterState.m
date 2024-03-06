function [data, info] = filterState
%FilterState gives an empty data for pronto_msgs/FilterState
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'pronto_msgs/FilterState';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.quat, info.quat] = ros.internal.ros2.messages.geometry_msgs.quaternion;
info.quat.MLdataType = 'struct';
[data.state, info.state] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
[data.cov, info.cov] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
info.MessageType = 'pronto_msgs/FilterState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'quat';
info.MatPath{7} = 'quat.x';
info.MatPath{8} = 'quat.y';
info.MatPath{9} = 'quat.z';
info.MatPath{10} = 'quat.w';
info.MatPath{11} = 'state';
info.MatPath{12} = 'cov';