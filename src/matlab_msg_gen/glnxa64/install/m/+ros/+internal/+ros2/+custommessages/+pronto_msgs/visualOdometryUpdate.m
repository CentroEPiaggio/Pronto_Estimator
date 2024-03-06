function [data, info] = visualOdometryUpdate
%VisualOdometryUpdate gives an empty data for pronto_msgs/VisualOdometryUpdate
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'pronto_msgs/VisualOdometryUpdate';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.curr_timestamp, info.curr_timestamp] = ros.internal.ros2.messages.builtin_interfaces.time;
info.curr_timestamp.MLdataType = 'struct';
[data.prev_timestamp, info.prev_timestamp] = ros.internal.ros2.messages.builtin_interfaces.time;
info.prev_timestamp.MLdataType = 'struct';
[data.relative_transform, info.relative_transform] = ros.internal.ros2.messages.geometry_msgs.transform;
info.relative_transform.MLdataType = 'struct';
[data.covariance, info.covariance] = ros.internal.ros2.messages.ros2.default_type('double',36,0);
[data.estimate_status, info.estimate_status] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.NO_DATA, info.NO_DATA] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 0, [NaN]);
[data.ESTIMATE_VALID, info.ESTIMATE_VALID] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 1, [NaN]);
[data.ESTIMATE_INSUFFICIENT_FEATURES, info.ESTIMATE_INSUFFICIENT_FEATURES] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 2, [NaN]);
[data.ESTIMATE_DEGENERATE, info.ESTIMATE_DEGENERATE] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 3, [NaN]);
[data.ESTIMATE_REPROJECTION_ERROR, info.ESTIMATE_REPROJECTION_ERROR] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 4, [NaN]);
info.MessageType = 'pronto_msgs/VisualOdometryUpdate';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,28);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'curr_timestamp';
info.MatPath{7} = 'curr_timestamp.sec';
info.MatPath{8} = 'curr_timestamp.nanosec';
info.MatPath{9} = 'prev_timestamp';
info.MatPath{10} = 'prev_timestamp.sec';
info.MatPath{11} = 'prev_timestamp.nanosec';
info.MatPath{12} = 'relative_transform';
info.MatPath{13} = 'relative_transform.translation';
info.MatPath{14} = 'relative_transform.translation.x';
info.MatPath{15} = 'relative_transform.translation.y';
info.MatPath{16} = 'relative_transform.translation.z';
info.MatPath{17} = 'relative_transform.rotation';
info.MatPath{18} = 'relative_transform.rotation.x';
info.MatPath{19} = 'relative_transform.rotation.y';
info.MatPath{20} = 'relative_transform.rotation.z';
info.MatPath{21} = 'relative_transform.rotation.w';
info.MatPath{22} = 'covariance';
info.MatPath{23} = 'estimate_status';
info.MatPath{24} = 'NO_DATA';
info.MatPath{25} = 'ESTIMATE_VALID';
info.MatPath{26} = 'ESTIMATE_INSUFFICIENT_FEATURES';
info.MatPath{27} = 'ESTIMATE_DEGENERATE';
info.MatPath{28} = 'ESTIMATE_REPROJECTION_ERROR';
