function [data, info] = omniMulinexCommand
%OmniMulinexCommand gives an empty data for pi3hat_moteus_int_msgs/OmniMulinexCommand
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'pi3hat_moteus_int_msgs/OmniMulinexCommand';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.v_x, info.v_x] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.v_y, info.v_y] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.omega, info.omega] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.height_rate, info.height_rate] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
info.MessageType = 'pi3hat_moteus_int_msgs/OmniMulinexCommand';
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
info.MatPath{6} = 'v_x';
info.MatPath{7} = 'v_y';
info.MatPath{8} = 'omega';
info.MatPath{9} = 'height_rate';
