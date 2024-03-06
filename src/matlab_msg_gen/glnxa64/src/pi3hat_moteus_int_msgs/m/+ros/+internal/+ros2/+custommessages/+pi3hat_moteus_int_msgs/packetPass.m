function [data, info] = packetPass
%PacketPass gives an empty data for pi3hat_moteus_int_msgs/PacketPass
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'pi3hat_moteus_int_msgs/PacketPass';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.valid, info.valid] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.cycle_dur, info.cycle_dur] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.name, info.name] = ros.internal.ros2.messages.ros2.char('string',NaN,NaN,0);
[data.pack_loss, info.pack_loss] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
info.MessageType = 'pi3hat_moteus_int_msgs/PacketPass';
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
info.MatPath{6} = 'valid';
info.MatPath{7} = 'cycle_dur';
info.MatPath{8} = 'name';
info.MatPath{9} = 'pack_loss';
