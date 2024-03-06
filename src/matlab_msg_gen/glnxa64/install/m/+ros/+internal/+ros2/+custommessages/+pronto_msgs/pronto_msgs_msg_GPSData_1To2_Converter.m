function ros2msg = pronto_msgs_msg_GPSData_1To2_Converter(message,ros2msg)
%pronto_msgs_msg_GPSData_1To2_Converter passes data of ROS message to ROS 2 message.
% Copyright 2019 The MathWorks, Inc.
ros2msg.header.stamp.sec = message.Header.Stamp.Sec;
ros2msg.header.stamp.nanosec = message.Header.Stamp.Nsec;
ros2msg.header.frame_id = message.Header.FrameId;
ros2msg.utime = message.Utime;
ros2msg.gps_lock = message.GpsLock;
ros2msg.longitude = message.Longitude;
ros2msg.latitude = message.Latitude;
ros2msg.elev = message.Elev;
ros2msg.horizontal_accuracy = message.HorizontalAccuracy;
ros2msg.vertical_accuracy = message.VerticalAccuracy;
ros2msg.num_satellites = message.NumSatellites;
ros2msg.speed = message.Speed;
ros2msg.heading = message.Heading;
ros2msg.xyz_pos = message.XyzPos;
ros2msg.gps_time = message.GpsTime;
end