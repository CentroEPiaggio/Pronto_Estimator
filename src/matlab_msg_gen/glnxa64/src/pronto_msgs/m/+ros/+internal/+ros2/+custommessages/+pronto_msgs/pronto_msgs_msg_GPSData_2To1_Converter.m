function ros1msg = pronto_msgs_msg_GPSData_2To1_Converter(message,ros1msg)
%pronto_msgs_msg_GPSData_2To1_Converter passes data of ROS 2 message to ROS message.
% Copyright 2019 The MathWorks, Inc.    
ros1msg.Header.Stamp.Sec = message.header.stamp.sec;
ros1msg.Header.Stamp.Nsec = message.header.stamp.nanosec;
ros1msg.Header.FrameId = message.header.frame_id{1};
ros1msg.Utime = message.utime;
ros1msg.GpsLock = message.gps_lock;
ros1msg.Longitude = message.longitude;
ros1msg.Latitude = message.latitude;
ros1msg.Elev = message.elev;
ros1msg.HorizontalAccuracy = message.horizontal_accuracy;
ros1msg.VerticalAccuracy = message.vertical_accuracy;
ros1msg.NumSatellites = message.num_satellites;
ros1msg.Speed = message.speed;
ros1msg.Heading = message.heading;
ros1msg.XyzPos = message.xyz_pos;
ros1msg.GpsTime = message.gps_time;
end