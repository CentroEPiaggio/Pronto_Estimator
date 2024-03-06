%% build ws with custom message
ros2genmsg("/home/jacopo/Documents/mulinex_ws/src/");
close all 
clear
clc
%%
% varNames = {"Experiemnt Name", " q_gyro", "q_acc","r_vx","r_vy","r_vz","vx_err_mean","vx_err_std","vx_max","vy_err_mean","vy_err_std","vy_max","vz_err_mean","vz_err_std","vz_max"} ;
% varTypes = {'string','double','double','double','double','double','double','double','double','double','double','double','double','double','double'} ;
% delimiter = ',';
% dataStartLine = 2;
% extraColRule = 'ignore';
% opts = spreadsheetImportOptions
%  % {"Experiemnt Name", " q_gyro", "q_acc","r_vx","r_vy","r_vz","vx_err_mean","vx_err_std","vx_max","vy_err_mean","vy_err_std","vy_max","vz_err_mean","vz_err_std","vz_max"}); 
% opts.VariableNames = {"Experiemnt Name"}
%% set up the folder and basg name 
bag_folder = "Anymal_c/Exp_4/";
bag_name = "straight_mot_mv_015_2024_2_19_14_46_25";
est_t_msg_type = "geometry_msgs/TwistWithCovarianceStamped";
est_p_msg_type = "geometry_msgs/PoseWithCovarianceStamped";
gt_t_msg_type = "geometry_msgs/TwistStamped";
gt_p_msg_type = "geometry_msgs/PoseStamped";
est_stance_type = "pronto_msgs/QuadrupedStance";
est_force_type = "pronto_msgs/QuadrupedForceTorqueSensors";
odom_cor_type = "geometry_msgs/Vector3Stamped";

bag = ros2bagreader(bag_folder + bag_name);


est_twist_msg = select(bag,"MessageType",est_t_msg_type);

gt_twist_msg = select(bag,"MessageType",gt_t_msg_type);

gt_nm = gt_twist_msg.NumMessages;
est_nm = est_twist_msg.NumMessages;

gt_lin_vel = zeros(3,gt_twist_msg.NumMessages);

gt_ang_vel = zeros(3,gt_twist_msg.NumMessages);



gt_time = zeros(1,gt_twist_msg.NumMessages);

est_lin_vel = zeros(3,est_twist_msg.NumMessages);

est_ang_vel = zeros(3,est_twist_msg.NumMessages);

est_time = zeros(1,est_twist_msg.NumMessages);


est_twist_msg = readMessages(est_twist_msg);
gt_twist_msg = readMessages(gt_twist_msg);



q_acc = 0.1;
q_gyro = 0.01;
r_vx = 0.0000005;
r_vy = 0.0000005;
r_vz = 0.0001;
%%
gt_pose_bg = select(bag,"MessageType",gt_p_msg_type);
gt_p_num = gt_pose_bg.NumMessages;

est_pose_bg = select(bag,"MessageType",est_p_msg_type);
est_p_num = est_pose_bg.NumMessages;

gt_pos = zeros(3,gt_p_num);
gt_p_time = zeros(1,gt_p_num);
gt_ori = zeros(4,gt_p_num);
est_ori_r = zeros(4,gt_p_num);
est_pos = zeros(3,est_p_num);
est_p_time = zeros(1,est_p_num);
est_ori = zeros(4,est_p_num);

gt_pose_msg = readMessages(gt_pose_bg);
est_pose_msg = readMessages(est_pose_bg);

ori_err = zeros(3,gt_nm);

%%
est_stance_bag = select(bag,"MessageType",est_stance_type);
est_stance_msg = readMessages(est_stance_bag);

est_force_bag = select(bag,"MessageType",est_force_type);
est_force_msg = readMessages(est_force_bag);
% 
est_force_num = est_force_bag.NumMessages;
est_force_time = zeros(1,est_force_num);
est_force_val = zeros(4,est_force_num);

est_stance_num = est_stance_bag.NumMessages;
est_stance_time = zeros(1,est_stance_num);
est_stance_val = zeros(4,est_stance_num);

%% 
est_odom_cor_bag = select(bag,"MessageType",odom_cor_type);
est_odom_cor_msg = readMessages(est_odom_cor_bag);

est_odom_cor_num = est_odom_cor_bag.NumMessages;

est_odom_cor_time = zeros(1,est_odom_cor_num);
est_odom_cor_values = zeros(3,est_odom_cor_num);


%% fill the matrixes

for i= 1:est_nm
    est_time(i) = ros_h_2_secs(est_twist_msg{i}.header.stamp);
    
    est_lin_vel(1,i) = est_twist_msg{i}.twist.twist.linear.x;
    est_ang_vel(1,i) = est_twist_msg{i}.twist.twist.angular.x;
    est_lin_vel(2,i) = est_twist_msg{i}.twist.twist.linear.y;
    est_ang_vel(2,i) = est_twist_msg{i}.twist.twist.angular.y;
    est_lin_vel(3,i) = est_twist_msg{i}.twist.twist.linear.z;
    est_ang_vel(3,i) = est_twist_msg{i}.twist.twist.angular.z;
end

for i = 1:gt_nm
    gt_time(i) = ros_h_2_secs(gt_twist_msg{i}.header.stamp);
    gt_lin_vel(1,i) = gt_twist_msg{i}.twist.linear.x;
    gt_ang_vel(1,i) = gt_twist_msg{i}.twist.angular.x;
    gt_lin_vel(2,i) = gt_twist_msg{i}.twist.linear.y;
    gt_ang_vel(2,i) = gt_twist_msg{i}.twist.angular.y;
    gt_lin_vel(3,i) = gt_twist_msg{i}.twist.linear.z;
    gt_ang_vel(3,i) = gt_twist_msg{i}.twist.angular.z;
end



% if(gt_time(1) > est_time(1))
%     offset = est_time(1);
% else
%     offset = gt_time(1);
% end
% 
% gt_time = gt_time - offset;
% est_time = est_time - offset;

%% 
for i = 1:gt_p_num
    gt_p_time(i) = ros_h_2_secs(gt_pose_msg{i}.header.stamp);
    gt_pos(1,i) = gt_pose_msg{i}.pose.position.x;
    gt_pos(2,i) = gt_pose_msg{i}.pose.position.y;
    gt_pos(3,i) = gt_pose_msg{i}.pose.position.z;
    gt_ori(1,i) = gt_pose_msg{i}.pose.orientation.w;
    gt_ori(2,i) = gt_pose_msg{i}.pose.orientation.x;
    gt_ori(3,i) = gt_pose_msg{i}.pose.orientation.y;
    gt_ori(4,i) = gt_pose_msg{i}.pose.orientation.z;
end

for i = 1:est_p_num
    est_p_time(i) = ros_h_2_secs(est_pose_msg{i}.header.stamp);
    est_pos(1,i) = est_pose_msg{i}.pose.pose.position.x;
    est_pos(2,i) = est_pose_msg{i}.pose.pose.position.y;
    est_pos(3,i) = est_pose_msg{i}.pose.pose.position.z;

    est_ori(1,i) = est_pose_msg{i}.pose.pose.orientation.w;
    est_ori(2,i) = est_pose_msg{i}.pose.pose.orientation.x;
    est_ori(3,i) = est_pose_msg{i}.pose.pose.orientation.y;
    est_ori(4,i) = est_pose_msg{i}.pose.pose.orientation.z;
end
% if(gt_p_time(1) > est_p_time(1))
%     offset = est_p_time(1);
% else
%     offset = gt_p_time(1);
% end
% 
% gt_p_time = gt_p_time - offset;
% est_p_time = est_p_time - offset;
%% 
for i = 1:est_odom_cor_num
    est_odom_cor_time(i) = ros_h_2_secs(est_odom_cor_msg{i}.header.stamp);
    
    est_odom_cor_values(1,i) = est_odom_cor_msg{i}.vector.x;
    est_odom_cor_values(2,i) = est_odom_cor_msg{i}.vector.y;
    est_odom_cor_values(3,i) = est_odom_cor_msg{i}.vector.z;
end
%%
for i = 1:est_stance_num
    est_stance_time(i) = ros_h_2_secs(est_stance_msg{i}.header.stamp);
    
    est_stance_val(1,i) = est_stance_msg{i}.lf;
    est_stance_val(2,i) = est_stance_msg{i}.lh;
    est_stance_val(3,i) = est_stance_msg{i}.rf;
    est_stance_val(4,i) = est_stance_msg{i}.rh;
end

for i = 1:est_force_num
    est_force_time(i) = ros_h_2_secs(est_force_msg{i}.header.stamp);
    % est_stance_val(1,1,i) = est_force_msg{i}.lf.force.x;
    % est_stance_val(1,2,i) = est_force_msg{i}.lf.force.y;
    est_force_val(1,i) = est_force_msg{i}.lf.force.z;
    % 
    % est_stance_val(2,1,i) = est_force_msg{i}.lh.force.x;
    % est_stance_val(2,2,i) = est_force_msg{i}.lh.force.y;
    est_force_val(2,i) = est_force_msg{i}.lh.force.z;

    % est_stance_val(3,1,i) = est_force_msg{i}.rf.force.x;
    % est_stance_val(3,2,i) = est_force_msg{i}.rf.force.y;
    est_force_val(3,i) = est_force_msg{i}.rf.force.z;

    % est_stance_val(4,1,i) = est_force_msg{i}.rh.force.x;
    % est_stance_val(4,2,i) = est_force_msg{i}.rh.force.y;
    est_force_val(4,i) = est_force_msg{i}.rh.force.z;
end
time_init = [gt_time(1),est_time(1),gt_p_time(1),est_p_time(1),est_odom_cor_time(1),est_force_time(1),est_stance_time(1)];
offset = min(time_init)
% gt_time = gt_time - offset;
% est_time = est_time - offset;
% gt_p_time = gt_p_time - offset;
% est_p_time = est_p_time - offset;
% est_odom_cor_time = est_odom_cor_time - offset;
% est_force_time = est_force_time - offset;
% est_stance_time = est_stance_time - offset;
%%
plot_fs(est_stance_time,est_stance_val,est_force_time,est_force_val,est_odom_cor_time,est_odom_cor_values)
%%
[m_x,s_x,sup_x,fig] = plot_err(gt_time,gt_lin_vel,est_time,est_lin_vel,1,1,"velocity [m/s]","linear velocity");
saveas(fig,bag_folder + bag_name+"/linear_velocity_error_x.fig")
[m_y,s_y,sup_y,fig] = plot_err(gt_time,gt_lin_vel,est_time,est_lin_vel,2,2,"velocity [m/s]","linear velocity");
saveas(fig,bag_folder + bag_name+"/linear_velocity_error_y.fig")
[m_z,s_z,sup_z,fig] = plot_err(gt_time,gt_lin_vel,est_time,est_lin_vel,3,3,"velocity [m/s]","linear velocity");
saveas(fig,bag_folder + bag_name+"/linear_velocity_error_z.fig")
%%

[m_p_x,s_p_x,sup_p_x,fig] = plot_err(gt_p_time,gt_pos,est_p_time,est_pos,4,1,"position [m]","position");
saveas(fig,bag_folder + bag_name+"/position_error_x.fig")
[m_p_y,s_p_y,sup_p_y,fig] = plot_err(gt_p_time,gt_pos,est_p_time,est_pos,5,2,"position [m]","position");
saveas(fig,bag_folder + bag_name+"/position_error_y.fig")
[m_p_z,s_p_z,sup_p_z,fig] = plot_err(gt_p_time,gt_pos,est_p_time,est_pos,6,3,"position [m]","position");
saveas(fig,bag_folder + bag_name+"/position_error_z.fig")

%% orientation error computation
for i = 1:4
    ts = timeseries(est_ori(i,:),est_p_time);
    ts = resample(ts,gt_p_time);
    est_ori_r(i,:) = ts.Data(1,:);
    
end
for i = 1:gt_nm
    gt_q = quaternion(gt_ori(1,i),gt_ori(2,i),gt_ori(3,i),gt_ori(4,i));
    est_q = quaternion(est_ori_r(1,i),est_ori_r(2,i),est_ori_r(3,i),est_ori_r(4,i));
    diffq = gt_q*conj(est_q);
    ori_err(:,i) = quat2eul(diffq,"ZYX");
    for j = 1:3
        ori_err(j,i) = rad2deg(ori_err(j,i));
    end

end
axis = ["z","y","x"];
for i = 1:3
    f = figure(10 + i);
    plot(gt_p_time,ori_err(i,:),"Color","red")
    grid on 
    grid minor 
    xlabel("time [s]")
    ylabel("orientation error [degrees]")
    title("orientation error " + axis(i))
    savefig(f,bag_name+"/orientation_error_"+axis(i)+".fig")


end

%%
new_line = [bag_name,q_gyro,q_acc,r_vx,r_vy,r_vz,m_x,s_x,sup_x,m_y,s_y,sup_y,m_z,s_z,sup_z];
% 
% m = readmatrix("Tuning_Pronto_Exp.xls");
% 
% m = [m;new_line];
writematrix(new_line,"Tuning_Pronto_Exp_4.xls","WriteMode","append");
%%




% %% create and resample timeseries
% 
% lin_x_est_ts = timeseries(est_lin_vel(1,:),est_time);
% 
% lin_x_est_ts = resample(lin_x_est_ts,gt_time);
% 
% 
% est_lin_x_r = lin_x_est_ts.Data(1,:);
% lin_x_err = gt_lin_vel(1,:) - est_lin_x_r ;
% 
% figure(2)
% plot(gt_time,lin_x_err,"Color","red")
% grid on 
% m_err = mean(lin_x_err);
% s_err = mean(lin_x_err);
% sup_err = max(lin_x_err);
% hold on 
% plot(gt_time,ones(1,gt_nm)*m_err,"Color","black")
% plot(gt_time,ones(1,gt_nm)*(3*s_err + m_err),"Color","blue")
% plot(gt_time,ones(1,gt_nm)*(m_err - 3*s_err),"Color","blue")
% hold off
% title("x linear velocity error max: "+ sup_err)
% legend("error","mean error","std error bound")


%% Odom Corretcion plot 


%% utility 
function secs = ros_h_2_secs(stamp)
    secs = double(stamp.sec) + (double(stamp.nanosec)/10^9);
end


function [mean_err,std_err,max_err,f] = plot_err(gt_time,gt_vals,est_time,est_vals,fig_num,vers,meas,meas_t)
    
    if(vers == 1)
        axis = "X";
    elseif (vers == 2)
        axis = "Y";
    elseif(vers == 3)
        axis = "Z";
    else
        axis="VIVI IN UN MONDO TRIDIMENSIONALE, RIPIGLIATI"
        assert(true)
    end
    
    ts = timeseries(gt_vals(vers,:),gt_time);
    ts = resample(ts,est_time);
    err =  ts.Data(1,:) - est_vals(vers,:);
    size(err)
    f = figure(fig_num);
    subplot(2,1,1)
    grid on 
    grid minor
    hold on 
    plot(gt_time,gt_vals(vers,:),"Color","red")
    
    plot(est_time,est_vals(vers,:),"Color","blue","LineStyle","--")
    hold off 
    legend("ground truth","estimation")
    xlabel("time[s]")
    ylabel(meas)
    subplot(2,1,2)
    plot(est_time,err,"Color","red")
    hold on 
    grid on 
    grid minor
    
    mean_err = mean(err);
    std_err = std(err);
    max_err = max(err);
    gt_nm = length(gt_time);

    plot(gt_time,ones(1,gt_nm)*mean_err,"Color","black")
    plot(gt_time,ones(1,gt_nm)*(3*std_err + mean_err),"Color","blue")
    plot(gt_time,ones(1,gt_nm)*(mean_err - 3*std_err),"Color","blue")
    hold off
    xlabel("time[s]")
    ylabel(meas)
    title(axis+" "+meas_t +" error max: "+ max_err)
    legend("error","mean error","std error bound")

    % saveas(gcf,bagname+"/estimation_error_"+axis+".fig")
end

function [fig] = plot_fs(s_time,s_val,f_time,f_val,odom_t,odom_v)
    
    fig = figure(15);
    title("Stance/Force graph")
    subplot(3,2,1)
    hold on 
    plot(s_time,s_val(1,:)*200,"Color","red")
    plot(f_time,f_val(1,:),"Color","blue")
    legend("FH Z stance est","HL force")
    hold off 

    subplot(3,2,2)
    hold on 
    plot(s_time,s_val(2,:)*200,"Color","red")
    plot(f_time,f_val(2,:),"Color","blue")
    legend("FH Z stance est","HL force")
    hold off 

    subplot(3,2,3)
    hold on 
    plot(s_time,s_val(3,:)*200,"Color","red")
    plot(f_time,f_val(3,:),"Color","blue")
    legend("RL Z stance est","FR force")
    hold off 

    subplot(3,2,4)
    hold on 
    plot(s_time,s_val(4,:)*200,"Color","red")
    plot(f_time,f_val(4,:),"Color","blue")
    legend("RH Z stance est","HR force")
    hold off 

    subplot(3,2,5)
    plot(odom_t,odom_v(1,:),"Color","black")

    subplot(3,2,6)
    plot(odom_t,odom_v(2,:),"Color","black")

end
