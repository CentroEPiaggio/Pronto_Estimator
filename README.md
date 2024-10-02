# Pronto_Estimator
This is an  EKF base filetr implementation adpted to floating base system and then update to be use in ROS2 Framework and add some module to manage wheeled system.

The system offer a span of module to manage different sensors subscribing to different topics.
The available list of module contains:
<ol>
    <li>ins: inertial navigation system(IMU)</li>
    <li>legodo: legged odometry</li>
    <li>bias_lock: bias updater when sytem is standing</li>
    <li>scan_matcher: lidar scan matcher </li>
    <li>qualysis_mt: qualysis motion tracker</li>

</ol>

### ROS2 Code UML 
![Prova](/doc/pronto_node_ros1.png)

### Pronto Estimator Parameters


#### estimator parameters

<ul>
    <li>pose_topic: Estimated Pose topic name </li>
    <li>pose_frame_id: pose tf frame name  </li>
    <li>twist_topic: Estimated Twist topic name</li>
    <li>publish_pose: Bool if true publish pose </li>
    <li>publish_tf: Bool if true publish tf </li>
    <li>tf_child_frame_id </li>
    <li>republish_sensors </li>
    <li>init_sensors: sensor list used to init the filter </li>
    <li>active_sensors: sensor list used to update the estimation  </li>
    <li>utime_history_span: max update temporal span dimension </li>
    <li>sigma0: init covariance </li>
    <li>x0: init state </li>
</ul>

#### base module parameters

<ul>
    <li>topic: module subcrition topic name </li>
    <li>roll_forward_on_receive: Bool if true update the estimation when the message is recieved </li>
    <li>publish_head_on_message: Bool if true publish the estimarion when the message is recieved </li>
</il>

#### ins parameters

<ul>    
    <li>q_gyro: gyro measures covariance</li>
    <li>q_accel: accelerometer measure covariance</li>
    <li>q_gyro_bias: gyro bias covariance </li>
    <li>q_accel_bias: accelerometer bias covariance</li>
    <li>num_to_init: message number needed to initialize ins module </li>
    <li>gyro/accel_bias_initial: bias initial value </li>
    <li>gyro/accel_bias_recalc_at_start: Bool if true recalc bias during the module initialization </li>
    <li>gyro/accel_bias_update_online: Bool if true recalc bias during the module initialization </li>
    <li>frame: imu tf frame name</li>
    <li>base_frame: base tf frame name</li>
</ul>

#### legodo parameters

<ul>
    <li>legodo_mode: it desctibe the covariance computation  mode between: STATIC_SIGMA, VAR_SIGMA,IMPACT_SIGMA, WEIGHTED_AVG and ALPHA_FILTER  </li>
    <li>stance_mode: it describe the contact detection mode between: THRESHOLD, HYSTERESIS, and  REGRESSION</li>
    <li>stance_threshold: threshold value</li>
    <li>stance_hysteresis_low</li>
    <li>stance_hysteresis_high</li>
    <li>stance_hysteresis_delay_low</li>
    <li>stance_hysteresis_delay_high</li>
    <li>stance_alpha</li>
    <li>stance_regression_beta_size</li>
    <li>stance_regression_beta</li>
    <li>r_vx: init covariance in x direction </li>
    <li>r_vy: init covariance in y direction</li>
    <li>r_vz: init covariance in z direction</li>
    <li>sim: Bool if true the robot use the sensor_msgs otherwise it use pi3hat_msgs</li>
</ul>

#### bias_lock parameters

<ul>
    <li>torque_threshold: minimum torque on knee to be in contact with the ground </li>
    <li>velocity_threshold: maximum velocity to consider the robot standing</li>
    <li>secondary_topic: joint state topic name</li>
    <li>sim: Bool if true the robot use the sensor_msgs otherwise it use pi3hat_msgs</li>
</ul>

#### scan_matcher parameters

<ul>
    <li>mode: it describe the correction mode between position, yaw and position with yaw</li>
    <li>r_yaw: yaw covariance</li>
    <li>r_pxy: position covariance</li>
    
</ul>

#### Wheel Odomeretry parameters

<ul>
    <li>mode: it describe the correction mode between linear velocity, agular_velocity and both</li>
    <li>r_linear: velocity covariance</li>
    <li>r_chi: angular covariance</li>
    
</ul>

#### qualysis_mt parameters

<ul>
    <li>robot_name: qualisys rigid body name</li>
    <li>r_xyz position covariance</li>
    <li>r_chi: orientation covariance</li>
    <li>mode: it describe the correction mode between position, yaw and position with yaw, orientation and position with orientation</li>
    
</ul>

## Usage 

### Start Estimator
The estimator node can be start in this way:

``` ros2 launch  pronto_ros2_node pronto_node.launch.py xacro_pkg:=<robot_description_package> xacro_name:=<xacro_file_name> config_name:=<configuration_file name>```

The config file must be add to the config folder in pronto_ros2_node package.

### Benchmarking
The Benchmarking launch allows the user to start a list of pronto instance to compare the obtained estimations. The input is a bag cointains all the filter measures input, or at least it should be consistent with the configuration files. The configuration files has to be placed into the config folder and named omnicar_tune_i.yaml, each file is associated with a pronto instace. 
The launch file cointain the global variable to set the bag name and the list of instance number, subset of the configuration files' list.
To start the filters the command line is:

``` ros2 launch pronto_ros2_node bench_pronto.launch.py ```

### Dependecies
Eigen3


<!-- ### Class and methods
<ol>
    <li>
    ROS_FrontEnd (Pronto Ros)
    <ul>
        <li>
        Constructor: get the topic name of the estimate position, velocity and tf from ROS parameter; at the end initialize filter state and covariance using its own method
        </li>
        <li>
        initializeState: get the initial state, pose and velocity,  from ROS parameter and set the init state variable.
        </li>
        <li>
        initializeCovariance: get the initial covariance matrix from ROS parameter and set the init covariance variable.
        </li>
        <li>
            initializeFilter: check if the sensors module are initialize and if the filter is not then initialize it.
        </li>
        <li>
            areModouleInitialize: evaluate the initialization of all the sensors that need it.
        </li>
        <li>
            addSensingModule: create the data structure for the sensor: maps and topic.
        </li>
        <li>
            InitCallback: callback to initialize the sensing module, when initialized the sensor destroy the topic.
        </li>
        <li>
            callback: data topic subscribe callback, compute the update due to the data update.
        </li>
        </li>
    </ul>
    </li>
    <li>
        InsHandlerRos(Pronto_ROS): class used by the frontend to manage the InsModule.
        <ul>
        <li>
                Constructor: build the InsModule and create its topic
            </li>
            <li>
                ProcessMessage: get the IMU message and the estimator to return the state and covariance update.
            </li>
            <li>
                processMessageInit: get the IMU message and return a boolean reporting the initialization result.
            </li>
        </ul>
    </li>
    <li>
        SensinModule (Pronto_Core): virtual class to describe generic sensor.
        <ul>
            <li>
                ProcessMessage(virtual): get the sensor data and the estimator to return the state and covariance update.
            </li>
            <li>
                processMessageInit(virtual): get the init message and return a boolean reporting the initialization result.
            </li>
        </ul>
    </li>
    <li>
        DualSensinModule (Pronto_Core): virtual class to describe a sensing module with two sensor input.
        <ul>
            <li>
                ProcessMessage(virtual): get the sensor data and the estimator to return the state and covariance update.
            </li>
            <li>
                processMessageInit(virtual): get the init message and return a boolean reporting the initialization result.
            </li>
        </ul>
    </li>
    <li>
        InsModule(Pronto_Core):
        <ul>
            <li>
                Constructor: build the inertial sensing modue from the imu configuration and the imu-body frame tranformation
            </li>
            <li>
                ProcessMessage: get the IMU data and the estimator to return the state and covariance update.
            </li>
            <li>
                processMessageInit: get the IMU init message and return a boolean reporting the initialization result.
            </li>
        </ul>
    </li>
</ol> -->
