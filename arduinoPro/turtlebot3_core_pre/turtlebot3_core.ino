#include "turtlebot3_core_config.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "pid_controller.h"

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
turtlebot3_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Command velocity of Turtlebot3 using RC100 remote controller
geometry_msgs::Twist cmd_vel_rc100_msg;
ros::Publisher cmd_vel_rc100_pub("cmd_vel_rc100", &cmd_vel_rc100_msg);

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

/*******************************************************************************
  Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped tfs_msg;
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tfbroadcaster;

/*******************************************************************************
  SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[5];

/*******************************************************************************
  Declaration for motor
*******************************************************************************/
//Turtlebot3MotorDriver motor_driver;
bool init_encoder_[2]  = {false, false};
int32_t last_diff_tick_[2];
int32_t last_tick_[2];
double last_rad_[2];
double last_velocity_[2];
double goal_linear_velocity  = 0.0;
double goal_angular_velocity = 0.0;

/*******************************************************************************
  Declaration for IMU
*******************************************************************************/
//cIMU imu;
int aix, aiy, aiz;// 加速度计原始数据
int gix, giy, giz;// 陀螺仪原始数据
float ax, ay, az;//加速度值
float gx, gy, gz;//角速度值
float angle[3], q0, q1, q2, q3;
Madgwick filter;
extern SetPointInfo leftMotor, rightMotor;

/*******************************************************************************
  Declaration for RC100 remote controller
*******************************************************************************/
//RC100 remote_controller;
//double const_cmd_vel    = 0.2;

/*******************************************************************************
  Declaration for test drive
*******************************************************************************/
bool start_move = false;
bool start_rotate = false;
int32_t last_left_encoder  = 0;
int32_t last_right_encoder = 0;

/*******************************************************************************
  Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
char *joint_states_name[] = {"wheel_left_joint", "wheel_right_joint"};
float joint_states_pos[2] = {0.0, 0.0};
float joint_states_vel[2] = {0.0, 0.0};
float joint_states_eff[2] = {0.0, 0.0};

static bool    setup_end       = false;

/*******************************************************************************
  Setup function
*******************************************************************************/
void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(cmd_vel_sub);
  nh.advertise(sensor_state_pub);
  nh.advertise(imu_pub);
  nh.advertise(cmd_vel_rc100_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  tfbroadcaster.init(nh);

  nh.loginfo("Connected to OpenCR board!");

  //motor_driver.init();
  //初始化电机PID和编码器
  initEncoders();//初始化编码器中断库对象
  initMotorController();//初始化电机控制引脚
  initPID();//初始化PID控制器
  resetPID();

  Serial.begin(115200);//ROS

  // Setting for IMU
  //imu.begin();
  CurieIMU.begin();
  CurieIMU.setAccelerometerRate(200);
  CurieIMU.setAccelerometerRange(8);//参照opencr设置
  CurieIMU.setGyroRate(200);
  CurieIMU.setGyroRange(2000);//(+/-2000°/s)
  filter.begin(200);

  CurieIMU.autoCalibrateGyroOffset();//陀螺仪自动校准
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);//加速度自动计校准
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  //remote_controller.begin(1);  // 57600bps baudrate for RC100 control ---> Serial2

  cmd_vel_rc100_msg.linear.x  = 0.0;
  cmd_vel_rc100_msg.angular.z = 0.0;

  // Setting for SLAM and navigation (odometry, joint states, TF)
  odom_pose[0] = 0.0;
  odom_pose[1] = 0.0;
  odom_pose[2] = 0.0;

  joint_states.header.frame_id = "base_footprint";
  joint_states.name            = joint_states_name;

  joint_states.name_length     = 2;
  joint_states.position_length = 2;
  joint_states.velocity_length = 2;
  joint_states.effort_length   = 2;

  prev_update_time = millis();

  pinMode(13, OUTPUT);

  setup_end = true;  
}

/*******************************************************************************
  Loop function
*******************************************************************************/
void loop()
{  
  //receiveRemoteControlData();
  /*Serial.println(sizeof(sensor_state_msg));//28
  Serial.println(sizeof(imu_msg));//184
  Serial.println(sizeof(cmd_vel_rc100_msg));//36
  Serial.println(sizeof(odom));//400
  Serial.println(sizeof(joint_states));//72
  Serial.println(sizeof(tfs_msg));//68
  Serial.println(sizeof(odom_tf));//68  
  delay(2000);*/

  if ((millis() - tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD)) {
    updatePID();
    tTime[0] = millis();
  }

  if ((millis() - tTime[1]) >= (1000 / CMD_VEL_PUBLISH_PERIOD)) {
    cmd_vel_rc100_pub.publish(&cmd_vel_rc100_msg);
    tTime[1] = millis();
  }

  if ((millis() - tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD)) {
    //publishSensorStateMsg();
    //publishDriveInformation();
    tTime[2] = millis();
  }

  if ((millis() - tTime[3]) >= (1000 / IMU_PUBLISH_PERIOD)) {
    publishImuMsg();
    tTime[3] = millis();
  }

  if ((millis() - tTime[4]) >= (1000 / 200)) {//陀螺仪数据发布频率200hz
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
    ax = (aix * 8.0) / 32768.0;
    ay = (aiy * 8.0) / 32768.0;
    az = (aiz * 8.0) / 32768.0;
    gx = (gix * 2000.0) / 32768.0;
    gy = (giy * 2000.0) / 32768.0;
    gz = (giz * 2000.0) / 32768.0;
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    //计算欧拉角
    angle[0] = filter.getRoll();
    angle[1] = filter.getPitch();
    angle[2] = filter.getYaw();
    //计算四元数
    q0 = filter.q0;
    q1 = filter.q1;
    q2 = filter.q2;
    q3 = filter.q3;
    tTime[4] = millis();
  }
  
  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();
}

/*******************************************************************************
  Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_linear_velocity  = cmd_vel_msg.linear.x;
  goal_angular_velocity = cmd_vel_msg.angular.z;

  static double wheel_speed_cmd[2];

  wheel_speed_cmd[LEFT]  = goal_linear_velocity - (goal_angular_velocity * WHEEL_SEPARATION / 2);//速度m/s
  wheel_speed_cmd[RIGHT] = goal_linear_velocity + (goal_angular_velocity * WHEEL_SEPARATION / 2);
  if (wheel_speed_cmd[LEFT] > MAX_LINEAR_VELOCITY) {
    wheel_speed_cmd[LEFT] = MAX_LINEAR_VELOCITY;
  }
  if (wheel_speed_cmd[RIGHT] > MAX_LINEAR_VELOCITY) {
    wheel_speed_cmd[RIGHT] = MAX_LINEAR_VELOCITY;
  }

  leftMotor.TargetTicksPerFrame = wheel_speed_cmd[LEFT];
  rightMotor.TargetTicksPerFrame = wheel_speed_cmd[RIGHT];
}

/*******************************************************************************
  Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg.header.stamp    = nh.now();
  imu_msg.header.frame_id = "imu_link";

  imu_msg.angular_velocity.x = gx;
  imu_msg.angular_velocity.y = gy;
  imu_msg.angular_velocity.z = gz;
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.orientation.w = q0;
  imu_msg.orientation.x = q1;
  imu_msg.orientation.y = q2;
  imu_msg.orientation.z = q3;

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;

  imu_pub.publish(&imu_msg);

  tfs_msg.header.stamp    = nh.now();
  tfs_msg.header.frame_id = "base_link";
  tfs_msg.child_frame_id  = "imu_link";
  tfs_msg.transform.rotation.w = q0;
  tfs_msg.transform.rotation.x = q1;
  tfs_msg.transform.rotation.y = q2;
  tfs_msg.transform.rotation.z = q3;

  tfs_msg.transform.translation.x = 0.032;
  tfs_msg.transform.translation.y = 0.0;
  tfs_msg.transform.translation.z = 0.068;

  tfbroadcaster.sendTransform(tfs_msg);
}

/*******************************************************************************
  Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void)
{
  int32_t current_tick;

  sensor_state_msg.stamp = nh.now();
  sensor_state_msg.battery = 12.0;  
  sensor_state_msg.left_encoder = readEncoder(LEFT);//读取编码器的值
  sensor_state_msg.right_encoder = readEncoder(RIGHT);
  sensor_state_pub.publish(&sensor_state_msg);

  current_tick = sensor_state_msg.left_encoder;

  if (!init_encoder_[LEFT])
  {
    last_tick_[LEFT] = current_tick;
    init_encoder_[LEFT] = true;
  }

  last_diff_tick_[LEFT] = current_tick - last_tick_[LEFT];
  last_tick_[LEFT] = current_tick;
  last_rad_[LEFT] += TICK2RAD * (double)last_diff_tick_[LEFT];

  current_tick = sensor_state_msg.right_encoder;

  if (!init_encoder_[RIGHT])
  {
    last_tick_[RIGHT] = current_tick;
    init_encoder_[RIGHT] = true;
  }

  last_diff_tick_[RIGHT] = current_tick - last_tick_[RIGHT];
  last_tick_[RIGHT] = current_tick;
  last_rad_[RIGHT] += TICK2RAD * (double)last_diff_tick_[RIGHT];
}

/*******************************************************************************
  Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;
  prev_update_time = time_now;
  ros::Time stamp_now = nh.now();

  // odom
  updateOdometry((double)(step_time * 0.001));
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // joint_states
  updateJoint();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);

  // tf
  updateTF(odom_tf);
  tfbroadcaster.sendTransform(odom_tf);
}

/*******************************************************************************
  Calculate the odometry
*******************************************************************************/
bool updateOdometry(double diff_time)
{
  double odom_vel[3];

  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick_[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick_[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  delta_theta = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) - last_theta;

  v = delta_s / step_time;
  w = delta_theta / step_time;

  last_velocity_[LEFT]  = wheel_l / step_time;
  last_velocity_[RIGHT] = wheel_r / step_time;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  // We should update the twist of the odometry
  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];

  last_theta = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);

  return true;
}

/*******************************************************************************
  Calculate the joint states
*******************************************************************************/
void updateJoint(void)
{
  joint_states_pos[LEFT]  = last_rad_[LEFT];
  joint_states_pos[RIGHT] = last_rad_[RIGHT];

  joint_states_vel[LEFT]  = last_velocity_[LEFT];
  joint_states_vel[RIGHT] = last_velocity_[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
  Calculate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom.header.frame_id = "odom";
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}

