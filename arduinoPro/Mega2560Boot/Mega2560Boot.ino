/*
   rosserial Subscriber Example
   Blinks an LED on callback
*/

#include "ros_core_config.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "pid_controller.h"

uint8_t signal = 0;
double a[3], w[3], angle[3], q0, q1, q2, q3;
extern SetPointInfo leftMotor, rightMotor;

/*******************************************************************************
  ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
  Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

/*******************************************************************************
  Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
turtlebot3_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Command velocity of Turtlebot3 using RC100 remote controller
//geometry_msgs::Twist cmd_vel_rc100_msg;
//ros::Publisher cmd_vel_rc100_pub("cmd_vel_rc100", &cmd_vel_rc100_msg);

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
static uint32_t tTime[4];

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

/*******************************************************************************
  Declaration for LED
*******************************************************************************/
#define LED_TXD         0
#define LED_RXD         1
#define LED_LOW_BATTERY 2
#define LED_ROS_CONNECT 3

/*******************************************************************************
  Declaration for Battery
*******************************************************************************/
#define BATTERY_POWER_OFF             0
#define BATTERY_POWER_STARTUP         1
#define BATTERY_POWER_NORMAL          2
#define BATTERY_POWER_CHECK           3
#define BATTERY_POWER_WARNNING        4

static bool    setup_end       = false;
static uint8_t battery_voltage = 0;
static float   battery_valtage_raw = 0;
static uint8_t battery_state   = BATTERY_POWER_OFF;

void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(cmd_vel_sub);//订阅cmd_vel主题
  nh.advertise(sensor_state_pub);//发布一系列主题
  nh.advertise(imu_pub);
  //nh.advertise(cmd_vel_rc100_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  tfbroadcaster.init(nh);

  nh.loginfo("Connected to OpenCR board!");

  //初始化电机PID和编码器
  initEncoders();//初始化编码器中断库对象
  initMotorController();//初始化电机控制引脚
  initPID();//初始化PID控制器
  resetPID();

  // 初始化IMU
  Serial1.begin(115200);//IMU
  Serial.begin(115200);//ROS

  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  //remote_controller.begin(1);  // 57600bps baudrate for RC100 control

  //cmd_vel_rc100_msg.linear.x  = 0.0;
  //cmd_vel_rc100_msg.angular.z = 0.0;

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

  //SerialBT2.begin(57600);

  setup_end = true;
}

void loop()
{
  //接收上位机发送的行走控制命令
  //receiveRemoteControlData();

  if ((millis() - tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))
  {
    updatePID();
    tTime[0] = millis();
  }

  if ((millis() - tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD))
  {
    publishSensorStateMsg();
    publishDriveInformation();
    tTime[2] = millis();
  }

  if ((millis() - tTime[3]) >= (1000 / IMU_PUBLISH_PERIOD))
  {
    publishImuMsg();
    tTime[3] = millis();
  }

  // Check push button pressed for simple test drive
  //checkPushButtonState();

  //采用中断的方式获取IMU的值
  //imu.update();
  if(signal == 1){
    a[0] = (double)JY901.stcAcc.a[0]/32768*16;
    a[1] = (double)JY901.stcAcc.a[1]/32768*16;
    a[2] = (double)JY901.stcAcc.a[2]/32768*16;
    w[0] = (double)JY901.stcGyro.w[0]/32768*2000;
    w[1] = (double)JY901.stcGyro.w[1]/32768*2000;
    w[2] = (double)JY901.stcGyro.w[2]/32768*2000;
    angle[0] = (double)JY901.stcAngle.Angle[0]/32768*180;
    angle[1] = (double)JY901.stcAngle.Angle[1]/32768*180;
    angle[2] = (double)JY901.stcAngle.Angle[2]/32768*180;
    q0 = (double)JY901.stcDStatus.sDStatus[0]/32768;
    q1 = (double)JY901.stcDStatus.sDStatus[1]/32768;
    q2 = (double)JY901.stcDStatus.sDStatus[2]/32768;
    q3 = (double)JY901.stcDStatus.sDStatus[3]/32768;
    signal = 0;
  }

  // Start Gyro Calibration after ROS connection
  //updateGyroCali();

  // Show LED status
  //showLedStatus();

  // Update Voltage
  //updateVoltageCheck();

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();
}

//串口中断程序
void serialEvent1() {
  while (Serial1.available()) {
    JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
    signal = 1;
  }
}


/*******************************************************************************
  Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_linear_velocity  = cmd_vel_msg.linear.x;
  goal_angular_velocity = cmd_vel_msg.angular.z;

  static double wheel_speed_cmd[2];
  static double lin_vel1;
  static double lin_vel2;

  wheel_speed_cmd[LEFT]  = goal_linear_velocity - (goal_angular_velocity * WHEEL_SEPARATION / 2);//速度m/s
  wheel_speed_cmd[RIGHT] = goal_linear_velocity + (goal_angular_velocity * WHEEL_SEPARATION / 2);
  if(wheel_speed_cmd[LEFT] > MAX_LINEAR_VELOCITY){
    wheel_speed_cmd[LEFT] = MAX_LINEAR_VELOCITY;
  }
  if(wheel_speed_cmd[RIGHT] > MAX_LINEAR_VELOCITY){
    wheel_speed_cmd[RIGHT] = MAX_LINEAR_VELOCITY;
  }

  leftMotor.TargetTicksPerFrame = wheel_speed_cmd[LEFT];
  rightMotor.TargetTicksPerFrame = wheel_speed_cmd[RIGHT];
}

/*******************************************************************************
  Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
float checkVoltage(void)
{
  float vol_value;

  //vol_value = getPowerInVoltage();
  vol_value = 12.0;

  return vol_value;
}

void publishSensorStateMsg(void)
{
  bool dxl_comm_result = false;

  int32_t current_tick;

  sensor_state_msg.stamp = nh.now();
  sensor_state_msg.battery = checkVoltage();

  //motor_driver.readEncoder(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);
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
bool updateOdometry(double diff_time)
{
  double odom_vel[3];

  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, delta_theta;
  double temp_angle = angle[2];
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
  //delta_theta = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) - last_theta;
  if(angle[2] < 0 && last_theta > 0){
    delta_theta = 180 - last_theta + 180 + angle[2];
  }else{
    delta_theta = temp_angle - last_theta; 
  }  

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

  //last_theta = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
  last_theta = temp_angle;

  return true;
}
void updateJoint(void)
{
  joint_states_pos[LEFT]  = last_rad_[LEFT];
  joint_states_pos[RIGHT] = last_rad_[RIGHT];

  joint_states_vel[LEFT]  = last_velocity_[LEFT];
  joint_states_vel[RIGHT] = last_velocity_[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}
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

/**************************************************************
 * 欧拉角转四元数
 */
void eulerAnglesToQuaternion(void){
  static float cosRoll = cosf(angle[0]*0.5f);
  static float sinRoll = sinf(angle[0]*0.5f);

  static float cosPitch = cosf(angle[1]*0.5f);
  static float sinPitch = sinf(angle[1]*0.5f);

  static float cosHeading = cosf(angle[2]*0.5f);
  static float sinHeading = sinf(angle[2]*0.5f);
  q0 = cosRoll*cosPitch*cosHeading + sinRoll*sinPitch*sinHeading;
  q1 = sinRoll*cosPitch*cosHeading - cosRoll*sinPitch*sinHeading;
  q2 = cosRoll*sinPitch*cosHeading + sinRoll*cosPitch*sinHeading;
  q3 = cosRoll*cosPitch*sinHeading - sinRoll*sinPitch*cosHeading;  
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg.header.stamp    = nh.now();
  imu_msg.header.frame_id = "imu_link";

  imu_msg.angular_velocity.x = w[0];
  imu_msg.angular_velocity.y = w[1];
  imu_msg.angular_velocity.z = w[2];
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration.x = a[0];
  imu_msg.linear_acceleration.y = a[1];
  imu_msg.linear_acceleration.z = a[2];
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

  tfs_msg.transform.translation.x = -0.032;
  tfs_msg.transform.translation.y = 0.0;
  tfs_msg.transform.translation.z = 0.068;

  tfbroadcaster.sendTransform(tfs_msg);
}
