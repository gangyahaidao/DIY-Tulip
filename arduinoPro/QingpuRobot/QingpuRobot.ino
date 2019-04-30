#define BAUDRATE     115200
#define MAX_PWM        255

#include "Arduino.h"
#include "commands.h"
#include "motor_driver.h"
#include "pid_controller.h"
#include "encoder_driver.h"
#include "QingpuRobot_core_config.h"

#include <Wire.h>
#include <JY901.h>

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
//turtlebot3_msgs::SensorState sensor_state_msg;
//ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped tfs_msg;
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tfbroadcaster;

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[4];

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
// Turtlebot3MotorDriver motor_driver;
bool init_encoder_[2]  = {false, false};
int32_t last_diff_tick_[2];
int32_t last_tick_[2];
double last_rad_[2];
double last_velocity_[2];
double goal_linear_velocity  = 0.0;
double goal_angular_velocity = 0.0;

/*******************************************************************************
* Declaration for test drive
*******************************************************************************/
bool start_move = false;
bool start_rotate = false;
int32_t last_left_encoder  = 0;
int32_t last_right_encoder = 0;

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
char *joint_states_name[] = {"wheel_left_joint", "wheel_right_joint"};
float joint_states_pos[2] = {0.0, 0.0};
float joint_states_vel[2] = {0.0, 0.0};
float joint_states_eff[2] = {0.0, 0.0};

/*******************************************************************************
* Declaration for LED
*******************************************************************************/
#define LED_TXD         0
#define LED_RXD         1
#define LED_LOW_BATTERY 2
#define LED_ROS_CONNECT 3

/*******************************************************************************
* Declaration for Battery
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


#define PID_RASIO 50.0
#define PID_RATE           30     // Hz
const int PID_INTERVAL = 1000 / PID_RATE;
unsigned long nextPID = PID_INTERVAL;
#define AUTO_STOP_INTERVAL 3000*100
long lastMotorCommand = AUTO_STOP_INTERVAL;
extern unsigned char moving;
extern SetPointInfo leftMotor, rightMotor;
extern double Kp;
extern double Ki;
extern double Kd;
extern PID myPIDL, myPIDR;
/* Variable initialization */
int arg = 0;
int index = 0;
char chr;
char cmd;
char argv1[16];
char argv2[16];
long arg1;
long arg2;
unsigned long time = 0, old_time = 0;
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case READ_PIDIN:
      Serial.print( readPidIn(LEFT));
      Serial.print(" ");
      Serial.println( readPidIn(RIGHT));
      break;
    case READ_PIDOUT:
      Serial.print( readPidOut(LEFT));
      Serial.print(" ");
      Serial.println( readPidOut(RIGHT));
      break;
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        moving = 0;
      }else{
        moving = 1;  
      }
      //需要将接收的每帧tick数转换为mm/s
      leftMotor.TargetTicksPerFrame = arg1*19.792034;//向全局对象变量赋值,最大支持输入33
      rightMotor.TargetTicksPerFrame = arg2*19.792034;
      Serial.println("OK");
      break;
    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != '\0') {
        pid_args[i] = atoi(str);
        i++;
      }
      Kp = pid_args[0]/PID_RASIO;
      Ki = pid_args[1]/PID_RASIO;
      Kd = pid_args[2]/PID_RASIO;
      myPIDL.SetTunings(Kp, Ki, Kd);
      myPIDR.SetTunings(Kp, Ki, Kd);
      Serial.println("PID OK");
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
}
/***********************************************
 * 程序初始化setup()
 ***********************************************/
void setup() {  
  Serial.begin(BAUDRATE);//连接到上位机
  Serial2.begin(BAUDRATE);//连接到陀螺仪
  
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(cmd_vel_sub);
  nh.advertise(sensor_state_pub);
  nh.advertise(imu_pub);
  // nh.advertise(cmd_vel_rc100_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  tfbroadcaster.init(nh);

  nh.loginfo("Connected to OpenCR board!");

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

  //初始化电机PID和编码器
  initEncoders();//初始化编码器中断库对象
  initMotorController();//初始化电机控制引脚
  initPID();//初始化PID控制器
  resetPID();
  
  setup_end = true;
}

void loop() {
  while (Serial.available() > 0) {//直接接收串口的命令
    chr = Serial.read();
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    else if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        cmd = chr;
      }
      else if (arg == 1) {
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  //按照约定的时间间隔执行不同功能程序
  //1.使用下发的速度指令控制电机
  if ((millis()-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD)){
    //设置左右轮目标速度
    leftMotor.TargetTicksPerFrame = goal_linear_velocity - (goal_angular_velocity * WHEEL_SEPARATION / 2)*1000;//mm/s
    rightMotor.TargetTicksPerFrame = goal_linear_velocity + (goal_angular_velocity * WHEEL_SEPARATION / 2);
    updatePID();
    tTime[0] = millis();
  }
  //2.发布传感器状态信息，编码器等
  /*if ((millis()-tTime[1]) >= (1000 / SENSOR_STATE_PUBLISH_PERIOD)){
    publishSensorStateMsg();
    tTime[1] = millis();
  }*/
  //3.发布消息：odometry, joint states, tf
  if ((millis()-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD)){
    publishDriveInformation();
    tTime[2] = millis();
  }
  //4.发布imu消息
  if ((millis()-tTime[3]) >= (1000 / IMU_PUBLISH_PERIOD)){
    publishImuMsg();
    tTime[3] = millis();
  }
  //5.获取IMU数据
  while (Serial2.available()) {
    JY901.CopeSerialData(Serial2.read()); //Call JY901 data cope function
  }
  //6.
  //showLedStatus();
  //7.
  //updateVoltageCheck();
  
  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_linear_velocity  = cmd_vel_msg.linear.x;
  goal_angular_velocity = cmd_vel_msg.angular.z;
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg.header.stamp    = nh.now();
  imu_msg.header.frame_id = "imu_link";

  imu_msg.angular_velocity.x = (float)JY901.stcGyro.w[0]/32768*500;
  imu_msg.angular_velocity.y = (float)JY901.stcGyro.w[1]/32768*500;
  imu_msg.angular_velocity.z = (float)JY901.stcGyro.w[2]/32768*500;
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration.x = (float)JY901.stcAcc.a[0]/32768*4;
  imu_msg.linear_acceleration.y = (float)JY901.stcAcc.a[1]/32768*4;
  imu_msg.linear_acceleration.z = (float)JY901.stcAcc.a[2]/32768*4;
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.orientation.w = JY901.stcDStatus.sDStatus[0]/32768.0;
  imu_msg.orientation.x = JY901.stcDStatus.sDStatus[1]/32768.0;
  imu_msg.orientation.y = JY901.stcDStatus.sDStatus[2]/32768.0;
  imu_msg.orientation.z = JY901.stcDStatus.sDStatus[3]/32768.0;

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
  tfs_msg.transform.rotation.w = JY901.stcDStatus.sDStatus[0]/32768.0;
  tfs_msg.transform.rotation.x = JY901.stcDStatus.sDStatus[1]/32768.0;
  tfs_msg.transform.rotation.y = JY901.stcDStatus.sDStatus[2]/32768.0;
  tfs_msg.transform.rotation.z = JY901.stcDStatus.sDStatus[3]/32768.0;

  // HCR IMU
  tfs_msg.transform.translation.x = -0.12;
  tfs_msg.transform.translation.y = 0.0;
  tfs_msg.transform.translation.z = 0.17;

  tfbroadcaster.sendTransform(tfs_msg); 
}

/*******************************************************************************
* Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
/*void publishSensorStateMsg(void)
{
  int32_t current_tick;

  sensor_state_msg.stamp = nh.now();
  sensor_state_msg.battery = 12;//checkVoltage();//获取当前电压值
  sensor_state_msg.left_encoder = readEncoder(LEFT);//读取左右编码器值
  sensor_state_msg.right_encoder = readEncoder(RIGHT);
  sensor_state_pub.publish(&sensor_state_msg);

  current_tick = sensor_state_msg.left_encoder;
  if (!init_encoder_[LEFT]){
    last_tick_[LEFT] = current_tick;
    init_encoder_[LEFT] = true;
  }
  last_diff_tick_[LEFT] = current_tick - last_tick_[LEFT];
  last_tick_[LEFT] = current_tick;
  last_rad_[LEFT] += TICK2RAD * (double)last_diff_tick_[LEFT];

  current_tick = sensor_state_msg.right_encoder;
  if (!init_encoder_[RIGHT]){
    last_tick_[RIGHT] = current_tick;
    init_encoder_[RIGHT] = true;
  }
  last_diff_tick_[RIGHT] = current_tick - last_tick_[RIGHT];
  last_tick_[RIGHT] = current_tick;
  last_rad_[RIGHT] += TICK2RAD * (double)last_diff_tick_[RIGHT];
}*/

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
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
* Calculate the odometry
*******************************************************************************/
bool updateOdometry(double diff_time){
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
  delta_theta = atan2f(imu.quat[1]*imu.quat[2] + imu.quat[0]*imu.quat[3],
                       0.5f - imu.quat[2]*imu.quat[2] - imu.quat[3]*imu.quat[3]) - last_theta;

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

  last_theta = atan2f((JY901.stcDStatus.sDStatus[1]/32768.0)*(JY901.stcDStatus.sDStatus[2]/32768.0) 
                        + (JY901.stcDStatus.sDStatus[0]/32768.0)*(JY901.stcDStatus.sDStatus[3]/32768.0),
                      0.5f - (JY901.stcDStatus.sDStatus[2]/32768.0)*(JY901.stcDStatus.sDStatus[2]/32768.0) 
                        - (JY901.stcDStatus.sDStatus[3]/32768.0)*(JY901.stcDStatus.sDStatus[3]/32768.0));

  return true;
}
/*******************************************************************************
* Calculate the joint states
*******************************************************************************/
void updateJoint(void){
  joint_states_pos[LEFT]  = last_rad_[LEFT];
  joint_states_pos[RIGHT] = last_rad_[RIGHT];

  joint_states_vel[LEFT]  = last_velocity_[LEFT];
  joint_states_vel[RIGHT] = last_velocity_[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}
/*******************************************************************************
* Calculate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf){
  odom.header.frame_id = "odom";
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}
