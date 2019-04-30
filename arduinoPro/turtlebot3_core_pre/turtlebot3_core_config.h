#ifndef TURTLEBOT3_CORE_CONFIG_H_
#define TURTLEBOT3_CORE_CONFIG_H_

#include <math.h>

//#define USE_USBCON
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>

//#include "IMU.h"
//#include <RC100.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h>

//#include "turtlebot3_motor_driver.h"

#define CONTROL_MOTOR_SPEED_PERIOD       30   //hz
#define IMU_PUBLISH_PERIOD               100  //hz
#define SENSOR_STATE_PUBLISH_PERIOD      30   //hz
#define CMD_VEL_PUBLISH_PERIOD           30   //hz
#define DRIVE_INFORMATION_PUBLISH_PERIOD 30   //hz
#define DRIVE_TEST_PERIOD                30   //hz

#define WHEEL_RADIUS                     0.063           // meter
#define WHEEL_SEPARATION                 0.350           // meter (BURGER : 0.160, WAFFLE : 0.287)
#define TURNING_RADIUS                   0.1435          // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                     0.203           // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define LEFT                             0
#define RIGHT                            1

#define VELOCITY_CONSTANT_VALUE          1263.632956882  // V = r * w = r * RPM * 0.10472
                                                         //   = 0.033 * 0.229 * Goal RPM * 0.10472
                                                         // Goal RPM = V * 1263.632956882

#define MAX_LINEAR_VELOCITY              0.60   // m/s (BURGER => 0.22, WAFFLE => 0.25)
#define MAX_ANGULAR_VELOCITY             1.82   // rad/s (BURGER => 2.84, WAFFLE => 1.82)
#define VELOCITY_STEP                    0.01   // m/s
#define VELOCITY_LINEAR_X                0.01   // m/s
#define VELOCITY_ANGULAR_Z               0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X          1
#define SCALE_VELOCITY_ANGULAR_Z         1

#define TICK2RAD                         0.01047198  //2*PI/600

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14      // 180 degree

#define WAIT_FOR_BUTTON_PRESS            0
#define WAIT_SECOND                      1
#define CHECK_BUTTON_RELEASED            2

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);

// Function prototypes
void publishImuMsg(void);
void publishSensorStateMsg(void);
void publishDriveInformation(void);
bool updateOdometry(double diff_time);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void receiveRemoteControlData(void);
void controlMotorSpeed(void);
uint8_t getButtonPress(void);
void testDrive(void);
void checkPushButtonState(void);
float checkVoltage(void);
void showLedStatus(void);
void updateRxTxLed(void);

#endif // TURTLEBOT3_CORE_CONFIG_H_
