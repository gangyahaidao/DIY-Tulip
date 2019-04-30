#ifndef QINGPUROBOT_CORE_CONFIG_H
#define QINGPUROBOT_CORE_CONFIG_H

#include <math.h>

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

//#include <turtlebot3_msgs/SensorState.h>
//#include <IMU.h>

#define CONTROL_MOTOR_SPEED_PERIOD       30   //hz
#define IMU_PUBLISH_PERIOD               200  //hz
#define SENSOR_STATE_PUBLISH_PERIOD      30   //hz
#define DRIVE_INFORMATION_PUBLISH_PERIOD 30   //hz

/*******************************************************************************
* Robot geometric parameters
*******************************************************************************/
#define WHEEL_RADIUS                     0.063           // meter (HCR : 0.068, BURGER : 0.033, WAFFLE : 0.033)
#define WHEEL_CIRCUMFERENCE              0.395841     // meter (HCR : 0.427. BURGER : 0.207)
#define WHEEL_SEPARATION                 0.350           // meter (HCR : 0.275, BURGER : 0.160, WAFFLE : 0.287)
#define TURNING_RADIUS                   0.14          // meter (HCR : 0.1375, BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                     0.203           // meter (HCR : 0.220, BURGER : 0.105, WAFFLE : 0.220)

/*******************************************************************************
* Encoder parameters
*******************************************************************************/
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw
#define ENCODER_TICK                     600          // encoder tick per revolution: 13 * 51 * 4 (HCR : 663, BURGER : 4096)

/*******************************************************************************
* Wheel definition
*******************************************************************************/
#define LEFT                             0
#define RIGHT                            1

/*******************************************************************************
* Velocity parameters
*******************************************************************************/
#define VELOCITY_CONSTANT_VALUE           151.575782

#define LIMIT_X_MAX_VELOCITY             99    // Limit RPM values of Motor
#define MAX_LINEAR_VELOCITY              0.653139   // maxRPM / VELOCITY_CONSTANT_VALUE, m/s (HCR : < 1.0, BURGER : 0.22)
#define MAX_ANGULAR_VELOCITY             4.665278   // MAX_LINEAR_VELOCITY / TURNING_RADIUS, rad/s (HCR : < 7.3, BURGER : 2.84)
#define VELOCITY_STEP                    0.01   // m/s
#define VELOCITY_LINEAR_X                0.01   // m/s
#define VELOCITY_ANGULAR_Z               0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X          1
#define SCALE_VELOCITY_ANGULAR_Z         1

#define TICK2RAD                         0.010472  // 2 * pi / ENCODER_TICK

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

/*******************************************************************************
* Test
*******************************************************************************/
#define TEST_DISTANCE                    1.00      // meter
#define TEST_RADIAN                      3.14      // degree

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
#endif
