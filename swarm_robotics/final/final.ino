#define AIN1 11
#define AIN2 12
#define PWMA 13
#define LOUTA 2
#define LOUTB 3
//Right motor pin map
#define BIN1 10
#define BIN2 9
#define PWMB 8
#define ROUTA 19
#define ROUTB 18
#define LED	6
//Sensor
#define SENSORL A0
#define SENSORR A1
#define LDR     A2

#include <math.h>
#include <Encoder.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>           
#include <Adafruit_NeoPixel.h>

unsigned long rosSync=0;
unsigned int pub_frequency=200;  // in ms   |  5hz = 200ms
const volatile float r = 1.6  / 100.0;  //radius of the wheel
const volatile float b = 9.0 / 100.0;  //distance at base of the wheel
const float    c = 3575.04;  //encoder resolution (pulses per resolution)
const float    D = 2.0* M_PI*r/c; //linear wheel displacement for one pulse
volatile int nl_pulse,nr_pulse =0;  // number of pulse of last orientation
float w_right,w_left;
float x = 0 ,y = 0 ,theta = 0 ;
int oldLeft = 0,oldRight = 0; //old number of pulse
int newLeft = 0,newRight = 0; //current number of pulse

/* Global struct */
struct {
  byte led_R = 0;
  byte led_G = 0;
  byte led_B = 0;
  float pose_x = 0.0;
  float pose_y = 0.0;
  float pose_theta = 0.0;
  float initial_pose_x = 0.0;
  float initial_pose_y = 0.0;
  float initial_pose_theta = 0.0;
  float vel_x = 0.0;
  float vel_w = 0.0;
  boolean rumble_flag = true;
} robot;
  
/*Create note*/
ros::NodeHandle  nh;

/* TF http://wiki.ros.org/tf */
geometry_msgs::TransformStamped t;
geometry_msgs::Pose2D p;
tf::TransformBroadcaster broadcaster;

/* Publishers */
std_msgs::String debug_msg; 
std_msgs::UInt16 IR_sensor1;
std_msgs::UInt16 IR_sensor2;
std_msgs::UInt16 ldr_val;
ros::Publisher ir1("/groupJ/IR_sensor1",&IR_sensor1);
ros::Publisher ir2("/groupJ/IR_sensor2",&IR_sensor2);
ros::Publisher ldr("/groupJ/ldr",&ldr_val);
ros::Publisher pose("/groupJ/odom",&p);
// Just for debug, once the serial port is busy with the ROS serial interface
ros::Publisher pub_debug("/groupJ/arduino_debug", &debug_msg);

/* Encoder */
Encoder encL(LOUTA,LOUTB);
Encoder encR(ROUTB,ROUTA);

/*LED*/
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(0, LED, NEO_GRB + NEO_KHZ800);

void twistRB( const geometry_msgs::Twist &cmd_vel )
{
  //Controlled by kinematics
  robot.vel_x = cmd_vel.linear.x;
  robot.vel_w = cmd_vel.angular.z;
}

void rgbLED( const std_msgs::UInt8MultiArray &rgb_led)
{
  //  RGB LED
  robot.led_R = rgb_led.data[0];
  robot.led_G = rgb_led.data[1];
  robot.led_B = rgb_led.data[2]; 
}
void pose2D( const geometry_msgs::Pose2D &ini_pose)
{
  // Initial position
  robot.pose_x = ini_pose.x;
  robot.pose_y = ini_pose.y;
  robot.pose_theta = ini_pose.theta;
}
void letsRumble(const std_msgs::Bool &let_rum)
{
  // 
  robot.rumble_flag = let_rum.data;
}


ros::Subscriber<geometry_msgs::Twist> sub_cmd("/groupJ/cmd_vel", &twistRB);
ros::Subscriber<std_msgs::UInt8MultiArray> sub_rgb("/groupJ/RGB_led", &rgbLED);
ros::Subscriber<geometry_msgs::Pose2D> sub_ini("/groupJ/initial_pose", &pose2D);
ros::Subscriber<std_msgs::Bool> sub_rum("/lets_rumble", &letsRumble);


void setup()
{
  nh.initNode();

  /* init tf */
  broadcaster.init(nh);
  
  /* init pub */
  nh.advertise(ir1);
  nh.advertise(ir2);
  nh.advertise(pose);
  nh.advertise(pub_debug);  // Just for debug, once the serial port is busy with the ROS serial interface
  nh.advertise(ldr);
  nh.subscribe(sub_cmd);
  nh.subscribe(sub_rgb);
  nh.subscribe(sub_ini);
  nh.subscribe(sub_rum);
  pixels.begin();
}

void loop()
{

 
 if(robot.rumble_flag){     // The flag rumble published from pc/rpi side, will enable or disable the main routine
    
    nl_pulse = newLeft - oldLeft;
    nr_pulse = newRight - oldRight;

    oldLeft  = newLeft;
    oldRight = newRight;
    /* Your robot routines here */
    Kinematics_Update();
    Odometry_Update(nl_pulse,nr_pulse);
    newLeft = encL.read();
    newRight= encR.read();
   MotorMotion(w_left,w_right);
    /*sensor update*/
    IR_sensor1.data = sensor(0);
    IR_sensor2.data = sensor(1);

    /*Robot Position*/
    p.x = robot.pose_x;
    p.y = robot.pose_y;
    p.theta = robot.pose_theta;
    
    // TF, this will be needed for Rviz animation during the final contest. more info http://wiki.ros.org/tf
    t.header.frame_id = "/groupJ/odom";
    t.child_frame_id = "/base_link";
    t.transform.translation.x = robot.pose_x; // in meters | Pose x calculated in odometry expressions 
    t.transform.translation.y = robot.pose_y; // in meters | Pose x calculated in odometry expressions
    t.transform.rotation = tf::createQuaternionFromYaw(robot.pose_theta);// createQuaternionFromYaw() convert radians to quaternions | theta in radians | orientation calculated in odometry expressions
    t.header.stamp = nh.now();  // Just to get the time of the sample
//    pixels.setPixelColor(0,robot.led_R,robot.led_G,robot.led_B);
//    pixels.show();

    
 }

 

 if(millis()-rosSync > pub_frequency){  // Just publish only at 5hz, every 200ms
    rosSync=millis();

    ir1.publish(&IR_sensor1);
    ir2.publish(&IR_sensor2);
    pose.publish(&p);
    
    /* Just for debug, once the serial po  Odometry_Update(nl_pulse,nr_pulse);
  newLeft = encL.read();
  newRight= encR.read();
rt is busy with the ROS serial interface */
//    char final_array[40];
//    String str1 = String(robot.initial_pose_x,2);
//    String str2 = String(robot.initial_pose_y,2);
//    String str3 = String(robot.initial_pose_theta,2);
//    String str4 = String(robot.led_R);
//    String str5 = String(robot.led_G);
//    String str6 = String(robot.led_B);
//    String str7 = String(robot.vel_x,2);
//    String str8 = String(robot.vel_w,2);
//    String str9 = String(robot.rumble_flag);
//    String final_str = str1+','+str2+','+str3+','+str4+','+str5+','+str6+','+str7+','+str8+','+str9;
//    final_str.toCharArray(final_array, 40);
//    debug_msg.data = final_array;
//    pub_debug.publish( &debug_msg );
    /* --------------------------------------------------------------------------- */  
   
    
  }
  nh.spinOnce();   // update available publish msg to ros pc/rpi side
  
} 
  
  //Update Odometry system
void Odometry_Update(int left, int right)
{
  float d_left = left * D;          //Distance travelled left
  float d_right = right * D;          //Distance travelled right
  float d   = (d_right + d_left)/2.0;     //Distance travelled of the robot
  robot.pose_theta = robot.pose_theta + (d_right - d_left)/2.0;    //New theta
  robot.pose_x = robot.pose_x + d * cos(robot.pose_theta);           //New x 
  robot.pose_y = robot.pose_y + d * sin(robot.pose_theta);           //New y
}


//Update the kinematic
void Kinematics_Update()
{
  w_left = (robot.vel_x - b * robot.vel_w/2.0)*c/(r*2*M_PI);
  w_right = (robot.vel_x + robot.vel_w * b/2.0)*c/(r*2*M_PI); 
  
}


//Sensor reading
uint16_t sensor(int pin){
  float dis_vol = (analogRead(pin)/1024.0) *5.0;
  float dis_mm = 155.4 * exp(-2.597*dis_vol)+ 22.67*exp (-0.379*dis_vol); 
  if ( dis_mm >= 80 ){
    dis_mm = 80;
  }
  return (uint16_t)dis_mm;
}


//Motor Motion
void MotorMotion(float Vr, float Vl){

  float kp_left  = (100.0*13.0/(26.0*1000.0));
  float kp_right = (100.0*13.0/(25.9*1000.0));
  
  if(Vl > 0)
  {
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
  }
  else if(Vl < 0)
  {
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
  }
  else
  {
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,LOW);   
  }
  analogWrite(PWMA,kp_left*Vl);
  //analogWrite(PWMA,abs(Vl));
  if(Vr > 0)
  {
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
  }
  else if(Vr < 0)
  {
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,HIGH);
  }
  else
  {
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,LOW);   
  }
  analogWrite(PWMB,kp_right*Vr);
}

float ldr_per(int pin){
   return float(analogRead(LDR)) * 100.0 / 1023.0;
}
