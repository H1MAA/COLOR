
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <Timer.h>
#include <PIDF.h>

//test
//tweak_ROS_parameters//

//from ROS//
#define TOPIC_NAME_SPEED_SETPOINTS "speeds"                              // TOPIC FOR SPEED SETPOINTS 
                          
//TO ROS//                                                    
#define TOPIC_NAME_PWM_FEEDBACK "dc_pwm_feedback2"                      //array for sending current pwm top ros (debugging)
#define TOPIC_NAME_VELOCITY_FEEDBACK "dc_velocity_feedback2"            // //array for sending current speed top ros (debugging)

#define BAUD_RATE 9600

// PID settings
#define KP1 7      
#define KI1 1 
#define KD1 0.02

#define KP2 0
#define KI2 0
#define KD2 0

#define MAX_SPEED 25.0
                                          
#define Ts 0.01                                                          
#define PID_PERIOD 10   
#define RADIUS 0.0325                                                        // m

// Base parmeters
#define ENCODER_RESOLUTION 2000                                           // CPR

#define MIN_OUTPUT -255                                                     // PWM
#define MAX_OUTPUT  255                                                      // PWM 


//MOTOR PINS//
#define M1_1 10
#define M1_2 9
#define M1_EN 5

#define M2_1 8
#define M2_2 7
#define M2_EN 6

// Encoder pins
#define ENC_A_M1 26
#define ENC_B_M1 27
#define ENC_A_M2 12
#define ENC_B_M2 13

//DEFINE VARIABLES
double ppr = 2000;
long long counts[2] = { 0, 0 };                            // Array to store counts for two motors

long long lastcounts[2] = { 0, 0 };                             // Array to store last counts for two motors

double speed[2] = { 0, 0 };                                 // Array to store speed for two motors IN RPM

double setpoint[2] = {0, 0 };                              // Array to store setpoint for two motors

double pwm_value[2] = { 0, 0 };                            // Array to store PWM value for two motors

//DEFINE PID
MIA::PIDF velocity_pid[] = {MIA::PIDF(KP1, KI1, KD1, Ts),
                             MIA::PIDF(KP2, KI2, KD2, Ts)};

//DEFINE TIMER 
Timer myTimer;

//DEFINE NODE_HANDLE
ros::NodeHandle   nh;

//DEFINE ROS TOPICS AND PUBLISHERS (TO ROS)
//Feedback topics
std_msgs::Float32MultiArray pwm_feedback_msg;
ros::Publisher pwm_pub(TOPIC_NAME_PWM_FEEDBACK, &pwm_feedback_msg);   //pwm topic

std_msgs::Float32MultiArray velocity_feedback_msg;                     
ros::Publisher velocity_pub(TOPIC_NAME_VELOCITY_FEEDBACK, &velocity_feedback_msg);


//DEFINE CALLBACK FUNCTION FOR ASSIGNING SPEEDS
void callback_speeds(const std_msgs::Float32MultiArray &speeds_msg)
{
  double speed_0=(double)speeds_msg.data[0];      // ASSING FIRST MOTOR SPEED 
  double speed_1=(double)speeds_msg.data[1];      // ASSIGN SECOND MOTOR SPEED
  if(speed_0>MAX_SPEED)
    speed_0=MAX_SPEED;
  else if (speed_0<-MAX_SPEED)                   
    speed_0=-MAX_SPEED;
  if(speed_1>MAX_SPEED)
    speed_1=MAX_SPEED;
  else if (speed_1<-MAX_SPEED)
    speed_1=-MAX_SPEED;
  velocity_pid[0].setpoint(speed_0);  // ASSIGN NEW SPEED AS SET POINT
  velocity_pid[1].setpoint(speed_1);  // ASSIGN NEW SPEED AS SET POINT
}

//DEFINE ROS TOPICS AND SUBSCRIBERS  (FROM ROS)
ros::Subscriber<std_msgs::Float32MultiArray> sub_speeds(
    TOPIC_NAME_SPEED_SETPOINTS,
    &callback_speeds);

void initEncoders();
void initRos();
void debugROS();







// ----------------------------------------------------//SETUP//-------------------------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  init_pid();
  initEncoders();
  initRos();
  myTimer.every(PID_PERIOD, pidRoutine);
}


// ----------------------------------------------------//LOOP//-------------------------------------------------------------------
void loop()
{
  myTimer.update();
  nh.spinOnce();
}


// ----------------------------------------------------//PID_ROUTINE//-------------------------------------------------------------------

void pidRoutine() {
  for (int i = 0; i < 2; i++) {

    speed[i] = (counts[i] - lastcounts[i]) * RADIUS / (ppr * Ts);          // (m/s)

    pwm_value[i] = velocity_pid[i].calculate(speed[i]);

    if (velocity_pid[i].setpoint() == 0) {
      velocity_pid[i].integralAccumulator = 0;
    }

    if (velocity_pid[i].setpoint() == 0 && abs(velocity_pid[i].setpoint() - speed[i]) <= 1)
      pwm_value[i] = 0;

    lastcounts[i] = counts[i];
  }

  analogWrite(M1_EN, pwm_value[0]);
  analogWrite(M2_EN, pwm_value[1]);
}




// ----------------------------------------------------//ENCODER_ISR//-------------------------------------------------------------------

void ISR_encoderPinA_M1()
{
  counts[0] += digitalRead(ENC_A_M1) != digitalRead(ENC_B_M1) ? -1 : 1;
}
void ISR_encoderPinB_M1()
{
//  counts[0] += digitalRead(PIN_ENCODER_A[0]) == digitalRead(PIN_ENCODER_B[0]) ? -1 : 1;
}


void ISR_encoderPinA_M2() 
{
  counts[0] += digitalRead(ENC_A_M1) != digitalRead(ENC_B_M1) ? -1 : 1;
}
void ISR_encoderPinB_M2() 
{
//  counts[0] += digitalRead(PIN_ENCODER_A[0]) == digitalRead(PIN_ENCODER_B[0]) ? -1 : 1;
}







//----------------------------------------------------//INITIALIZATION DUNCTIONS//-------------------------------------------------------------------

void init_pid() {
  for (int i = 0; i < 2; i++) {
    velocity_pid[i].limitOutput(MIN_OUTPUT, MAX_OUTPUT);
    velocity_pid[i].setpoint(setpoint[i]);
  }
}

void initEncoders()
{
    // Motor1 Encoder
  pinMode(ENC_A_M1, INPUT_PULLUP);
  pinMode(ENC_B_M1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_M1), ISR_encoderPinA_M1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_M1), ISR_encoderPinB_M1, CHANGE);

    //MOTOR2 ENCODER
  pinMode(ENC_A_M2, INPUT_PULLUP);
  pinMode(ENC_B_M2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_M2), ISR_encoderPinA_M2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_M2), ISR_encoderPinB_M2, CHANGE);
}


void initMotorDrivers()
{
    
  pinMode(M1_1, OUTPUT);
  pinMode(M1_2, OUTPUT);
  pinMode(M1_EN, OUTPUT);

  pinMode(M2_1, OUTPUT);
  pinMode(M2_2, OUTPUT);
  pinMode(M2_EN, OUTPUT);

  digitalWrite(M1_1, LOW);
  digitalWrite(M1_2, LOW);
  digitalWrite(M2_1, LOW);
  digitalWrite(M2_2, LOW);
}


void initRos()
{
   
    nh.initNode();                                      //      INITIALIZE NODE
    nh.advertise(pwm_pub);                              //ADVERTISE
    nh.advertise(velocity_pub);
    // nh.advertise(encoders_0);
    // nh.advertise(encoders_1);
    // nh.advertise(c_0);
    // nh.advertise(c_1);
    nh.subscribe(sub_speeds);                           //SUBSCRIBE
    // nh.subscribe(params);
    
    // ROS Float32MultiArray msg setup
    char dim0_label[] = "PWM";
    pwm_feedback_msg.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    pwm_feedback_msg.layout.dim[0].label = dim0_label;
    pwm_feedback_msg.layout.dim[0].size = 2;
    pwm_feedback_msg.layout.dim[0].stride = 1*2;
    pwm_feedback_msg.data = (float *)malloc(sizeof(float)*2);
    pwm_feedback_msg.layout.dim_length = 0;
    pwm_feedback_msg.data_length = 2;

    char dim1_label[] = "Velocity";
    velocity_feedback_msg.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    velocity_feedback_msg.layout.dim[0].label = dim1_label;
    velocity_feedback_msg.layout.dim[0].size = 2;
    velocity_feedback_msg.layout.dim[0].stride = 1*2;
    velocity_feedback_msg.data = (float *)malloc(sizeof(float)*2);
    velocity_feedback_msg.layout.dim_length = 0;
    velocity_feedback_msg.data_length = 2;
}





//----------------------------------------------------//FEED_BACK (DEBUGGING)//-------------------------------------------------------------------

void debugROS()
{
    // Feedback from PID output
    pwm_feedback_msg.data[0] = (float)pwm_value[0];
    pwm_feedback_msg.data[1] = (float)pwm_value[1];
    pwm_pub.publish(&pwm_feedback_msg); 

    // Feedback from encoder
    velocity_feedback_msg.data[0] = float(speed[0]);      // Measured velocity in RPM
    velocity_feedback_msg.data[1] = float(speed[1]);      // Measured velocity in RPM
    velocity_pub.publish(&velocity_feedback_msg); 
}
