#define __STM32F1__
/*************************************************************************
 * M.I.A ROBOTICS TEAM
 * Slave code for controlling velocities of 2 DC motors
 * Closed loop control system is applied
 * Controller   --> PID
 * Feedback     --> Encoder
 * Actuator     --> DC motor
 * Motor driver --> Cytron
 * Communications is done with ROS.. the slave subscribes the velocities
 * then makes the PID calculations on it with respect of the feedback
 * then for debuging slave publishes feedback of pwm & measured velocities.
 **************************************************************************/
// Including libraries
#define __STM32F1__


#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
//#include <std_msgs/Float64MultiArray.h>
//#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <Timer.h>
#include <PIDF.h>

// ROS Settings
#define TOPIC_NAME_SETPOINTS "speeds"
#define TOPIC_NAME_PWM_FEEDBACK "dc_pwm_feedback2"
#define TOPIC_NAME_VELOCITY_FEEDBACK "dc_velocity_feedback2"

#define MAX_PUBLISHERS 10
#define MAX_SUBSCRIBERS 10
#define PUBLISHERS_BUFFER_SIZE 2048
#define SUBSCRIBERS_BUFFER_SIZE 2048

#define BAUD_RATE 57600

// PID settings
#define KP1 700.0 //9625.16806 //1025.2801 //900 //22525.0037 //12v -> 1100,3500,0.8,1100,3500,1.2  //24v -> 500,1600,1,800,1600,1 //500,1600,2,800,1600,2.2,400,1000,2.2,400,1100,2
#define KI1 45000.0 //565485.06348 //15100.9274 //17000 //1361061.2676
#define KD1 1.2 //-62.858924 //50.2999 //35 //-137.0774
#define KP2 700
#define KI2 45000.0
#define KD2 1.2

#define MAX_SPEED 25.0


#define DEADZONE  3      // When setpoint arround Deadzone integral will become zero   (rad/s)
#define FREQUENCY 10000  // Frequency of PWM (HZ)

//#define KN //64.19764 //38.0432 //103.4293
#define Ts 0.01
#define PID_PERIOD 10   // ms
//#define sampling_time 20

// Base parmeters
#define ENCODER_RESOLUTION 540  // CPR
#define RADIUS 5              // d = 15 cm
#define MIN_OUTPUT -65000       // PWM
#define MAX_OUTPUT  65000       // PWM 

// Encoder pins
const int PIN_ENCODER_A[] = {PA6, PA1};  //encoder1(a) bach left,encoder2 back right(a)
const int PIN_ENCODER_B[] = {PA7, PA2};  //encoder1(b) bach left,encoder2 back right(b)
// Motor pins
const int PIN_MOTOR_DIR[] = {PB9, PB7};   //cytron0(direction),cytron1(direction)
const int PIN_MOTOR_PWM[] = {PB8, PB6};   //cytron0(pwm),cytron1(pwm)

// Define TTL pins
#define PIN_UART1_RX PA10
#define PIN_UART1_TX PA9
#define PIN_UART3_RX PB11
#define PIN_UART3_TX PB10
//HardwareSerial Serial1(PIN_UART1_RX, PIN_UART1_TX);
//HardwareSerial Serial3(PIN_UART3_RX, PIN_UART3_TX);

#define SERIAL_DEBUG Serial3
#define SERIAL_ROS Serial1

MIA::PIDF velocity_pid[] = {MIA::PIDF(KP1, KI1, KD1, 0, Ts), 
                            MIA::PIDF(KP2, KI2, KD2, 0, Ts)};

Timer myTimer;

// Variables
double sp[] {0,0}, 
       speeds[] {0,0};

long long     pwm    [] {0,0},
              counter  [] {0,0}, 
              lastCounter[] {0,0};

ros::NodeHandle_<ArduinoHardware,
                 MAX_PUBLISHERS,
                 MAX_SUBSCRIBERS,
                 PUBLISHERS_BUFFER_SIZE,
                 SUBSCRIBERS_BUFFER_SIZE>
    nh;
    
// Feedback topics
std_msgs::Float32MultiArray pwm_feedback_msg;
ros::Publisher pwm_pub(TOPIC_NAME_PWM_FEEDBACK, &pwm_feedback_msg);   //pwm topic

std_msgs::Float32MultiArray velocity_feedback_msg;
ros::Publisher velocity_pub(TOPIC_NAME_VELOCITY_FEEDBACK, &velocity_feedback_msg);

// Function to call Speeds from output matrics data from master
void callback_speeds(const std_msgs::Float32MultiArray &speeds_msg)
{
  double speed_0=(double)speeds_msg.data[0];
  double speed_1=(double)speeds_msg.data[1];
  if(speed_0>MAX_SPEED)
    speed_0=MAX_SPEED;
  else if (speed_0<-MAX_SPEED)
    speed_0=-MAX_SPEED;
  if(speed_1>MAX_SPEED)
    speed_1=MAX_SPEED;
  else if (speed_1<-MAX_SPEED)
    speed_1=-MAX_SPEED;
  velocity_pid[0].setpoint(speed_0);  // Get setpoint from output matrix in master
  velocity_pid[1].setpoint(speed_1);  // Get setpoint from output matrix in master
}

ros::Subscriber<std_msgs::Float32MultiArray> sub_speeds(
    TOPIC_NAME_SETPOINTS,
    &callback_speeds);  // Subscriber to subscribe from Master
    
void callback_parameters(const std_msgs:: Float32MultiArray& parameters)    //parameters method
{
  velocity_pid[0].tune(parameters.data[0],parameters.data[1],parameters.data[2]);
  velocity_pid[1].tune(parameters.data[3],parameters.data[4],parameters.data[5]);
}

ros::Subscriber<std_msgs::Float32MultiArray> params("parameters", &callback_parameters);   //parameters topic

//controlled velocity(0) topic 
std_msgs::Float64 vel_0;
ros::Publisher encoders_0("encoders_0", &vel_0);
//controlled velocity(1) topic 
std_msgs::Float64 vel_1;
ros::Publisher encoders_1("encoders_1", &vel_1);

//encoders _counts

std_msgs::Int16 c_0msg;
ros::Publisher c_0("c_0", &c_0msg);

std_msgs::Int16 c_1msg;
ros::Publisher c_1("c_1", &c_1msg);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
 // SERIAL_ROS.begin(BAUD_RATE);
  Serial1.begin(BAUD_RATE);

  // PWM setup
  analogWriteFrequency(FREQUENCY);
  analogWriteResolution(16);

  initRos();
  initEncoders();
  initMotorDrivers();
  initPid();
 
  //  Start scheduler and LED indication
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW);
  //digitalWrite(LED_BUILTIN, HIGH);

  myTimer.every(PID_PERIOD, velocityPidRoutine);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
  myTimer.update();
  nh.spinOnce();
}

// ================================================================
// ===                   CLOSED LOOP CONTROL                    ===
// ================================================================



void velocityPidRoutine(void)
{
    // main routine, measures velocity, passes velocity to the PID then applies
    // the output to the motors

    // Motor1
    speeds[0] = (((1.0*(counter[0]-lastCounter[0])/ENCODER_RESOLUTION)*2*PI))/(1.0*(PID_PERIOD/1000.0));   //speed(0) (radian/s)
    pwm[0]       = velocity_pid[0].calculate(speeds[0]);    

    if(velocity_pid[0].setpoint()==0){
      velocity_pid[0].integralAccumulator = 0;
    }

    if (velocity_pid[0].setpoint()==0 && abs(velocity_pid[0].setpoint()-speeds[0])<=DEADZONE)
        pwm[0]=0;
    else if (pwm[0] > 0)
        digitalWrite(PIN_MOTOR_DIR[0], HIGH);
    else if (pwm[0] < 0)
        digitalWrite(PIN_MOTOR_DIR[0], LOW);
    analogWrite(PIN_MOTOR_PWM[0], abs(pwm[0]));
    lastCounter[0] = counter[0];
    

    // Motor2
    speeds[1] = (((1.0*(counter[1]-lastCounter[1])/ENCODER_RESOLUTION)*2*PI))/(1.0*(PID_PERIOD/1000.0));   //speed(1) (radian/s)
    pwm[1]       = velocity_pid[1].calculate(speeds[1]);

    if(velocity_pid[1].setpoint()==0){
      velocity_pid[1].integralAccumulator = 0;
    }

    if (velocity_pid[1].setpoint()==0 && abs(velocity_pid[1].setpoint()-speeds[1])<=DEADZONE)
        pwm[1]=0;
    else if (pwm[1] > 0)
        digitalWrite(PIN_MOTOR_DIR[1], HIGH);
    else if (pwm[1] < 0)
        digitalWrite(PIN_MOTOR_DIR[1], LOW);
        
    analogWrite(PIN_MOTOR_PWM[1], abs(pwm[1]));
    lastCounter[1] = counter[1];
    
    vel_0.data=(double)speeds[0];
    vel_1.data=(double)speeds[1];

    encoders_0.publish(&vel_0);
    encoders_1.publish(&vel_1);
  
  //PUBLISH COUNTS
    c_0msg.data=(double)counter[0];
    c_1msg.data=(double)counter[1];
  
    c_0.publish(&c_0msg);
    c_1.publish(&c_1msg);

    // Debugging
    debugROS();
//    debugSERIAL();
}

// ================================================================
// ===               INTERRUPT SERVICE ROUTINE                ===
// ================================================================

void encoderISR_A0(void)
{
  counter[0] += digitalRead(PIN_ENCODER_A[0]) == digitalRead(PIN_ENCODER_B[0]) ? -1 : 1;
}
void encoderISR_B0(void)
{
  counter[0] += digitalRead(PIN_ENCODER_A[0]) != digitalRead(PIN_ENCODER_B[0]) ? -1 : 1;
}

void encoderISR_A1(void)
{
  counter[1] += digitalRead(PIN_ENCODER_A[1]) != digitalRead(PIN_ENCODER_B[1]) ? -1 : 1;
}
void encoderISR_B1(void)
{
  counter[1] += digitalRead(PIN_ENCODER_A[1]) == digitalRead(PIN_ENCODER_B[1]) ? -1 : 1;
}

// ================================================================
// ===                  INITIALLIZING FUNCTIONS                 ===
// ================================================================

void initRos()
{
    //(nh.getHardware())->setPort(&Serial1);
    //(nh.getHardware())->setBaud(BAUD_RATE);

//    (nh.getHardware())->setPort(&Serial1);
//    (nh.getHardware())->setBaud(BAUD_RATE);
    nh.initNode();
    nh.advertise(pwm_pub);
    nh.advertise(velocity_pub);
    nh.advertise(encoders_0);
    nh.advertise(encoders_1);
    nh.advertise(c_0);
    nh.advertise(c_1);
    nh.subscribe(sub_speeds);
    nh.subscribe(params);
    
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

void initEncoders()
{
    // Motor1 Encoder
    pinMode(PIN_ENCODER_A[0], INPUT_PULLUP);
    pinMode(PIN_ENCODER_B[0], INPUT_PULLUP);
    attachInterrupt(PIN_ENCODER_A[0], encoderISR_A0, CHANGE);
    attachInterrupt(PIN_ENCODER_B[0], encoderISR_B0, CHANGE);

    // Motor2 Encoder
    pinMode(PIN_ENCODER_A[1], INPUT_PULLUP);
    pinMode(PIN_ENCODER_B[1], INPUT_PULLUP);
    attachInterrupt(PIN_ENCODER_A[1], encoderISR_A1, CHANGE);
    attachInterrupt(PIN_ENCODER_B[1], encoderISR_B1, CHANGE);
}

void initMotorDrivers()
{
    // Motor1
    pinMode(PIN_MOTOR_DIR[0], OUTPUT);
    pinMode(PIN_MOTOR_PWM[0], OUTPUT);
    digitalWrite(PIN_MOTOR_DIR[0], LOW);
    digitalWrite(PIN_MOTOR_PWM[0], LOW);
    
    // Motor2
    pinMode(PIN_MOTOR_DIR[1], OUTPUT);
    pinMode(PIN_MOTOR_PWM[1], OUTPUT);
    digitalWrite(PIN_MOTOR_DIR[1], LOW);
    digitalWrite(PIN_MOTOR_PWM[1], LOW);
}

void initPid()
{
    // Motor1
    velocity_pid[0].setpoint(sp[0]);
    velocity_pid[0].limitOutput(MIN_OUTPUT, MAX_OUTPUT);
    
    // Motor2
    velocity_pid[1].setpoint(sp[1]);
    velocity_pid[1].limitOutput(MIN_OUTPUT, MAX_OUTPUT);
}

// ================================================================
// ===                    UNITS CONVERTER                       ===
// ================================================================

double rps2ppms(double rps)  // Revelution per second to pulse per millisecond
{
  return (rps * ENCODER_RESOLUTION) / (1000 * 60 * 60);
}

double ppms2cmps(double pulses_per_ms)  // pulse per millisecond to cm per second
{
  return (pulses_per_ms * 1000.0 / ENCODER_RESOLUTION) * (2 * PI * RADIUS);
}

double ppms2rps(double pulses_per_ms)  // pulse per millisecond to cm per second
{
  return (pulses_per_ms * 1000.0 / ENCODER_RESOLUTION) * (60 * 60);
}

double cmps2ppms(double cm_per_s)  // cm per second to pulse per millisecond
{
  return (cm_per_s / (2 * PI * RADIUS) * ENCODER_RESOLUTION) / 1000;
}

// ================================================================
// ===                        DEBUGING                          ===
// ================================================================

void debugROS()
{
    // Feedback from PID output
    pwm_feedback_msg.data[0] = (float)pwm[0];
    pwm_feedback_msg.data[1] = (float)pwm[1];
    pwm_pub.publish(&pwm_feedback_msg); 

    // Feedback from encoder
    velocity_feedback_msg.data[0] = (float)ppms2cmps(speeds[0]);      // Measured velocity in RPS
    velocity_feedback_msg.data[1] = (float)ppms2cmps(speeds[1]);      // Measured velocity in RPS
    velocity_pub.publish(&velocity_feedback_msg); 
}
/*
void debugSERIAL()
{
    SERIAL_DEBUG.print("PWM1: ");
    SERIAL_DEBUG.print(pwm[0]);
    SERIAL_DEBUG.print(", ");
    SERIAL_DEBUG.print("Velocity1: ");
    SERIAL_DEBUG.print(ppms2cmps(speeds[0]));
    SERIAL_DEBUG.println(" cms");
    SERIAL_DEBUG.print("PWM2: ");
    SERIAL_DEBUG.print(pwm[1]);
    SERIAL_DEBUG.print(", ");
    SERIAL_DEBUG.print("Velocity2: ");
    SERIAL_DEBUG.print(ppms2cmps(speeds[1]));
    SERIAL_DEBUG.println(" cms");
}*/
