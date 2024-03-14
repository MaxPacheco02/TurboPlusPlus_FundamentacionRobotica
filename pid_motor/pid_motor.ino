#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>  

#include "PID.h"


//MICRO-ROS 
//----------------------------------------------
rcl_publisher_t vel_publisher_handle, error_publisher_handle;
rcl_subscription_t set_subscriber_handle;

std_msgs__msg__Float32 duty_cycle_msg;
std_msgs__msg__Float32 angular_vel_msg, angle_error_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t timer1_handle;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    //TRUN ON A LED OR SOMETHING  IDK
    delay(100);
  }
}

float duty_cycle;
//----------------------------------------------

// Motor pins
#define ENA 15
#define mA1 2
#define mA2 4
#define ENB 16
#define mB1 17
#define mB2 15
// Encode pins
#define ENC1 21
#define ENC2 18


int in = 0; // PWM desired variable
int inP = 0;
double omega_desired = 0; // Omega desired variable
double omega_desired_buffer = 0; // Omega for changing direction
int flag_direction = 0; // Tells current direction 0 +, 1 -

// Encoder variables
volatile int  n    = 0;
volatile byte ant  = 0;
volatile byte act  = 0;

unsigned long lastTime = 0;  
int sampleTime = 100; //Sampling time 

double Pcurr = 0;
double Plast = 0;
double R = 1980;
double v = 0; //Degrees per millis
double omega = 0;//Radians per second


// Initialize PID 
//        kp, ki, kd,minU,maxU,minD,maxD
PID mypid(20, 30, 1, 0, 255, 0, 11, sampleTime);
//Smooth
//20, 10, 0
//Faster
//20, 50, 1


//CALLBACKS 
//-----------------------------------------------------
void subscription_callback(const void * msgin){
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  duty_cycle = msg->data;
}

void timer1_callback(rcl_timer_t * timer, int64_t last_call_time){  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //Calculating Speed
    speed();

    angular_vel_msg.data = omega;
    //Asking user omega 
    omega_desired = 11 * duty_cycle;

    // changeDirection();

    //Changing direction (if omega_desired < 0)
    //Calculating pwm with PID
    in = mypid.pid_controller(abs(omega_desired), abs(omega));

    //Write to motor speed
    analogWrite(ENA, in);

    angular_vel_msg.data = angular_vel_msg.data/11;

    angle_error_msg.data = duty_cycle - angular_vel_msg.data;

    RCSOFTCHECK(rcl_publish(&vel_publisher_handle, &angular_vel_msg, NULL));
    RCSOFTCHECK(rcl_publish(&error_publisher_handle, &angle_error_msg, NULL));
  }
}
//-----------------------------------------------------S


void setup() {
	in = 0; // Setting a starting pwm 

  //PIN SETUP
  //---------------------------------------------------------------------------
	//Motor pins
	pinMode(mA1, OUTPUT);
	pinMode(mA2, OUTPUT);
	pinMode(mB1, OUTPUT);
	pinMode(mB2, OUTPUT);
	//Encode pins
	pinMode(ENC1, INPUT);
	pinMode(ENC2, INPUT);
	//Attaching interrupts for Encoder
	attachInterrupt(digitalPinToInterrupt(ENC1), encoder, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENC2), encoder, CHANGE);

	// Setting up of directions
	digitalWrite(mA1, LOW);
	digitalWrite(mA2, HIGH);
  //---------------------------------------------------------------------------


  //MICRO-ROS SECTION 
  //---------------------------------------------------------------------------
  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  //CREATE NODE
  RCCHECK(rclc_node_init_default(&node, "controller", "", &support));

  //CREATE PUBLISHERS
  RCCHECK(rclc_publisher_init_default(
  &vel_publisher_handle,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
  "duty_cycle"));

  RCCHECK(rclc_publisher_init_default(
  &error_publisher_handle,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
  "angle_error"));

  //CREATE SUBSCRIBERS
  RCCHECK(rclc_subscription_init_default(
  &set_subscriber_handle,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
  "setpoint"));

  //CREATE TIMERS (FOR TIMER CALLBACKS)
  const unsigned int timer_1 = 10;
  RCCHECK(rclc_timer_init_default(
  &timer1_handle,
  &support,
  RCL_MS_TO_NS(timer_1),
  timer1_callback));

  //CREATE EXECUTOR
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer1_handle));
  RCCHECK(rclc_executor_add_subscription(&executor, &set_subscriber_handle, &duty_cycle_msg, &subscription_callback, ON_NEW_DATA));

  duty_cycle_msg.data = 0;
  //---------------------------------------------------------------------------
}

void loop() {
  // put your main code here, to run repeatedly:
	delay(20);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  changeDirection();

}

// Calculating speed (with encoder)
void speed(){
    if (millis() - lastTime >= sampleTime || lastTime==0){ 
      Pcurr = (n*360.0)/R;
      v = (Pcurr - Plast)/(millis() - lastTime);
      lastTime = millis();
      omega = v * 0.0174533 * 1000; 
      Plast = Pcurr;
    }
}

// Encoder with quadraple precision
void encoder(void){
  ant=act;
  
  if(digitalRead(ENC1)) bitSet(act,1); else bitClear(act,1);            
  if(digitalRead(ENC2)) bitSet(act,0); else bitClear(act,0);
  
  if(ant == 2 && act ==0) n++;
  if(ant == 0 && act ==1) n++;
  if(ant == 3 && act ==2) n++;
  if(ant == 1 && act ==3) n++;
  
  if(ant == 1 && act ==0) n--;
  if(ant == 3 && act ==1) n--;
  if(ant == 0 && act ==2) n--;
  if(ant == 2 && act ==3) n--;    
}


void changeDirection(){
	//To go from positive to negative
  // If omega_desired negative save it in buffer and change omega_desired to 0 (first have to arrive to 0)
  /*
  if(omega_desired < 0.0 and omega > 0){ 
    omega_desired_buffer = omega_desired;
    omega_desired = 0.0;
  }
  // If already at 0 and have omega_desire(_buffer) less than 0
  // change direction and stablish omega_desired as omega_desired_buffer
  // and change flag_direction to 1. Have arrived at intermediate.
  if(omega <= 0.0 and omega_desired_buffer < 0){ 
    digitalWrite(mA1, LOW);
    digitalWrite(mA2, HIGH);
    omega_desired = omega_desired_buffer;
    omega_desired_buffer = 0;
  }
  else if(omega_desired > 0.0 and omega < 0){ 
    omega_desired_buffer = omega_desired;
    omega_desired = 0.0;
  }
  // If already at 0 and have omega_desire(_buffer) less than 0
  // change direction and stablish omega_desired as omega_desired_buffer
  // and change flag_direction to 1. Have arrived at intermediate.
  if(omega >= 0.0 and omega_desired_buffer > 0){ 
    digitalWrite(mA1, HIGH);
    digitalWrite(mA2, LOW);
    omega_desired = omega_desired_buffer;
    omega_desired_buffer = 0;
  }
  */
  
  if(omega_desired < 0){ 
    digitalWrite(mA1, LOW);
    digitalWrite(mA2, HIGH);

  }
  
  if(omega_desired > 0){ 
    digitalWrite(mA1, HIGH);
    digitalWrite(mA2, LOW);
  }
  
}
