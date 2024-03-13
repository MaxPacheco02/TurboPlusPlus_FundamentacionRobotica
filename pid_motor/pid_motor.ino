#include "PID.h"


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
// kp, ki, kd, minU, maxU, minD, maxD
PID mypid(20, 30, 0, 0, 255, 0, 11, sampleTime);
//Smooth
//20, 10, 0
//Faster
//20, 50, 1


void setup() {
	Serial.begin(9600);
	in = 0; // Setting a starting pwm 

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
	digitalWrite(mA1, HIGH);
	digitalWrite(mA2, LOW);
//	digitalWrite(mA1, LOW);
//	digitalWrite(mA2, HIGH);

//	digitalWrite(mB1, HIGH);
//	digitalWrite(mB2, LOW);

	Serial.println("Begin... ");

}

void loop() {

	//Calculating Speed
	speed();
	//Asking user omega 
	input_user_omegaDesired();
	//Changing direction (if omega_desired < 0)
	changeDirection();
	//Calculating pwm with PID
	in = mypid.pid_controller(omega_desired, abs(omega));
	Serial.println(omega);
	//Write to motor speed
	analogWrite(ENA, in);

	delay(100);
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

float input_user_pwm(){
	if(Serial.available() > 0){
		in = Serial.parseInt();
//		Serial.println(in);
	}

	//Saturating input
	in = (in <= 0) ? 0 : in;
	in = (in >= 255) ? 255 : in;
}

void input_user_omegaDesired(){
	if(Serial.available() > 0){
		omega_desired = Serial.parseFloat();
//		Serial.print("Omega_desired: ");
		Serial.println(omega_desired);
	}

}

void changeDirection(){
	// 0+, 1-

	if(flag_direction == 0){
		if(omega_desired < 0.0 and flag_direction == 0){
			omega_desired_buffer = omega_desired;
			omega_desired = 0.0;
		}

		if(omega_desired_buffer < 0.0 and omega <= 1.0){
			digitalWrite(mA1, LOW);
			digitalWrite(mA2, HIGH); 
			flag_direction = 1;
			omega_desired = abs(omega_desired_buffer);
		}
	}

	if(flag_direction == 1){
		if(omega_desired < 0.0) omega_desired = abs(omega_desired);

//		if(omega_desired > 0.0 and flag_direction == 1){
//			omega_desired_buffer = omega_desired;
//			omega_desired = 0.0;
//		}
//	
//		if(omega_desired_buffer > 0.0 and omega >= -1.0){
//			digitalWrite(mA1, HIGH);
//			digitalWrite(mA2, LOW); 
//			flag_direction = 0;
//			omega_desired = omega_desired_buffer;
//		}
	}

}
