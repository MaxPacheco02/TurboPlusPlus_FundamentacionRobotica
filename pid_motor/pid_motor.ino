// Motor pins
#define ENA 15
#define mA1 2
#define mA2 4
#define ENB 16
#define mB1 17
#define mB2 15
// Encode pins
#define ENC1 2
#define ENC2 4

// PWM variable
int in;
// Encoder variables
int counter = 0;
int state;
int lastState;

void setup() {
	Serial.begin(9600);
	in = 0;

	//Motor pins
	pinMode(mA1, OUTPUT);
	pinMode(mA2, OUTPUT);
	pinMode(mB1, OUTPUT);
	pinMode(mB2, OUTPUT);
	//Encode pins
	pinMode(ENC1, INPUT);
	pinMode(ENC2, INPUT);

	// Setting up of directions
	digitalWrite(mA1, HIGH);
	digitalWrite(mA2, LOW);
	digitalWrite(mB1, HIGH);
	digitalWrite(mB2, LOW);

	// Reads initial state
	lastState = digitalRead(ENC1);

	Serial.println("Begin... ");

}

void loop() {
	//Motor speed
	analogWrite(ENA, 255);
//	analogWrite(ENB, 255);


	delay(100);

	if(Serial.available() > 0){
		in = Serial.parseInt();
//		Serial.println(in);
	}

	//Saturating input
	in = (in <= 0) ? 0 : in;
	in = (in >= 255) ? 255 : in;

	//Reading encoder
	speed();
	Serial.println(counter);
}

void speed(){
	state = digitalRead(ENC1);
	if(state != lastState) {
		if(digitalRead(ENC2) != state){
			counter++;
		}else{
			counter--;
		}
	}
	lastState = state;
}
