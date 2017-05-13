#include <Wire.h>
#include "DEFINITIONS.h"
#include "PID.h"

TwoWire wireSend = TwoWire();
TwoWire wireReceive = TwoWire();

void setup()
{
	// Configure serial interface
	Serial.begin(9600);

	// Configure program data
	firstStart = true;

	// USER CONFIGURATION
	debugMode = true;

	// Define I/Os
	pinMode(di_encoderPinA, INPUT);
	pinMode(di_encoderPinB, INPUT);
	pinMode(di_motorDirection1, INPUT);
	pinMode(di_enableController, INPUT);
	pinMode(do_motorDirection1, OUTPUT);
	pinMode(do_motorDirection2, OUTPUT);
	pinMode(do_motorStandby, OUTPUT);

	// Init I/Os
	digitalWrite(do_motorStandby, HIGH);

	// Init
	soll_motorAngle.data = 0;

	// Attach interrupt for encoder
	attachInterrupt(digitalPinToInterrupt(di_encoderPinA), doEncoderA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(di_encoderPinB), doEncoderB, CHANGE);

	// Configure I2C Bus
	//Wire.begin(); // join i2c bus (address optional for master)
	wireReceive.begin(8);
	wireSend.begin(9);     
	// join i2c bus with address #8
	wireReceive.onReceive(receiveEvent); // register event
	//Wire.onRequest(requestEvent); // register event

	// Give time to set up
	delay(100);

	// Move motor until 60deg to elmininate encoder offset (Only neccessary for premium gear motor (big motor)
	//while (firstStart) {
	//	if (encoderValue < MAX_ENCODER_OFFSET) analogWrite(do_pwm, 50);
	//	else {
	//		analogWrite(do_pwm, 0);
	//		firstStart = false;
	//	}
	//	Serial.print("firstValue: ");
	//	Serial.println(encoderValue*ENCODER_TO_DEGREE);
	//}
}

void receiveEvent(int howMany) {
	// TODO: Expand to 2 bytes
	// Read 1 byte from bus
	soll_motor_angle_temp = Wire.read();
	// Make it a negaitve value when direction input value is set to zero
	if (digitalRead(di_motorDirection1) == LOW) soll_motor_angle_temp = soll_motor_angle_temp*(-1);
	Serial.println(soll_motor_angle_temp);
}

//void requestEvent() {
//	// Send 2 bytes on bus
//	Wire.write((int)current_motor_angle);
//}

void loop()
{
	// Convert encoder value to degree and set it as output vor pid_can_bus_controller
	current_motor_angle = encoderValue*ENCODER_TO_DEGREE;

	// Calculate error term (soll - ist)
	pid_error = current_motor_angle - soll_motor_angle_temp;
	if (abs(pid_error) <= MIN_PID_ERROR) pid_error = 0;

	// Calculate output for motor
	double pid_control_value = pidController(pid_error);

	// Set control value to zero when error term is zero
	if (pid_error == 0) pid_control_value = 0;

	// Configure direction value for motor
	// Direction input: when DIR is high (negative) current will flow from OUTA to OUTB, when it is low current will flow from OUTB to OUTA (positive).
	//if (pid_control_value < 0)
	//{
	//	pid_control_value = pid_control_value*(-1);
	//	digitalWrite(do_motorDirection1, LOW);
	//	digitalWrite(do_motorDirection2, HIGH);
	//}
	//else {
	//	digitalWrite(do_motorDirection1, HIGH);
	//	digitalWrite(do_motorDirection2, LOW);
	//}

	if (pid_control_value < 0) pid_control_value = pid_control_value*(-1);
	digitalWrite(do_motorDirection1, digitalRead(di_motorDirection1));

	// Check if controller is activated
	if (digitalRead(di_enableController)) {
		wireSend.beginTransmission(9);
		wireSend.write((int)pid_control_value); // Send 1 byte
		wireSend.endTransmission();
	}
	//if (digitalRead(di_enableController)) analogWrite(do_pwm, (int)pid_control_value);
}

void doEncoderA() {

	// look for a low-to-high on channel A
	if (digitalRead(di_encoderPinA) == HIGH) {
		// check channel B to see which way encoder is turning
		if (digitalRead(di_encoderPinB) == LOW) {
			encoderValue = encoderValue + 1;         // CW
		}
		else {
			encoderValue = encoderValue - 1;         // CCW
		}
	}
	else   // must be a high-to-low edge on channel A                                       
	{
		// check channel B to see which way encoder is turning  
		if (digitalRead(di_encoderPinB) == HIGH) {
			encoderValue = encoderValue + 1;          // CW
		}
		else {
			encoderValue = encoderValue - 1;          // CCW
		}
	}
}

void doEncoderB() {

	// look for a low-to-high on channel B
	if (digitalRead(di_encoderPinB) == HIGH) {
		// check channel A to see which way encoder is turning
		if (digitalRead(di_encoderPinA) == HIGH) {
			encoderValue = encoderValue + 1;         // CW
		}
		else {
			encoderValue = encoderValue - 1;         // CCW
		}
	}
	// Look for a high-to-low on channel B
	else {
		// check channel B to see which way encoder is turning  
		if (digitalRead(di_encoderPinA) == LOW) {
			encoderValue = encoderValue + 1;          // CW
		}
		else {
			encoderValue = encoderValue - 1;          // CCW
		}
	}
}

