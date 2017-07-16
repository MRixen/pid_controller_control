#include <Wire.h>
#include "DEFINITIONS.h"
#include "PID.h"
#include <EEPROM.h>

double pid_control_value = 0;
bool reset_eeprom_manual;

void setup()
{
	// Configure serial interface
	Serial.begin(9600);

	// Configure program data
	firstStart = true;
	eeprom_init_state_ok = true;


	// USER CONFIGURATION
	debugMode = true;
	reset_eeprom_manual = false;

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
	save_action = 0;

	// init eeprom settings
	resetEeprom();
	restoreLastConfig();

	// Attach interrupt for encoder
	attachInterrupt(digitalPinToInterrupt(di_encoderPinA), doEncoderA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(di_encoderPinB), doEncoderB, CHANGE);

	// Configure I2C Bus
	//Wire.begin(); // join i2c bus (address optional for master)
	Wire.begin(I2C_ID_PID_CONTROLLER);
	Wire.onReceive(receiveEvent); // register event
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

void receiveEvent(int numBytes) {
	// Receive 3 bytes (1. angle, 2. save-action, 3. speed)
	for (size_t i = 0; i < numBytes; i++) i2c_data_in[i] = Wire.read();

	save_action = i2c_data_in[1];


	switch (save_action)
	{
	case 0:
		soll_motor_angle_temp = i2c_data_in[0];
		soll_motorSpeed = i2c_data_in[2];

		// Make it a negative value when direction input value is set to zero
		if (digitalRead(di_motorDirection1) == LOW) soll_motor_angle_temp = soll_motor_angle_temp*(-1);
		lockAction = false;
		break;
	case 1:
		if (!lockAction)
		{
			lockAction = true;

			if (current_motor_angle < 0) {
				EEPROM.update(eeprom_addr_ref_pos, current_motor_angle*(-1));
				EEPROM.update(eeprom_addr_ref_pos_sign, 0);
			}
			else {
				EEPROM.update(eeprom_addr_ref_pos, current_motor_angle);
				EEPROM.update(eeprom_addr_ref_pos_sign, 1);
			}
		}
		break;
	case 2:
		if (!lockAction)
		{
			lockAction = true;
			Serial.print("current_motor_angle: ");
			Serial.println(current_motor_angle);

			if (current_motor_angle < 0) {
				EEPROM.update(eeprom_addr_act_pos, current_motor_angle*(-1));
				EEPROM.update(eeprom_addr_act_pos_sign, 0);
			}
			else {
				EEPROM.update(eeprom_addr_act_pos, current_motor_angle);
				EEPROM.update(eeprom_addr_act_pos_sign, 1);
			}
		}
		break;
	default:
		break;
	}
}


void loop()
{
	// Convert encoder value to degree and set it as output for pid_can_bus_controller
	current_motor_angle = (encoderValue*ENCODER_TO_DEGREE) + last_motor_angle;


	// Calculate error term (soll - ist)
	pid_error = current_motor_angle - (soll_motor_angle_temp + ref_pos);
	if (abs(pid_error) <= MIN_PID_ERROR) pid_error = 0;

	// Calculate output for motor
	pid_control_value = pidController(pid_error);

	// Set control value to zero when error term is zero
	if (pid_error == 0) pid_control_value = 0;

	if (pid_control_value < 0) {
		Serial.println(pid_control_value);

		pid_control_value = pid_control_value*(-1);
		digitalWrite(do_motorDirection1, 0);
	}
	else {
		digitalWrite(do_motorDirection1, 1);
	}

	pid_control_value = ((pid_control_value/100)*soll_motorSpeed);

	if (digitalRead(di_enableController)) {
		// Send pid value to monitoring device
		//Serial.print("pid_control_value: ");
		//Serial.println((int)pid_control_value);

		Wire.beginTransmission(I2C_ID_MONITOR);
		Wire.write((int)pid_control_value);
		Wire.endTransmission();
		//Serial.println(current_motor_angle);
		//Serial.println(digitalRead(do_motorDirection1));

	}
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

void restoreLastConfig() {
	// Read last encoder value from eeprom
	last_motor_angle = EEPROM.read(eeprom_addr_act_pos);
	act_pos_sign = EEPROM.read(eeprom_addr_act_pos_sign);

	// Set sign to act pos value
	if (act_pos_sign == 0) last_motor_angle = last_motor_angle*(-1);

	if (debugMode) {
		Serial.print("act_pos last: ");
		Serial.println(last_motor_angle);
		Serial.print("act_pos_sign last: ");
		Serial.println(act_pos_sign);
	}

	// Read ref pos value and ref pos sign from eeprom
	ref_pos = EEPROM.read(eeprom_addr_ref_pos);
	ref_pos_sign = EEPROM.read(eeprom_addr_ref_pos_sign);

	// Set sign to ref pos value
	if (ref_pos_sign == 0) ref_pos = ref_pos*(-1);

	if (debugMode) {
		Serial.print("ref_pos last: ");
		Serial.println(ref_pos);
		Serial.print("ref_pos_sign last: ");
		Serial.println(ref_pos_sign);
	}

}

void resetEeprom() {
	// Reset eeprom at first start
	if ((EEPROM.read(5) != 0) | reset_eeprom_manual) // At first start this all eeprom bytes are 255
	{
		Serial.println("Reset eeprom.");

		// Write zeros
		for (size_t i = 0; i < 6; i++) EEPROM.write(i, 0);  //EEPROM.update(i, 0);

		// Check written data
		for (size_t i = 0; i < 6; i++) if (EEPROM.read(i) != 0) eeprom_init_state_ok = false;

		// Show write state as blink code
		while (!eeprom_init_state_ok) blinkErrorCode(error_eeprom_reset, true, false);

		// Stop program execution
		Serial.println("Ready reset eeprom.");
	}

	// Check error on eeprom writing process (ref or act position)
	int value_eeprom = EEPROM.read(eeprom_addr_error);
	if (value_eeprom != error_ok) {
		// WHen there is an error stop the following operations and set the specific error code
		while (true) blinkErrorCode(value_eeprom, true, false);
	}
}


void blinkErrorCode(int error, int waitBefore, int waitAfter) {
	// Reset led state and wait some time
	if (waitBefore) setLedState(1000, 0, 0, LED_BUILTIN, LOW);

	// Set specific error code
	if (error = error_eeprom_act_pos)
	{
		setLedState(500, 0, 500, LED_BUILTIN, HIGH);
		for (size_t i = 0; i < 2; i++) setLedState(200, 0, 200, LED_BUILTIN, HIGH);
	}

	// Set specific error code
	if (error = error_eeprom_ref_pos)
	{
		setLedState(500, 0, 500, LED_BUILTIN, HIGH);
		for (size_t i = 0; i < 3; i++) setLedState(200, 0, 200, LED_BUILTIN, HIGH);
	}

	// Set specific error code
	if (error = error_eeprom_reset)
	{
		setLedState(500, 0, 500, LED_BUILTIN, HIGH);
		for (size_t i = 0; i < 4; i++) setLedState(200, 0, 200, LED_BUILTIN, HIGH);
	}

	// Reset led state and wait some time
	if (waitAfter) setLedState(1000, 0, 0, LED_BUILTIN, LOW);
}

void setLedState(int blinkLength, int pauseBefore, int pauseAfter, int led, int state) {
	delay(pauseBefore);
	digitalWrite(led, state);
	delay(blinkLength);
	digitalWrite(led, !state);
	delay(pauseAfter);
}