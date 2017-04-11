#include <EEPROM.h>
#include <SPI.h>
#include "DEFINITIONS.h"
#include "MCP2515.h"
#include "PID.h"
#include "DATAPACKAGE.h"

// ------------------------------------
// Controlling motors with id 2
// ------------------------------------
int motorId = 2; // Motor id (its NOT possible to use the same idetifier for two devices in the bus

// TODO: Configure identifier for can bus to receive data for motor 0 only
void setup()
{
	// Configure serial interface
	Serial.begin(9600);

	// Configure program data
	firstStart = true;

	// USER CONFIGURATION
	debugMode = true;
	reset_eeprom = false; // Here you can reset the eeprom (set all values to zero). Neccessary for the first use, because the init values are 255

	// Configure identifier for this motor
	//int motorIdTemp = motorId + 256;
	//for (int i = 0; i < 3; i++)
	//{
	//	REGISTER_TXBxSIDL_VALUE[i] = lowByte(motorIdTemp);
	//	REGISTER_TXBxSIDH_VALUE[i] = highByte(motorIdTemp);
	//}

	// Define I/Os
	pinMode(do_csMcp2515, OUTPUT); // Set as input to enable pull up resistor. It's neccessary because the ss line is defined at pin 10 + 9
	pinMode(do_csArduino, OUTPUT); // Set as input to enable pull up resistor. It's neccessary because the ss line is defined at pin 10 + 9
	pinMode(di_encoderPinA, INPUT);
	pinMode(di_encoderPinB, INPUT);
	pinMode(di_mcp2515_int_rec, INPUT);
	pinMode(do_motorDirection, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(di_powerOn, INPUT);

	// Read input signal for storing actual encoder position
	di_powerOn_state_old = digitalRead(di_powerOn);

	// Reset eeprom if user select it
	bool eeprom_init_state_ok = true;
	if (reset_eeprom)
	{
		// Write zeros
		for (size_t i = 0; i < 5; i++) EEPROM.update(i, 0);

		// Check written data
		for (size_t i = 0; i < 5; i++) if (EEPROM.read(i) != 0) eeprom_init_state_ok = false;

		// Show write state as blink code
		while (!eeprom_init_state_ok) blinkErrorCode(error_eeprom_reset, true, false);

		// Stop program execution
		while (true) delay(500);
	}

	// Check error on eeprom writing process (ref or act position)
	int value_eeprom = EEPROM.read(eeprom_addr_error);
	if (value_eeprom != error_ok) {
		// WHen there is an error stop the following operations and set the specific error code
		while (true) blinkErrorCode(value_eeprom, true, false);
	}

	// Configure SPI
	SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
	SPI.begin();

	// Configure MCP2515
	initMcp2515();

	// Set identifier, message length, etc.
	mcp2515_init_tx_buffer0(REGISTER_TXBxSIDL_VALUE[0], REGISTER_TXBxSIDH_VALUE[0], BYTES_TO_SEND);
	mcp2515_init_tx_buffer1(REGISTER_TXBxSIDL_VALUE[1], REGISTER_TXBxSIDH_VALUE[1], BYTES_TO_SEND);
	mcp2515_init_tx_buffer2(REGISTER_TXBxSIDL_VALUE[2], REGISTER_TXBxSIDH_VALUE[2], BYTES_TO_SEND);

	// Read last encoder value from eeprom
	ist_motorAngle.bytes[0] = EEPROM.read(eeprom_addr_act_pos_1);
	ist_motorAngle.bytes[1] = EEPROM.read(eeprom_addr_act_pos_2);
	encoderValue = ist_motorAngle.data;

	// TODO: Validate act pos value (maybe flip the bytes)
	if (debugMode) {
		Serial.print("encoderValue last: ");
		Serial.println(encoderValue);
	}

	// Read ref pos value from eeprom
	ref_pos.bytes[0] = EEPROM.read(eeprom_addr_ref_pos_1);
	ref_pos.bytes[1] = EEPROM.read(eeprom_addr_ref_pos_2);

	// TODO: Validate ref pos value (maybe flip the bytes)
	if (debugMode) {
		Serial.print("ref_pos.data last: ");
		Serial.println(ref_pos.data);
	}

	// Init
	soll_motorAngle.data = 0;

	// Attach interrupt for encoder
	attachInterrupt(digitalPinToInterrupt(di_encoderPinA), doEncoderA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(di_encoderPinB), doEncoderB, CHANGE);

	// Give time to set up
	delay(100);

	// Start timer to measure the program execution
	errorTimerValue = millis();

	// TODO:
	// Turn green led on to show that everything was inititalized and maybe ok


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

void loop()
{
	// TODO:
	// Cancel pending operations

	rxStateIst = 0x00;
	pwmValueTemp = 0;
	motorIsActive = true;

	// Check if message is received in buffer 0 or 1
	if ((digitalRead(di_mcp2515_int_rec) == 0))
	{
		// Get current rx buffer
		rxStateIst = mcp2515_execute_read_state_command(do_csMcp2515);

		// Read incoming data package
		receiveData(rxStateIst, rxStateSoll);

		// Wait after receive command
		delay((SAMPLE_TIME / 2) * 1000);
	}

	Serial.println(incoming_data[in_motorId]);

	// Do something when the motor id is right
	if (incoming_data[in_motorId] == motorId) {
		//if (true) {
		if ((incoming_data[in_action] == action_nothingToDo)) {
			lockAction = false;
			for (size_t i = 0; i < BYTES_TO_SEND; i++) outgoing_data[i] = 0;
			// Send motor id every time as alive flag
			outgoing_data[1] = 0;
		}

		// Check incoming action and set outgoing package
		if ((!lockAction) & ((incoming_data[in_action] == action_saveToEeprom) | (incoming_data[in_action] == action_disablePidController) | (incoming_data[in_action] == action_enablePidController) | (incoming_data[in_action] == action_newPosition)))
		{
			outgoing_data[out_action] = incoming_data[in_action];
			outgoing_data[out_actionState] = state_pending;
			outgoing_data[out_motorId] = incoming_data[in_motorId];
			Serial.println("package");
		}

		// Send package back to client
		sendData(BYTES_TO_SEND, outgoing_data);

		// Wait after send command
		delay((SAMPLE_TIME / 2) * 1000);

		// Work on action <saveToEeprom>
		if ((!lockAction) & (incoming_data[in_action] == action_saveToEeprom))
		{
			lockAction = true;
			// Write actual motor position as ref pos to eeprom (byte 0 and 1)
			EEPROM.update(eeprom_addr_ref_pos_1, lowByte(encoderValue));
			EEPROM.update(eeprom_addr_ref_pos_2, highByte(encoderValue));

			// Validate writing and reset state if position is written to eeprom
			if ((EEPROM.read(eeprom_addr_ref_pos_1) == lowByte(encoderValue)) & (EEPROM.read(eeprom_addr_ref_pos_2) == highByte(encoderValue))) {

				// Save ref pos value to local variable
				ref_pos.bytes[0] = lowByte(encoderValue);
				ref_pos.bytes[1] = highByte(encoderValue);

				// TODO: Validate ref pos value (maybe flip the bytes)
				if (debugMode)
				{
					Serial.print("ref_pos.data new: ");
					Serial.println(ref_pos.data);
				}

				outgoing_data[out_actionState] = state_complete;
				EEPROM.update(eeprom_addr_error, error_ok);
				Serial.println("eeprom");
			}
			else EEPROM.update(eeprom_addr_error, error_eeprom_ref_pos); // If there is an error the pending state will never change to complete 
		}

		// Work on action <disablePidController>
		if ((!lockAction) & (incoming_data[in_action] == action_disablePidController))
		{
			lockAction = true;
			pid_controller_enabled = false;
			outgoing_data[out_actionState] = state_complete;
			Serial.println("disablePidController");
		}

		// Work on action <enablePidController>
		if ((!lockAction) & (incoming_data[in_action] == action_enablePidController))
		{
			lockAction = true;
			pid_controller_enabled = true;
			outgoing_data[out_actionState] = state_complete;
			Serial.println("enablePidController");
		}

		// Work on action <newPosition>
		if ((!lockAction) & (incoming_data[in_action] == action_newPosition))
		{
			lockAction = true;
			// Convert byte to short 
			soll_motorAngle.bytes[0] = incoming_data[in_angle_2];
			soll_motorAngle.bytes[1] = incoming_data[in_angle_1];
			soll_motor_angle_temp = soll_motorAngle.data;

			if (incoming_data[in_motorDir] == 0) soll_motor_angle_temp = soll_motor_angle_temp*(-1);
			else soll_motor_angle_temp = soll_motor_angle_temp;
			Serial.println("newPosition");
		}

			//Serial.println(pid_controller_enabled);

		// Controll motor with pid
		if (pid_controller_enabled)
		{
			// Convert encoder value to degree
			current_motor_angle = encoderValue*ENCODER_TO_DEGREE;

			// Add ref pos value to soll motor angle 
			soll_motor_angle_temp = soll_motorAngle.data + ref_pos.data;

			// Calculate error term (soll - ist)
			pid_error = current_motor_angle - soll_motor_angle_temp;
			if (abs(pid_error) <= MIN_PID_ERROR) pid_error = 0;

			// Calculate output for motor
			double pid_control_value = pidController(pid_error);

			// Set value to zero to send complete state
			if (pid_error == 0) pid_control_value = 0;

			// Configure direction value for motor
			// Direction input: when DIR is high (negative) current will flow from OUTA to OUTB, when it is low current will flow from OUTB to OUTA (positive).
			if (pid_control_value < 0) {
				digitalWrite(do_motorDirection, LOW);
				pid_control_value = pid_control_value*(-1);
			}
			else digitalWrite(do_motorDirection, HIGH);

			// Limit the motor angle to prevent mechanical damage
			if ((soll_motorAngle.data <= MAX_MOTOR_ANGLE) & (soll_motorAngle.data >= MIN_MOTOR_ANGLE)) {
				analogWrite(do_pwm, (int)pid_control_value);
				posOutReached = false;
			}
			else {
				analogWrite(do_pwm, 0);
				posOutReached = true;
			}

			//TEST
			Serial.print("pid_control_value: ");
			Serial.println(pid_control_value);
			//TEST

			//TEST
			Serial.print("pid_error: ");
			Serial.println(pid_error);
			//TEST

			//TEST
			Serial.print("outgoing_data[out_actionState]: ");
			Serial.println(outgoing_data[out_actionState]);
			//TEST

			//TEST
			Serial.print("incoming_data[in_action]: ");
			Serial.println(incoming_data[in_action]);
			//TEST

			//TEST
			Serial.print("posOutReached: ");
			Serial.println(posOutReached);
			//TEST

			// Reset state if position is reached or not reached because of mechanical limit
			// TODO: Send different state (state eerror stop mechanic...)
			if (((pid_control_value == 0) | (posOutReached)) & (incoming_data[in_action] == action_newPosition)) {
				outgoing_data[out_actionState] = state_complete;
				posOutReached = false;
				//TEST
				Serial.print("outgoing_data[out_actionState]: ");
				Serial.println(outgoing_data[out_actionState]);
				//TEST
			}

			//TEST
			Serial.print("outgoing_data[out_actionState]_2: ");
			Serial.println(outgoing_data[out_actionState]);
			//TEST
		}


		// Store actual encoder value to eeprom when raspberry pi is off or if someone reset the input signal
		// Arduino maybe gets the rest of power from a condensator
		// Falling edge of di_powerOn
		int di_powerOn_state = digitalRead(di_powerOn);
		if (di_powerOn_state_old & !di_powerOn_state) {
			//di_powerOn_state_old = di_powerOn_state;
			//// Write actual motor position to eeprom
			//EEPROM.update(eeprom_addr_act_pos_1, lowByte(encoderValue));
			//EEPROM.update(eeprom_addr_act_pos_2, highByte(encoderValue));

			//ist_motorAngle.bytes[0] = EEPROM.read(eeprom_addr_act_pos_1);
			//ist_motorAngle.bytes[1] = EEPROM.read(eeprom_addr_act_pos_2);
			//encoderValue = ist_motorAngle.data;
			//// TODO
			//// Validate act pos value (maybe flip the bytes)
			//Serial.print("encoderValue di_powerOn: ");
			//Serial.println(encoderValue);

			//// Validate writing 
			//// Store ok byte to eeprom (and read this at startup to validate)
			//if ((EEPROM.read(eeprom_addr_act_pos_1) == lowByte(encoderValue)) & (EEPROM.read(eeprom_addr_act_pos_2) == highByte(encoderValue))) EEPROM.update(eeprom_addr_error, error_ok);
			//else EEPROM.update(eeprom_addr_error, error_eeprom_act_pos);

		}
		else if (di_powerOn_state) di_powerOn_state_old = true;

		//Serial.println(outgoing_data[out_actionState]);
		//Serial.println(encoderValue);
		//Serial.println(soll_motor_angle);
	}
}

bool receiveData(byte rxStateIst, byte rxStateSoll)
{
	if ((rxStateIst & rxStateSoll) == 1) readControlValue(SPI_INSTRUCTION_READ_RX_BUFFER0, do_csMcp2515);
	else if ((rxStateIst & rxStateSoll) == 2) readControlValue(SPI_INSTRUCTION_READ_RX_BUFFER1, do_csMcp2515);
	return true;
}

void sendData(byte maxByte, byte buffer[]) {
	for (size_t i = 0; i < maxByte; i++) {
		mcp2515_load_tx_buffer0(buffer[i], i, maxByte);
		//Serial.print("outgoing ");
		//Serial.print(i);
		//Serial.print(": ");
		//Serial.println(buffer[i]);
	}
	//Serial.print("state: ");
	//Serial.println(buffer[2]);
	mcp2515_execute_rts_command(0);
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