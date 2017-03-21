#include <EEPROM.h>
#include <SPI.h>
#include <DEFINITIONS.h>
#include <MCP2515.h>
#include <MCP2515.cpp>
#include <PID.h>
#include <DATAPACKAGE.h>

// ------------------------------------
// Controlling motors with id 0 and 1
// ------------------------------------
void setup()
{
	// Configure serial interface
	Serial.begin(9600);

	// Configure program data
	firstStart = true;
	extraCondition = false;

	// USER CONFIGURATION
	debugMode = true;

	// Define I/Os
	pinMode(do_csMcp2515, OUTPUT); // Set as input to enable pull up resistor. It's neccessary because the ss line is defined at pin 10 + 9
	pinMode(do_csArduino, OUTPUT); // Set as input to enable pull up resistor. It's neccessary because the ss line is defined at pin 10 + 9
	pinMode(di_encoderPinA, INPUT);
	pinMode(di_encoderPinB, INPUT);
	pinMode(di_mcp2515_int_rec, INPUT);
	pinMode(do_motorDirection, OUTPUT);

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
	if (EEPROM.read(eeprom_addr_act_pos_validate) == 1) {
		eeprom_states[eeprom_state_last_pos] = true;
		ist_motorAngle.bytes[0] = EEPROM.read(eeprom_addr_act_pos_1);
		ist_motorAngle.bytes[1] = EEPROM.read(eeprom_addr_act_pos_2);
		encoderValue = ist_motorAngle.data;
	}
	else {
		eeprom_states[eeprom_state_last_pos] = false;

		// TODO
		// SHow error blink code with red state led 
	}

	// Read ref pos value from eeprom
	if (EEPROM.read(eeprom_addr_ref_pos_validate) == 1) {
		eeprom_states[eeprom_state_ref_pos] = true;
		ref_pos.bytes[0] = EEPROM.read(eeprom_addr_ref_pos_1);
		ref_pos.bytes[1] = EEPROM.read(eeprom_addr_ref_pos_2);
		// TODO:
		// Convert motorId to array of ids to use motor id 0 and 1
		REFERENCE_POSITION[motorId] = ref_pos.data;
	}
	else {
		// TODO
		// SHow error blink code with red state led 
		eeprom_states[eeprom_state_ref_pos] = false;
	}

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
	// Do some work if eeprom values were read successfully
	if (eeprom_states[eeprom_state_ref_pos] & eeprom_states[eeprom_state_last_pos])
	{
		rxStateIst = 0x00;
		pwmValueTemp = 0;
		motorIsActive = true;

		// Check if message is received in buffer 0 or 1
		if ((digitalRead(di_mcp2515_int_rec) == 1))
		{
			// Get current rx buffer
			rxStateIst = mcp2515_execute_read_state_command(do_csMcp2515);

			// Read incoming data package
			receiveData(rxStateIst, rxStateSoll);

			// Wait after receive command
			delay((SAMPLE_TIME / 2) * 1000);
		}

		// Re initialize outgoing data array
		for (size_t i = 0; i < BYTES_TO_SEND; i++) outgoing_data[i] = 0;

		// Check incoming action and set outgoing package
		if ((incoming_data[in_action] == action_saveToEeprom) | (incoming_data[in_action] == action_disablePidController) | (incoming_data[in_action] == action_enablePidController) | (incoming_data[in_action] == action_newPosition))
		{
			outgoing_data[out_action] = incoming_data[in_action];
			outgoing_data[out_actionState] = state_pending;
			outgoing_data[out_motorId] = incoming_data[in_motorId];
		}

		// Send package back to server
		for (size_t i = 0; i < BYTES_TO_SEND; i++) mcp2515_load_tx_buffer0(outgoing_data[i], i, BYTES_TO_SEND);
		mcp2515_execute_rts_command(0);

		// Wait after send command
		delay((SAMPLE_TIME / 2) * 1000);

		// Work on action <saveToEeprom>
		if (incoming_data[in_action] == action_saveToEeprom)
		{
			// Write actual motor position to eeprom (byte 0 and 1)
			EEPROM.update(eeprom_addr_ref_pos_1, lowByte(encoderValue));
			EEPROM.update(eeprom_addr_ref_pos_2, highByte(encoderValue));

			// Validate writing and reset state if position is written to eeprom
			if ((EEPROM.read(eeprom_addr_ref_pos_1) == lowByte(encoderValue)) & (EEPROM.read(eeprom_addr_ref_pos_2) == highByte(encoderValue))) outgoing_data[out_actionState] = state_init;
		}

		// Work on action <disablePidController>
		if (incoming_data[in_action] == action_disablePidController)
		{
			pid_controller_enabled = false;
			outgoing_data[out_actionState] = state_init;
		}

		// Work on action <enablePidController>
		if (incoming_data[in_action] == action_enablePidController)
		{
			pid_controller_enabled = true;
			outgoing_data[out_actionState] = state_init;
		}

		// Work on action <newPosition>
		if (incoming_data[in_action] == action_newPosition)
		{
			// Convert byte to short 
			soll_motorAngle.bytes[0] = incoming_data[in_angle_1];
			soll_motorAngle.bytes[1] = incoming_data[in_angle_2];
			soll_motor_angle = soll_motorAngle.data;

			// Reset state if position is reached
			if (pid_error == 0) outgoing_data[out_actionState] = state_init;
		}

		// Controll motor with pid
		if (pid_controller_enabled)
		{
			// Convert encoder value to degree
			current_motor_angle = encoderValue*ENCODER_TO_DEGREE;

			// Calculate error term (soll - ist)
			pid_error = current_motor_angle - soll_motor_angle;
			if (abs(pid_error) <= MIN_PID_ERROR) pid_error = 0;

			// Calculate output for motor
			double pid_control_value = pidController(pid_error);

			// Configure direction value for motor
			// Direction input: when DIR is high (negative) current will flow from OUTA to OUTB, when it is low current will flow from OUTB to OUTA (positive).
			if (pid_control_value < 0) {
				digitalWrite(do_motorDirection, HIGH);
				pid_control_value = pid_control_value*(-1);
			}
			else digitalWrite(do_motorDirection, LOW);

			analogWrite(do_pwm, (int)pid_control_value);
		}

		// Store actual encoder value to eeprom when raspberry pi is off
		// Arduino gets the rest of power from a condensator
		if (!digitalRead(di_powerOn) & !writeToEeprom_inUse)
		{
			// Write actual motor position to eeprom (byte 2 and 3)
			EEPROM.update(eeprom_addr_act_pos_1, lowByte(encoderValue));
			EEPROM.update(eeprom_addr_act_pos_2, highByte(encoderValue));

			// Validate writing 
			// Store ok byte to eeprom (read this at startup to validate)
			if ((EEPROM.read(eeprom_addr_act_pos_1) == lowByte(encoderValue)) & (EEPROM.read(eeprom_addr_act_pos_2) == highByte(encoderValue))) EEPROM.write(eeprom_addr_act_pos_validate, 1);
			else EEPROM.write(eeprom_addr_act_pos_validate, 0);
		}

		//Send data to mcp
		sendData(BYTES_TO_SEND, sendBuffer);

	}
	delay((SAMPLE_TIME / 2) * 1000);
}

bool receiveData(byte rxStateIst, byte rxStateSoll)
{
	if ((rxStateIst & rxStateSoll) == 1) readControlValue(SPI_INSTRUCTION_READ_RX_BUFFER0, do_csMcp2515);
	else if ((rxStateIst & rxStateSoll) == 2) readControlValue(SPI_INSTRUCTION_READ_RX_BUFFER1, do_csMcp2515);
	return true;
}

void sendData(byte maxByte, byte buffer[]) {
	for (size_t i = 0; i < maxByte; i++) mcp2515_load_tx_buffer0(buffer[i], i, maxByte);
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
