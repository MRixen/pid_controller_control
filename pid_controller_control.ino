#include <SPI.h>

#define di_encoderPinA 2
#define di_encoderPinB 3
#define do_motorDirection 4
#define do_pwm 9
#define do_csMcp2515 8
#define do_csArduino 10
#define di_mcp2515_int_rec 7

union controlData
{
	short data;
	byte bytes[2];
};

controlData motorAngle, motorVel;
short pwmValueTemp = 0;
bool motorIsActive = false;
bool firstStart;
const int MAX_ENCODER_OFFSET = 60;

long timeout_start = 0;
int value = 0;
int encoderValueTemp = 0;
long MAX_TIMEOUT = 2000;

// USER DEFINED
const int SLOW_DOWN_CODE = 4;
const byte BYTE_TO_READ = 0x08;
const byte BYTES_TO_SEND = 0x08;

// ---- MCP SETUP BEGIN----
const byte REGISTER_TXB0SIDL_VALUE = 0x40;
const byte REGISTER_TXB0SIDH_VALUE = 0x02;
const byte REGISTER_TXB1SIDL_VALUE = 0x40;
const byte REGISTER_TXB1SIDH_VALUE = 0x02;
const byte REGISTER_TXB2SIDL_VALUE = 0x40;
const byte REGISTER_TXB2SIDH_VALUE = 0x02;
// ---- MCP SETUP END ----


// REGISTER MCP2515
const byte REGISTER_BFPCTRL = 0x0C;
const byte REGISTER_TXRTSCTRL = 0x0D;
const byte REGISTER_CANSTAT = 0x0E;
const byte REGISTER_CANCTRL = 0x0F;
const byte REGISTER_TEC = 0x1C;
const byte REGISTER_REC = 0x1D;
const byte REGISTER_CNF1 = 0x2A;
const byte REGISTER_CNF2 = 0x29;
const byte REGISTER_CNF3 = 0x28;
const byte REGISTER_CANINTE = 0x2B;
const byte REGISTER_CANINTF = 0x2C;
const byte REGISTER_EFLG = 0x2D;
const byte REGISTER_TXB0CTRL = 0x30;
const byte REGISTER_TXB1CTRL = 0x40;
const byte REGISTER_TXB2CTRL = 0x50;
const byte REGISTER_RXB0CTRL = 0x60;
const byte REGISTER_RXB1CTRL = 0x70;

const byte REGISTER_RXF0SIDL = 0x01;
const byte REGISTER_RXF1SIDL = 0x05;
const byte REGISTER_RXF2SIDL = 0x09;
const byte REGISTER_RXF3SIDL = 0x11;
const byte REGISTER_RXF4SIDL = 0x15;
const byte REGISTER_RXF5SIDL = 0x19;
const byte REGISTER_RXFxSIDL[] = { REGISTER_RXF0SIDL, REGISTER_RXF1SIDL ,REGISTER_RXF2SIDL ,REGISTER_RXF3SIDL ,REGISTER_RXF4SIDL, REGISTER_RXF5SIDL };

const byte REGISTER_RXF0SIDH = 0x00;
const byte REGISTER_RXF1SIDH = 0x04;
const byte REGISTER_RXF2SIDH = 0x08;
const byte REGISTER_RXF3SIDH = 0x10;
const byte REGISTER_RXF4SIDH = 0x14;
const byte REGISTER_RXF5SIDH = 0x18;
const byte REGISTER_RXFxSIDH[] = { REGISTER_RXF0SIDH, REGISTER_RXF1SIDH ,REGISTER_RXF2SIDH ,REGISTER_RXF3SIDH ,REGISTER_RXF4SIDH, REGISTER_RXF5SIDH };

const byte REGISTER_RXM0SIDL = 0x21;
const byte REGISTER_RXM1SIDL = 0x25;
const byte REGISTER_RXMxSIDL[] = { REGISTER_RXM0SIDL, REGISTER_RXM1SIDL };

const byte REGISTER_RXM0SIDH = 0x20;
const byte REGISTER_RXM1SIDH = 0x24;
const byte REGISTER_RXMxSIDH[] = { REGISTER_RXM0SIDH, REGISTER_RXM1SIDH };

const byte REGISTER_TXB0SIDH = 0x31;
const byte REGISTER_TXB0SIDL = 0x32;
const byte REGISTER_TXB0DLC = 0x35;
const byte REGISTER_TXB0D0 = 0x36;
const byte REGISTER_TXB0D1 = 0x37;
const byte REGISTER_TXB0D2 = 0x38;
const byte REGISTER_TXB0D3 = 0x39;
const byte REGISTER_TXB0D4 = 0x3A;
const byte REGISTER_TXB0D5 = 0x3B;
const byte REGISTER_TXB0D6 = 0x3C;
const byte REGISTER_TXB0D7 = 0x3D;
const byte REGISTER_TXB0Dx[] = { REGISTER_TXB0D0, REGISTER_TXB0D1, REGISTER_TXB0D2, REGISTER_TXB0D3, REGISTER_TXB0D4, REGISTER_TXB0D5, REGISTER_TXB0D6, REGISTER_TXB0D7 };
const byte REGISTER_TXB1SIDH = 0x41;
const byte REGISTER_TXB1SIDL = 0x42;
const byte REGISTER_TXB1DLC = 0x45;
const byte REGISTER_TXB1D0 = 0x46;
const byte REGISTER_TXB1D1 = 0x47;
const byte REGISTER_TXB1D2 = 0x48;
const byte REGISTER_TXB1D3 = 0x49;
const byte REGISTER_TXB1D4 = 0x4A;
const byte REGISTER_TXB1D5 = 0x4B;
const byte REGISTER_TXB1D6 = 0x4C;
const byte REGISTER_TXB1D7 = 0x4D;
const byte REGISTER_TXB1Dx[] = { REGISTER_TXB1D0, REGISTER_TXB1D1, REGISTER_TXB1D2, REGISTER_TXB1D3, REGISTER_TXB1D4, REGISTER_TXB1D5, REGISTER_TXB1D6, REGISTER_TXB1D7 };
const byte REGISTER_TXB2SIDH = 0x51;
const byte REGISTER_TXB2SIDL = 0x52;
const byte REGISTER_TXB2DLC = 0x55;
const byte REGISTER_TXB2D0 = 0x56;
const byte REGISTER_TXB2D1 = 0x57;
const byte REGISTER_TXB2D2 = 0x58;
const byte REGISTER_TXB2D3 = 0x59;
const byte REGISTER_TXB2D4 = 0x5A;
const byte REGISTER_TXB2D5 = 0x5B;
const byte REGISTER_TXB2D6 = 0x5C;
const byte REGISTER_TXB2D7 = 0x5D;
const byte REGISTER_TXB2Dx[] = { REGISTER_TXB2D0, REGISTER_TXB2D1, REGISTER_TXB2D2, REGISTER_TXB2D3, REGISTER_TXB2D4, REGISTER_TXB2D5, REGISTER_TXB2D6, REGISTER_TXB2D7 };
const byte REGISTER_RXB0D0 = 0x66;
const byte REGISTER_RXB0D1 = 0x67;
const byte REGISTER_RXB0D2 = 0x68;
const byte REGISTER_RXB0D3 = 0x69;
const byte REGISTER_RXB0D4 = 0x6A;
const byte REGISTER_RXB0D5 = 0x6B;
const byte REGISTER_RXB0D6 = 0x6C;
const byte REGISTER_RXB0D7 = 0x6D;
const byte REGISTER_RXB0Dx[] = { REGISTER_RXB0D0, REGISTER_RXB0D1, REGISTER_RXB0D2, REGISTER_RXB0D3, REGISTER_RXB0D4, REGISTER_RXB0D5, REGISTER_RXB0D6, REGISTER_RXB0D7 };
const byte REGISTER_RXB1D0 = 0x76;
const byte REGISTER_RXB1D1 = 0x77;
const byte REGISTER_RXB1D2 = 0x78;
const byte REGISTER_RXB1D3 = 0x79;
const byte REGISTER_RXB1D4 = 0x7A;
const byte REGISTER_RXB1D5 = 0x7B;
const byte REGISTER_RXB1D6 = 0x7C;
const byte REGISTER_RXB1D7 = 0x7D;
const byte REGISTER_RXB1Dx[] = { REGISTER_RXB1D0, REGISTER_RXB1D1, REGISTER_RXB1D2, REGISTER_RXB1D3, REGISTER_RXB1D4, REGISTER_RXB1D5, REGISTER_RXB1D6, REGISTER_RXB1D7 };
const byte REGISTER_CANSTAT_NORMAL_MODE = 0x00;
const byte REGISTER_CANSTAT_SLEEP_MODE = 0x20;
const byte REGISTER_CANSTAT_LOOPBACK_MODE = 0x40;
const byte REGISTER_CANSTAT_LISTEN_ONLY_MODE = 0x60;
const byte REGISTER_CANSTAT_CONFIGURATION_MODE = 0x80;
const byte REGISTER_CANCTRL_NORMAL_MODE = 0x00;
const byte REGISTER_CANCTRL_SLEEP_MODE = 0x20;
const byte REGISTER_CANCTRL_LOOPBACK_MODE = 0x40;
const byte REGISTER_CANCTRL_LISTEN_ONLY_MODE = 0x60;
const byte REGISTER_CANCTRL_CONFIGURATION_MODE = 0x80;

// REGISTER VALUES / DATA MCP2515
const byte REGISTER_CANINTF_VALUE_RESET_ALL_IF = 0x00;
const byte REGISTER_CANINTF_VALUE_RESET_MERRIF = 0x7F;
const byte REGISTER_CANINTF_VALUE_RESET_WAKIF = 0xBF;
const byte REGISTER_CANINTF_VALUE_RESET_ERRIF = 0xDF;
const byte REGISTER_CANINTF_VALUE_RESET_TX2IF = 0xEF;
const byte REGISTER_CANINTF_VALUE_RESET_TX1IF = 0xF7;
const byte REGISTER_CANINTF_VALUE_RESET_TX0IF = 0xFB;
const byte REGISTER_CANINTF_VALUE_RESET_RX1IF = 0xFD;
const byte REGISTER_CANINTF_VALUE_RESET_RX0IF = 0xFE;
const byte REGISTER_CANINTE_VALUE = 0x03; // Enable interrupt when rx buffer 0 full
const byte REGISTER_TXB0CTRL_VALUE = 0x00;
const byte REGISTER_TXB1CTRL_VALUE = 0x00;
const byte REGISTER_TXB2CTRL_VALUE = 0x00;
const byte REGISTER_RXB0CTRL_VALUE = 0x21; // Receive messages with standard identifier that meet filter criteria (filter 1)
const byte REGISTER_RXB1CTRL_VALUE = 0x22;  // Receive messages with standard identifier that meet filter criteria (filter 2)

const byte REGISTER_RXF0SIDL_VALUE = 0x01;
const byte REGISTER_RXF1SIDL_VALUE = 0xFF;
const byte REGISTER_RXF2SIDL_VALUE = 0xFF;
const byte REGISTER_RXF3SIDL_VALUE = 0xFF;
const byte REGISTER_RXF4SIDL_VALUE = 0xFF;
const byte REGISTER_RXF5SIDL_VALUE = 0xFF;
const byte REGISTER_RXFxSIDL_VALUE[] = { REGISTER_RXF0SIDL_VALUE, REGISTER_RXF1SIDL_VALUE , REGISTER_RXF2SIDL_VALUE , REGISTER_RXF3SIDL_VALUE , REGISTER_RXF4SIDL_VALUE , REGISTER_RXF5SIDL_VALUE };

const byte REGISTER_RXF0SIDH_VALUE = 0xFF;
const byte REGISTER_RXF1SIDH_VALUE = 0xFF;
const byte REGISTER_RXF2SIDH_VALUE = 0xFF;
const byte REGISTER_RXF3SIDH_VALUE = 0xFF;
const byte REGISTER_RXF4SIDH_VALUE = 0xFF;
const byte REGISTER_RXF5SIDH_VALUE = 0xFF;
const byte REGISTER_RXFxSIDH_VALUE[] = { REGISTER_RXF0SIDH_VALUE, REGISTER_RXF1SIDH_VALUE , REGISTER_RXF2SIDH_VALUE , REGISTER_RXF3SIDH_VALUE , REGISTER_RXF4SIDH_VALUE , REGISTER_RXF5SIDH_VALUE };

const byte REGISTER_RXM0SIDL_VALUE = 0xE0;
const byte REGISTER_RXM1SIDL_VALUE = 0xE0;
const byte REGISTER_RXMxSIDL_VALUE[] = { REGISTER_RXM0SIDL_VALUE, REGISTER_RXM1SIDL_VALUE };

const byte REGISTER_RXM0SIDH_VALUE = 0xFF;
const byte REGISTER_RXM1SIDH_VALUE = 0xFF;
const byte REGISTER_RXMxSIDH_VALUE[] = { REGISTER_RXM0SIDH_VALUE, REGISTER_RXM1SIDH_VALUE };

const byte REGISTER_TXBxSIDL_VALUE[] = { REGISTER_TXB0SIDL_VALUE, REGISTER_TXB1SIDL_VALUE, REGISTER_TXB2SIDL_VALUE };
const byte REGISTER_TXBxSIDH_VALUE[] = { REGISTER_TXB0SIDH_VALUE, REGISTER_TXB1SIDH_VALUE, REGISTER_TXB2SIDH_VALUE };

const byte REGISTER_CNF1_VALUE = 0x00;//0x03; // Baud rate prescaler calculated with application (Fosc = 8Mhz and CANspeed = 125kHz)
const byte REGISTER_CNF2_VALUE = 0xB8;//0x90; // BTLMODE = 1 (PHaseSegment 2 is configured with CNF 3) and PhaseSegment 1 = 8xTQ (7+1)
const byte REGISTER_CNF3_VALUE = 0x05;//0x02; // Set PhaseSegment 2 = 6xTQ (5+1)
bool currentTxBuffer[] = { true, false, false };

byte rxStateIst = 0x00;
const byte rxStateSoll = 0x03;

// SPI INSTRUCTIONS MCP2515
const byte SPI_INSTRUCTION_RESET = 0xC0;
const byte SPI_INSTRUCTION_READ = 0x03;
const byte SPI_INSTRUCTION_READ_RX_BUFFER0_SIDH = 0x90;
const byte SPI_INSTRUCTION_READ_RX_BUFFER0 = 0x92;
const byte SPI_INSTRUCTION_READ_RX_BUFFER1 = 0x96;
const byte SPI_INSTRUCTION_WRITE = 0x02;
const byte SPI_INSTRUCTION_LOAD_TX_BUFFER0_ID = 0x40;
const byte SPI_INSTRUCTION_LOAD_TX_BUFFER0_DATA = 0x41;
const byte SPI_INSTRUCTION_RTS_BUFFER0 = 0x81;
const byte SPI_INSTRUCTION_RTS_BUFFER1 = 0x82;
const byte SPI_INSTRUCTION_RTS_BUFFER2 = 0x84;
const byte SPI_INSTRUCTION_READ_STATUS = 0xA0;
const byte SPI_INSTRUCTION_RX_STATUS = 0xB0;
const byte SPI_INSTRUCTION_BIT_MODIFY = 0x05;

// TODO: Expand content of the send byte array
// CONTENT
// 0. byte: Number of the task 
// 1. byte: ID of the motor to move
// 2-3. byte: Velocity to move the motor to the specified position
// 4-5. byte: Endposition for the specific motor 
// 6. byte: Direction of the motor
// 7. byte: 

// CONTENT
// 0. byte: Direction of the motor
// 1-2. byte: Current angle of motor
byte sendBuffer[8];

// GLOBAL DATA
const long MAX_WAIT_TIME = 10000;
const int MAX_ERROR_COUNTER_MODESWITCH = 3;
bool stopAllOperations = false;
bool debugMode;
const int ADXL = 1;
const int MCP2515 = 2;
const int NO_DEVICE = 3;
long errorTimerValue;
const long MAX_PULSE_TYPE_DURATION = (5 * 3.14) * 1000;

byte ReadBuffer[7];
int pulseCounter = 1;

int counter = 1;


// PDI controller data
const int MULTIPLICATION_FACTOR = 10;

long encoderValue = 0;
int pwm_motorDirection, taskNumber, motorId, extraCondition;

double const SAMPLE_TIME = 0.05; // s
double const UPPER_SATURATION_LIMIT = 255;
double const LOWER_SATURATION_LIMIT = -255;
double const MIN_PID_ERROR = 0.15;

// -----------------------------------------
// PID CONTROLLER DATA
// -----------------------------------------

// -----------------------------------------
// Premium gear motor (638810 - 12rpm)
// -----------------------------------------
// VERY SLOW
//double P = -1.46749932499104;
//double I = -0.115013862304265;
//double D = 0.548945887967433;
//double N = 0.862662739489541;

// FAST AS POSSIBLE (BEST RESULTS)
//double P = -60.0661737867048;
//double I = -4.94953312365104;
//double D = -0.454950956464534;
//double N = 3.15338115327649;

//const double ENCODER_TO_DEGREE = ((double)360 / 34608);

// TEST FOR FASTER RESPONSE AT 30deg SOLLVALUE
//double P = -6.44036045007237;
//double I = -2.12725533466775;
//double D = 0.0227955275430827;
//double N = 3.51293845336784;
// -----------------------------------------
// -----------------------------------------


// -----------------------------------------
// Econ gear motor (638340 - 19rpm)
// -----------------------------------------
// FAST AS POSSIBLE
//double P = -12.8539033109958;
//double I = -0.712646331260047;
//double D = 0.343845666242764;
//double N = 5.46914538556796;

// TEST
double P = -10.6155510552541;
double I = -0.487958012814397;
double D = 0.312079469943773;
double N = 4.6424770619278;

const double ENCODER_TO_DEGREE = 1;
// -----------------------------------------
// -----------------------------------------

double soll_motor_angle = 0; // deg
double pid_error;
double current_motor_angle = 0;
double p_term = 0;
double i_term = 0;
double d_term = 0;
double preSat = 0;
double d_filter = 0;

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

	// Start timer to measure the program execution
	errorTimerValue = millis();

	// Attach interrupt for encoder
	attachInterrupt(digitalPinToInterrupt(di_encoderPinA), doEncoderA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(di_encoderPinB), doEncoderB, CHANGE);

	// Give time to set up
	delay(100);
}

void loop()
{
	// MESSAGES COMING FROM RASPBERRY VIA CAN BUS:
	// BYTE 1: DIRECTION OF MOTOR
	// BYTE 2, 3: PWM VALUE

	// MESSAGES GOING TO RASPBERRY VIA CAN BUS:
	// BYTE 1: DIRECTION OF MOTOR
	// BYTE 2, 3: ENCODER VALUE

	rxStateIst = 0x00;
	pwmValueTemp = 0;
	motorIsActive = true;

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

	// Wait until a message is received in buffer 0 or 1
	while ((digitalRead(di_mcp2515_int_rec) == 1)) {
		delay(1);
		Serial.println("wait");
	}

	// Get current rx buffer
	rxStateIst = mcp2515_execute_read_state_command(do_csMcp2515);

	// Read pwm and motor direction data
	receiveControlData(rxStateIst, rxStateSoll);
	soll_motor_angle = motorAngle.data;

	//Serial.print("soll_motor_angle: ");
	//Serial.println(soll_motor_angle);

	//Serial.print("encoderValue: ");
	//Serial.println(encoderValue);

	// Convert encoder value to degree
	current_motor_angle = encoderValue*ENCODER_TO_DEGREE;

	// Calculate error term (soll - ist)
	pid_error = current_motor_angle - soll_motor_angle;
	if (abs(pid_error) <= MIN_PID_ERROR) pid_error = 0;

	//Serial.print(encoderValue);

	//Serial.print("pid_error: ");
	//Serial.println(pid_error);

	// Calculate output for motor
	double pid_control_value = pidController(pid_error);

	//Serial.print("pid_control_value: ");
	//Serial.println((int)pid_control_value);

	// Configure direction value for motor
	// Direction input: when DIR is high (negative) current will flow from OUTA to OUTB, when it is low current will flow from OUTB to OUTA (positive).
	if (pid_control_value < 0) {
		digitalWrite(do_motorDirection, HIGH);
		pid_control_value = pid_control_value*(-1);
	}
	else digitalWrite(do_motorDirection, LOW);

	// Rotate motor only if no task 2 selected (safe ref pos)
	// THe user needs to rotate the motor manually....
	if(extraCondition == 0) analogWrite(do_pwm, (int)pid_control_value);

	delay((SAMPLE_TIME/2) * 1000);

	// Send encoder value back to raspberry
	encoderValueTemp = encoderValue*ENCODER_TO_DEGREE*MULTIPLICATION_FACTOR;

	// Set specific data to byteArray (task no, motor id, etc.)
	sendBuffer[0] = taskNumber;
	sendBuffer[1] = motorId;
	sendBuffer[2] = 0; // Set to zero because the server do nothing with it
	sendBuffer[3] = 0; // Set to zero because the server do nothing with it
	if (extraCondition == 0) sendBuffer[7] = 0; // Set to zero because the server do nothing with it
	else sendBuffer[7] = 1;

	// Write encoder direction to buffer
	if (encoderValueTemp < 0) {
		sendBuffer[6] = 0;
		encoderValueTemp = encoderValueTemp*(-1);
	}
	else sendBuffer[6] = 1;

	// Write encoder value to buffer
	sendBuffer[4] = lowByte(encoderValueTemp);
	sendBuffer[5] = highByte(encoderValueTemp);

	//Send data to mcp
	for (size_t i = 0; i < BYTES_TO_SEND; i++) mcp2515_load_tx_buffer0(sendBuffer[i], i, BYTES_TO_SEND);
	mcp2515_execute_rts_command(0);

	delay((SAMPLE_TIME / 2) * 1000);
}

double pidController(double error) {

	double pid_output = 0;

	// Calculate p term
	p_term = P * error;

	// Calculate i term with clamping (anti windup)
	if (clamp(preSat, error)) i_term = i_term + 0;
	else i_term = i_term + (error*SAMPLE_TIME);
	i_term = i_term*I;

	// Calculate d term with filtered derivative
	d_term = (D * error - d_filter) * N;

	// Summarize the p,i,d terms
	preSat = p_term + i_term + d_term;

	// Saturate output for max / min 255 / -255
	if (preSat > 255) pid_output = 255;
	else if (preSat < -255) pid_output = -255;
	else pid_output = preSat;

	d_filter = d_filter + d_term*SAMPLE_TIME;

	return pid_output;
}

bool clamp(double preSat, double preIntegrator) {

	double deadZone_out = 0;
	int signDeltaU = 0;
	int signPreIntegrator = 0;

	// Check if the presat value is inside the deadzone
	if ((preSat <= UPPER_SATURATION_LIMIT) & (preSat >= LOWER_SATURATION_LIMIT)) deadZone_out = 0;
	else {
		if ((preSat > UPPER_SATURATION_LIMIT)) deadZone_out = preSat - UPPER_SATURATION_LIMIT;
		else if ((preSat < LOWER_SATURATION_LIMIT)) deadZone_out = preSat - LOWER_SATURATION_LIMIT;
	}

	// Calculate sign delta u
	if (deadZone_out > 0) signDeltaU = 1;
	else if (deadZone_out == 0) signDeltaU = 0;
	else if (deadZone_out < 0) signDeltaU = -1;

	// Calculate sign pre integrator
	if (preIntegrator > 0) signPreIntegrator = 1;
	else if (preIntegrator == 0) signPreIntegrator = 0;
	else if (preIntegrator < 0) signPreIntegrator = -1;

	// Return true if both signs equal and the preSat outside the dead zone
	if ((signDeltaU == signPreIntegrator) & (deadZone_out != 0)) return true;
}

bool receiveControlData(byte rxStateIst, byte rxStateSoll)
{
	if ((rxStateIst & rxStateSoll) == 1) readControlValue(SPI_INSTRUCTION_READ_RX_BUFFER0, do_csMcp2515);
	else if ((rxStateIst & rxStateSoll) == 2) readControlValue(SPI_INSTRUCTION_READ_RX_BUFFER1, do_csMcp2515);
	return true;
}

void mcp2515_init_tx_buffer0(byte identifierLow, byte identifierHigh, byte messageSize) {

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	mcp2515_execute_write_command(REGISTER_TXB0SIDL, identifierLow, do_csMcp2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(REGISTER_TXB0SIDH, identifierHigh, do_csMcp2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(REGISTER_TXB0DLC, messageSize, do_csMcp2515);
}

void mcp2515_init_tx_buffer1(byte identifierLow, byte identifierHigh, byte messageSize) {

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	mcp2515_execute_write_command(REGISTER_TXB1SIDL, identifierLow, do_csMcp2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(REGISTER_TXB1SIDH, identifierHigh, do_csMcp2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(REGISTER_TXB1DLC, messageSize, do_csMcp2515);
}

void mcp2515_init_tx_buffer2(byte identifierLow, byte identifierHigh, byte messageSize) {

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	mcp2515_execute_write_command(REGISTER_TXB2SIDL, identifierLow, do_csMcp2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(REGISTER_TXB2SIDH, identifierHigh, do_csMcp2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(REGISTER_TXB2DLC, messageSize, do_csMcp2515);
}

void initMcp2515() {
	// Reset chip to set in operation mode
	mcp2515_execute_reset_command();

	// Configure bit timing
	mcp2515_configureCanBus();

	// Configure interrupts
	mcp2515_configureInterrupts();

	// Set device to normal mode
	mcp2515_switchMode(REGISTER_CANSTAT_NORMAL_MODE, REGISTER_CANCTRL_NORMAL_MODE);
}

void mcp2515_execute_reset_command() {
	// Reset chip to get initial condition and wait for operation mode state bit
	byte returnMessage;

	writeSimpleCommandSpi(SPI_INSTRUCTION_RESET, do_csMcp2515);

	// Read the register value
	byte actualMode = mcp2515_execute_read_command(REGISTER_CANSTAT, do_csMcp2515);
	Serial.print("Try to reset Mcp2515");
	while (REGISTER_CANSTAT_CONFIGURATION_MODE != (REGISTER_CANSTAT_CONFIGURATION_MODE & actualMode))
	{
		actualMode = mcp2515_execute_read_command(REGISTER_CANSTAT, do_csMcp2515);
		Serial.print(actualMode);
	}

	if (debugMode) Serial.print("Mcp2515 reset succesfully and switch do mode ");
	if (debugMode) Serial.println(actualMode);
}

void mcp2515_configureCanBus() {
	// Configure bit timing

	mcp2515_execute_write_command(REGISTER_CNF1, REGISTER_CNF1_VALUE, do_csMcp2515);

	mcp2515_execute_write_command(REGISTER_CNF2, REGISTER_CNF2_VALUE, do_csMcp2515);

	mcp2515_execute_write_command(REGISTER_CNF3, REGISTER_CNF3_VALUE, do_csMcp2515);

	if (debugMode) Serial.println("Mcp2515 configure bus succesfully");
}

void mcp2515_configureInterrupts() {
	mcp2515_execute_write_command(REGISTER_CANINTE, REGISTER_CANINTE_VALUE, do_csMcp2515);

	if (debugMode) Serial.println("Mcp2515 configure interrupts succesfully");
}

void mcp2515_configureMasksFilters(byte registerAddress, byte registerValue) {

	// Set parameters for rx buffer 0
	mcp2515_execute_write_command(REGISTER_RXB0CTRL, REGISTER_RXB0CTRL_VALUE, do_csMcp2515);

	// Set parameters for rx buffer 1
	mcp2515_execute_write_command(REGISTER_RXB1CTRL, REGISTER_RXB1CTRL_VALUE, do_csMcp2515);
}

void mcp2515_configureRxBuffer() {

	// Set parameters for rx buffer 0
	mcp2515_execute_write_command(0x60, 0x04, do_csMcp2515); // as dec 39: Filter 1 is enabled (rollover is active / bukt is set)

	mcp2515_execute_write_command(0x70, 0x00, do_csMcp2515); // as dec 33: Filter 1 is enabled (if bukt is set)

	if (debugMode) Serial.println("Mcp2515 configure rx buffer succesfully");
}

void mcp2515_switchMode(byte modeToCheck, byte modeToSwitch) {
	// Reset chip to get initial condition and wait for operation mode state bit
	byte returnMessage[1];
	byte actualMode;

	// Repeat mode switching for a specific time when the first trial is without success
	for (int i = 0; i < MAX_ERROR_COUNTER_MODESWITCH; i++)
	{
		mcp2515_execute_write_command(REGISTER_CANCTRL, modeToSwitch, do_csMcp2515);

		// Read the register value
		actualMode = mcp2515_execute_read_command(REGISTER_CANSTAT, do_csMcp2515);
		long elapsedTime = 0;
		long errorTimerValue = millis();
		while ((actualMode != modeToSwitch) && (elapsedTime <= MAX_WAIT_TIME))
		{
			actualMode = mcp2515_execute_read_command(REGISTER_CANSTAT, do_csMcp2515);
			elapsedTime = millis() - errorTimerValue; // Stop time to break loop when mode isnt switching
		}
		if (elapsedTime > MAX_WAIT_TIME)
		{
			if (debugMode) Serial.println("Abort waiting. Max. waiting time reached.");
			if (i == MAX_ERROR_COUNTER_MODESWITCH)
			{
				if (debugMode) Serial.println("ERROR MODE SWITCH - STOP ALL OPERATIONS");
				stopAllOperations = true;
			}
		}
		else break;
	}

	if (debugMode) Serial.print("Mcp2515 switch to mode ");
	if (debugMode) Serial.print(actualMode);
	if (debugMode) Serial.println(" succesfully");
}

byte mcp2515_execute_read_command(byte registerToRead, int cs_pin)
{
	byte returnMessage;

	// Enable device
	setCsPin(cs_pin, LOW);

	// Write spi instruction read  
	SPI.transfer(SPI_INSTRUCTION_READ);

	// Write the address of the register to read
	SPI.transfer(registerToRead);
	returnMessage = SPI.transfer(0x00);

	// Disable device
	setCsPin(cs_pin, HIGH);

	delay(SLOW_DOWN_CODE);

	mcp2515_execute_write_command(REGISTER_CANINTF, REGISTER_CANINTF_VALUE_RESET_ALL_IF, do_csMcp2515);

	return returnMessage;
}

byte mcp2515_execute_read_state_command(int cs_pin)
{
	byte returnMessage;

	// Enable device
	setCsPin(cs_pin, LOW);

	// Write spi instruction read  
	SPI.transfer(SPI_INSTRUCTION_READ_STATUS);
	returnMessage = SPI.transfer(0x00);

	// Disable device
	setCsPin(cs_pin, HIGH);

	delay(SLOW_DOWN_CODE);

	return returnMessage;
}

void mcp2515_load_tx_buffer0(byte messageData, int byteNumber, int messageSize) {

	//byte registerAddress;
	//int txBuffer_id = 0;

	//if (currentTxBuffer[0])
	//{
	//	registerAddress = REGISTER_TXB0Dx[byteNumber];
	//	txBuffer_id = 0;
	//	if (byteNumber == (messageSize - 1))
	//	{
	//		currentTxBuffer[0] = false;
	//		currentTxBuffer[1] = true;
	//	}
	//}
	//else if (currentTxBuffer[1]) {
	//	registerAddress = REGISTER_TXB1Dx[byteNumber];
	//	txBuffer_id = 1;
	//	if (byteNumber == (messageSize - 1))
	//	{
	//		currentTxBuffer[1] = false;
	//		currentTxBuffer[2] = true;
	//	}
	//}
	//else if (currentTxBuffer[2]) {
	//	registerAddress = REGISTER_TXB2Dx[byteNumber];
	//	txBuffer_id = 2;
	//	if (byteNumber == (messageSize - 1))
	//	{
	//		currentTxBuffer[2] = false;
	//		currentTxBuffer[0] = true;
	//	}
	//}

	//registerAddress = REGISTER_TXB0Dx[byteNumber];
	mcp2515_execute_write_command(REGISTER_TXB0Dx[byteNumber], messageData, do_csMcp2515);

	// DEBUG
	//if(byteNumber == 6) Serial.println(mcp2515_execute_read_command(registerAddress, CS_PIN_MCP2515));

}

void mcp2515_load_tx_buffer1(byte messageData, int byteNumber, int messageSize) {

	mcp2515_execute_write_command(REGISTER_TXB1Dx[byteNumber], messageData, do_csMcp2515);

}

void mcp2515_load_tx_buffer2(byte messageData, int byteNumber, int messageSize) {

	mcp2515_execute_write_command(REGISTER_TXB2Dx[byteNumber], messageData, do_csMcp2515);

}

void mcp2515_init_tx_buffer(byte identifierLow, byte identifierHigh, byte messageSize, int regAddr) {

	byte regAddr_sidl;
	byte regAddr_sidh;
	byte regAddr_dlc;

	switch (regAddr)
	{
	case 0:
		regAddr_sidl = REGISTER_TXB0SIDL;
		regAddr_sidh = REGISTER_TXB0SIDH;
		regAddr_dlc = REGISTER_TXB0DLC;
		break;
	case 1:
		regAddr_sidl = REGISTER_TXB1SIDL;
		regAddr_sidh = REGISTER_TXB1SIDH;
		regAddr_dlc = REGISTER_TXB1DLC;
		break;
	case 2:
		regAddr_sidl = REGISTER_TXB2SIDL;
		regAddr_sidh = REGISTER_TXB2SIDH;
		regAddr_dlc = REGISTER_TXB2DLC;
		break;
	default:
		break;
	}

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	mcp2515_execute_write_command(regAddr_sidl, identifierLow, do_csMcp2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(regAddr_sidh, identifierHigh, do_csMcp2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(regAddr_dlc, messageSize, do_csMcp2515);
}

void writeToSpi(byte address, byte data, int cs_pin) {
	byte spiMessage[] = { address, data };

	setCsPin(cs_pin, LOW);
	for (int i = 0; i < 2; i++) SPI.transfer(spiMessage[i]);
	setCsPin(cs_pin, HIGH);
	delay(SLOW_DOWN_CODE);
}

void setCsPin(int cs_pin, uint8_t value) {
	digitalWrite(10, value);
	digitalWrite(cs_pin, value);
}

void mcp2515_execute_write_command(byte address, byte data, int cs_pin)
{
	setCsPin(cs_pin, LOW); // Enable device
	SPI.transfer(SPI_INSTRUCTION_WRITE); // Write spi instruction write  
	SPI.transfer(address);
	SPI.transfer(data);
	setCsPin(cs_pin, HIGH); // Disable device
	delay(SLOW_DOWN_CODE);
}

void mcp2515_execute_rts_command(int bufferId)
{
	byte spiMessage[1];

	switch (bufferId)
	{
	case 0:
		spiMessage[0] = SPI_INSTRUCTION_RTS_BUFFER0;
		break;
	case 1:
		spiMessage[0] = SPI_INSTRUCTION_RTS_BUFFER1;
		break;
	case 2:
		spiMessage[0] = SPI_INSTRUCTION_RTS_BUFFER2;
		break;
	default:
		break;
	}
	writeSimpleCommandSpi(spiMessage[0], do_csMcp2515);
}

void writeSimpleCommandSpi(byte command, int cs_pin)
{
	setCsPin(cs_pin, LOW);
	SPI.transfer(command);
	setCsPin(cs_pin, HIGH);
	delay(SLOW_DOWN_CODE);
}

bool readControlValue(byte bufferId, int cs_pin)
{
	// Read byte from can bus
	// 1. byte: motor direction
	// 2. + 3. byte: pwm value

	byte returnMessage[BYTE_TO_READ];

	// Read pwm value as byte from can bus
	setCsPin(cs_pin, LOW);
	SPI.transfer(bufferId);
	for (int i = 0; i < BYTE_TO_READ; i++) returnMessage[i] = SPI.transfer(0x00);
	setCsPin(cs_pin, HIGH);
	//delay(SLOW_DOWN_CODE);

	//for (int i = 0; i < BYTE_TO_READ; i++) {
	//	Serial.print("returnMessage[");
	//	Serial.print(i);
	//	Serial.print("]: ");
	//	Serial.println(returnMessage[i]);
	//}

	// Convert byte to short 
	taskNumber = returnMessage[0];
	motorId = returnMessage[1];
	motorVel.bytes[0] = returnMessage[2];
	motorVel.bytes[0] = returnMessage[3];
	motorAngle.bytes[0] = returnMessage[4];
	motorAngle.bytes[1] = returnMessage[5];
	pwm_motorDirection = returnMessage[6];
	extraCondition = returnMessage[7];

	Serial.print("extraCondition: ");
	Serial.println(extraCondition);

	//Serial.print("motorAngle.angle: ");
	//Serial.println(motorAngle.angle);

	return true;
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
