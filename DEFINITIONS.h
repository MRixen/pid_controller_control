#define di_encoderPinA 2
#define di_encoderPinB 3
#define di_enableController 7
#define di_motorDirection1 10
#define ai_sollAngle A0

#define do_motorDirection1 4
#define do_motorDirection2 4
#define do_motorStandby 6

#define do_pwm 9

#define I2C_ID_CAN_BUS 7
#define I2C_ID_PID_CONTROLLER 8
#define I2C_ID_MONITOR 9

enum EepromAddresses
{
	eeprom_addr_ref_pos,
	eeprom_addr_ref_pos_sign,
	eeprom_addr_act_pos,
	eeprom_addr_act_pos_sign,
	eeprom_addr_error
};

enum ErrorCodes
{
	error_ok,
	error_eeprom_reset,
	error_eeprom_ref_pos,
	error_eeprom_act_pos
};


short pwmValueTemp = 0;
bool motorIsActive = false;
bool firstStart;
bool eeprom_init_state_ok = true;
const int MAX_ENCODER_OFFSET = 60;

long timeout_start = 0;
int value = 0;
int encoderValueTemp = 0;
long MAX_TIMEOUT = 2000;
bool eeprom_states[2];
bool reset_eeprom;

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
bool stopAllOperations = false;
union controlData_2b
{
	short data;
	byte bytes[2];
};
union controlData_5b
{
	short data;
	byte bytes[5];
};

controlData_2b soll_motorAngle, ist_motorAngle;
byte i2c_data_in[2] = { 0x00, 0x00};
byte i2c_data_out[1] = {0x00};

int ref_pos = 0;
int ref_pos_sign = 0;
int last_motor_angle = 0;
int act_pos_sign = 0;

int save_action;
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

bool pid_controller_enabled = true;
bool lockAction = false;
const int MAX_MOTOR_ANGLE = 90; 
const int MIN_MOTOR_ANGLE = -90;
int di_powerOn_state_old;
bool posOutReached = false;

union motor_id
{
	short lowByte;
	short highByte;
	byte bytes[2];
};
