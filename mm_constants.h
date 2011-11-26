/*
  mm_contants.h - constants
*/

#ifndef mm_contants_h
#define mm_contants_h

// This is the Mezzo Mill version of this code.
#define MM_VERSION "0.1"

// Settings that can only be set at compile-time:

//TODO_MM - this value needs to be communicated 
// in a setup packet after the version info
#define LINE_BUFFER_SIZE 128

//#define BAUD_RATE 9600
#define BAUD_RATE 57600
// every baud rate above this line worked when tested
//#define BAUD_RATE 19200
//#define BAUD_RATE 115200

// in milliseconds
#define MOTOR_SPIN_UP_AND_DOWN_TIME 1000

#define STEPPERS_ENABLE_SIGNAL 0
#define STEPPERS_DISABLE_SIGNAL 1

#define STEPPING_DDR       DDRC
#define STEPPING_PORT      PORTC 
//temporary change to fix motherboard screwup
#define X_STEP_BIT           4
#define Y_STEP_BIT           2
#define Z_STEP_BIT           0
#define X_DIRECTION_BIT            5
#define Y_DIRECTION_BIT            3
#define Z_DIRECTION_BIT            1

#define POWER_DETECTION_DDR DDRD
#define POWER_DETECTION_PIN PIND
#define POWER_DETECTION_BIT 7

#define POWER_IS_OFF 0

// MM_COMMENT added this
#define CAP_DDR      DDRB
#define CAP_PORT     PORTB
#define CAP_PIN      PINB

#define X_AXIS_CAP_SEND 0
#define X_AXIS_CAP_RECV 1
#define Y_AXIS_CAP_SEND 2
#define Y_AXIS_CAP_RECV 3
#define Z_AXIS_CAP_SEND 4
#define Z_AXIS_CAP_RECV 5


#define END_MILL_CAP_DDR      DDRD
#define END_MILL_CAP_PORT     PORTD
#define END_MILL_CAP_PIN      PIND

#define END_MILL_CAP_SEND 3
#define END_MILL_CAP_RECV 4

// This is our last DIO port
// which we will use to detect if the lid is open
#define LID_DDR      DDRD
#define LID_PORT     PORTD
#define LID_PIN      PIND

#define IS_ENCLOSURE_LID_OPEN_BIT 5

#define LID_IS_OPEN 0

// Default settings (used when resetting eeprom-settings)
#define MICROSTEPS 1
#define DEFAULT_X_STEPS_PER_MM (188.97637795275*MICROSTEPS)
#define DEFAULT_Y_STEPS_PER_MM (188.97637795275*MICROSTEPS)
#define DEFAULT_Z_STEPS_PER_MM (188.97637795275*MICROSTEPS)
#define DEFAULT_STEP_PULSE_MICROSECONDS 1

// MM_COMMENT - 381 = 15 in/min
#define DEFAULT_RAPID_FEEDRATE   (381.0) // in millimeters per min
#define DEFAULT_FEEDRATE (381.0) 

#define DEFAULT_ACCELERATION 7.0

// Use this line for default operation (step-pulses high)
// #define STEPPING_INVERT_MASK 0
// Uncomment this line for inverted stepping (step-pulses low, rest high)
// #define STEPPING_INVERT_MASK (STEP_MASK)
// Uncomment this line to invert all step- and direction bits
// #define STEPPING_INVERT_MASK (STEPPING_MASK)
// Or bake your own like this adding any step-bits or directions you want to invert:
// #define STEPPING_INVERT_MASK (STEP_MASK | (1<<X_DIRECTION_BIT) | (1<<Y_DIRECTION_BIT))
// MM_COMMENT - baking my own to fix the z direction
#define DEFAULT_STEPPING_INVERT_MASK ( (1<<Z_DIRECTION_BIT))

// Some useful constants
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)
#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

#endif
