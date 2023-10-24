/*   Populate with Constants Related to Teensy Control   */

//this is the 3 feedbacks
#define FEEDBACK_BUF_SIZE 3

#define MOTORS_OFF false //used for debugging

#define OUTERLOOP 10  //Hz

#define HWSERIAL Serial2

#define IDNAME(name) #name

#define DIST_CONSTANT             0.002

#define GYRO_X_CONSTANT           480.0
#define GYRO_YAW_CONSTANT         0

#define GYRO_Y_CONSTANT           323.45

//Define ceiling height from where we plug in battery in meters
#define CEIL_HEIGHT_FROM_START    4 

//sensor and controller rates
#define FAST_SENSOR_LOOP_FREQ           100.0
#define BARO_LOOP_FREQ                  50.0

//constants
#define MICROS_TO_SEC             1000000.0

// Define topic names
#define MULTIARRAY_TOPIC          "REPLACE WITH TOPIC NAME"
#define BOOL_TOPIC          "REPLACE WITH TOPIC NAME"
