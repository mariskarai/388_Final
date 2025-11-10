/******************************************************************************
 *   Header Includes
 *******************************************************************************/
#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>

#include "eecs_388_lib.h"

/******************************************************************************
 *   Constant definitions
 *******************************************************************************/

/* I2C BMP180 constants*/
#define BMP180_ADDR     (0x77)    // BMP180 I2C address
#define BASE_CAL_ADDR   (0xAA)    // Base address of calibration data
#define ID_REG_ADDR     (0xD0)    // Chip ID register address+
#define CTRL_MEAS_ADDR  (0xF4)    // Measure control register addresss
#define ADC_MSB_ADDR    (0xF6)    // out_msb ADC register addreess
#define CTRL_TEMP_CMND  (0x2E)    // Measure control register command to start temp conversion
#define BMP180_READ     ((BMP180_ADDR << 1) | 1)    // Read from BMP180 register
#define BMP180_WRITE    ((BMP180_ADDR << 1) | 0)    // Write to BMP180 register

/*ServoMotor constants*/
#define SERVO_PULSE_MAX (2400)    /* 2400 us */
#define SERVO_PULSE_MIN (544)     /* 544 us */
#define SERVO_PERIOD    (20000)   /* 20000 us (20ms) */
#define MIN_ANGLE       (0)       /* degrees */
#define MAX_ANGLE       (180)     /* degrees */

/******************************************************************************
 *   Type declarations
 *******************************************************************************/
typedef struct {
  int16_t  cal_AC1;
  int16_t  cal_AC2;
  int16_t  cal_AC3;
  uint16_t cal_AC4;
  uint16_t cal_AC5;
  uint16_t cal_AC6;
  int16_t  cal_B1;
  int16_t  cal_B2;
  int16_t  cal_MB;
  int16_t  cal_MC;
  int16_t  cal_MD;
} bmp180_calibration;

/******************************************************************************
 *   Global Variables
 *******************************************************************************/
bmp180_calibration calib_data;

/******************************************************************************
 *   Local function prototypes
 *******************************************************************************/
static void auto_brake();
static void engine_temp();
static void steering(int pos);

/******************************************************************************
 *   Function: auto_brake() - Auto Brake
 *      Pre condition: 
 *          None
 *      Post condition: 
 *          Checks the LiDAR distance and configures LEDs
 *******************************************************************************/
static void auto_brake()
{
    // Task-1&2: 
    // Your code goes here (Use Lab 2 & 4 for reference)
    // Check the project document to understand the task
  int new_dist = 0

  while (new_dist == 0)
  {
    uint16_t dist = 0;

    if ('Y' == ser_read() && 'Y' == ser_read())
    {
      uint8_t dist_l = ser_read();
      uint8_t dist_h = ser_read();

      dist = (dist_h << 8) | dist_l;

      for (int i = 0; i < 5; i++) ser_read;
    }

    if (dist == 0 || dist > 1200) return;

    if (dist > 200)
    {
      gpio_write(GPIO_13, OFF);
      gpio_write(GPIO_12, ON);
      gpio_write(GPIO_11, OFF);
    }
    else if (dist > 100 && dist <= 200)
    {
      gpio_write(GPIO_13, ON);
      gpio_write(GPIO_12, ON);
      gpio_write(GPIO_11, OFF);
    }
    else if (dist > 60 && dist <= 100)
    {
      gpio_write(GPIO_13, ON);
      gpio_write(GPIO_12, OFF);
      gpio_write(GPIO_11, OFF);
    }
    else if (dist <= 60)
    {
      gpio_write(GPIO_13, ON);
      delay_ms(100);
      gpio_write(GPIO_13, ON);
      delay_ms(100);
    }

      ser_printf("Distance: %d", dist);
  }
}

/******************************************************************************
 *   Function: engine_temp() - Auto Brake
 *      Pre condition: 
 *          None
 *      Post condition: 
 *          Checks the BMP180 temp_sensor data
 *******************************************************************************/
static void engine_temp()
{
    // BONUS TASK 
    // Your code goes here (Use Lab 6 for reference)
    // You will need to copy over several APIs to get the temperature from 
    // the BMP180 sensor. 
}

/******************************************************************************
 *   Function: steering() - Steering
 *      Pre condition: 
 *          None
 *      Post condition: 
 *          Control the servomotor with GPIO_6
 *******************************************************************************/
static void steering(int pos)
{
    // Task-4: 
    // Your code goes here (Use Lab 05 for reference)
    // Check the project document to understand the task

  if (pos < MIN_ANGLE) pos = MIN_ANGLE;
  if (pos > MAX_ANGLE) pos = MAX_ANGLE;

  int pulse_width = SERVO_PULSE_MIN + ((SERVO_PULSE_MAX - SERVO_PULSE_MIN) * pos / 180);
  int off = SERVO_PERIOD - pulse_width;

  gpio_write(GPIO_6, ON);
  delay_us(pulse_width);
  gpio_write(GPIO_6, OFF);

  if (off >= 1000)
  {
    delay_ms(off / 1000);
    off = off % 1000;
  }
  if (off > 0)
  {
    delay_us(off);
  }

  return;

}

/******************************************************************************
 *   Function: setup() - Initializes the Arduino System
 *      Pre condition: 
 *          Hardware must be properly connected (BMP180 sensors, etc.)
 *      Post condition: 
 *          Runs initialization calls one time on power up
 *          UART is initialized for ser_printf()
 *          I2C is initialized for communication with BMP180
 *          calib_data is filled with calibration data from BMP180
 *******************************************************************************/
void setup() 
{
uart_init();     // Initialize UART for serial output
i2c_init();      // Initialize I2C for communication with BMP180

//Setup Auto-break LEDs for Distance
gpio_mode(GPIO_13, GPIO_OUTPUT); //RED
gpio_mode(GPIO_12, GPIO_OUTPUT); //GREEN
gpio_mode(GPIO_11, GPIO_OUTPUT); //BLUE

//Setup Engine Temperature LEDs

//Setup GPIO_6 for PWM output
gpio_mode(GPIO_6, GPIO_OUTPUT);

ser_printf("System Initialized");
}

/******************************************************************************
 *   Function: loop() - Main execution loop
 *      Pre condition: 
 *          setup() has been executed and system is initialized
 *      Post condition: 
 *          Performs a single iteration of the system's function
 *          Repeates indefinetely unless the board is reset or powered off
 *******************************************************************************/
void loop() 
{
// Task-3: 
// Setup simulated code for the angles from the lab sheet
//int angle;
int angle_values[] = {10, 25, 75, 45, 100, 40, 125, 15, 150, 50, 170};
int num_angles = sizeof(angle_values) / sizeof(angle_values[0]);

// Task-1&2:
auto_brake();

//Bonus Task - Fill this if you wish to try the I2C bonus task
//engine_temp();

//  Task-4:
for (int i = 0; i < num_angles; i++)
  {
  // Here, we set the angle to 180 if the prediction from the DNN is a positive angle
  // and 0 if the prediction is a negative angle.
  // This is so that it is easier to see the movement of the servo.
  // You are welcome to pass the angle values directly to the steering function.
  // If the servo function is written correctly, it should still work,
  // only the movements of the servo will be more subtle
  //if( angle > 0 )
    //{
    //steering(180);
    //}
  //else 
    //{
    //steering(0);
    //}

      int angle = angle_values[i];

      for (int j = 0; j < 50; j++) {
        steering(angle);
      }
           
  // Uncomment the line below to see the actual angles on the servo.
  // Remember to comment out the if-else statement above!
  // steering(angle);
  }
}
