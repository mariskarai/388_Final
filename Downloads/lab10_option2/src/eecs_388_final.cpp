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
    uint8_t tfmini_frame[9];
    uint16_t dist = 0;
    static uint16_t prev_dist = 0;   // last good distance for smoothing
    static bool flash_on = false;    // for non-blocking flashing red

    // Keep reading until we get a valid TFmini frame
    while (true)
    {
        // Look for header 0x59 0x59
        if (ser_read() == 0x59 && ser_read() == 0x59)
        {
            // Read remaining 7 bytes of the frame
            for (int i = 0; i < 7; i++)
            {
                tfmini_frame[i + 2] = ser_read();
            }

            // Insert header bytes
            tfmini_frame[0] = 0x59;
            tfmini_frame[1] = 0x59;

            // Compute checksum (sum of first 8 bytes)
            uint16_t sum = 0;
            for (int i = 0; i < 8; i++)
            {
                sum += tfmini_frame[i];
            }

            // Validate checksum
            if ((uint8_t)sum == tfmini_frame[8])
            {
                // Valid frame → extract distance
                dist = tfmini_frame[2] | (tfmini_frame[3] << 8);
                break;
            }
        }
    }

    // Basic sanity filter: ignore clearly bad values
    if (dist == 0 || dist > 1200)
    {
        return;  // don't update LEDs if distance is garbage
    }

    // Simple smoothing: ignore huge jumps & average with previous
    if (prev_dist != 0)
    {
        // If jump is > 200 cm, treat it as noise and stick with previous
        if (dist > prev_dist + 200 || dist + 200 < prev_dist)
        {
            dist = prev_dist;
        }
        else
        {
            // Light smoothing: average current + previous
            dist = (dist + prev_dist) / 2;
        }
    }
    prev_dist = dist;

    // Debug print
    ser_printf("Distance: %d cm\n", dist);

    // ====== LED LOGIC (GPIO version) ======

    gpio_write(GPIO_11, OFF);  // Blue always off in your logic

    if (dist > 200)
    {
        // Green
        gpio_write(GPIO_12, ON);
        gpio_write(GPIO_13, OFF);
    }
    else if (dist > 100)  // 100 < dist <= 200
    {
        // Yellow (Red + Green)
        gpio_write(GPIO_12, ON);
        gpio_write(GPIO_13, ON);
    }
    else if (dist > 60)   // 60 < dist <= 100
    {
        // Red
        gpio_write(GPIO_12, OFF);
        gpio_write(GPIO_13, ON);
    }
    else  // dist <= 60
    {
        // Flashing red (non-blocking style)
        gpio_write(GPIO_12, OFF);

        if (flash_on)
        {
            gpio_write(GPIO_13, ON);
        }
        else
        {
            gpio_write(GPIO_13, OFF);
        }

        // Toggle flash state each time auto_brake() is called
        flash_on = !flash_on;
        // NOTE: no delay_ms() here → servo and rest of loop keep running
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

 pos = constrain(pos, MIN_ANGLE, MAX_ANGLE);
 int pwm_value = SERVO_PULSE_MIN + (((SERVO_PULSE_MAX - SERVO_PULSE_MIN)/ ))

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
    // Task-3: Setup simulated angles from lab sheet
    static int angle_values[] = {10, 25, 75, 45, 100, 40, 125, 15, 150, 50, 170};
    const int num_angles = sizeof(angle_values) / sizeof(angle_values[0]);

    // Task-1&2: Auto brake
   

    // Bonus Task
    // engine_temp();

    // Task-4: Loop through all angles
    for (int i = 0; i < num_angles; i++)
    {        
        // Call steering() 50 times for each angle
        int angle = angle_values[i];
        for (int j = 0; j < 50; j++) 
        {
             auto_brake();
            steering(angle);
        }
    }
        
}
