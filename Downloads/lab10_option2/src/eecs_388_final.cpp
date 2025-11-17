/******************************************************************************
 *   Header Includes
 *******************************************************************************/
#include <Arduino.h>        // Gives access to Arduino functions (pinMode, delay, etc.)
#include <stdint.h>         // Standard integer types (uint8_t, uint16_t, etc.)
#include <stdio.h>          // For formatted printing (printf-style)

#include "eecs_388_lib.h"   // EECS 388 provided library (GPIO, UART, etc.)

/******************************************************************************
 *   Constant definitions
 *******************************************************************************/

/* --- BMP180 I2C ADDRESS CONSTANTS (used for temperature sensor) --- */
#define BMP180_ADDR     (0x77)                     // Sensor I2C address
#define BASE_CAL_ADDR   (0xAA)                     // Calibration register start
#define ID_REG_ADDR     (0xD0)
#define CTRL_MEAS_ADDR  (0xF4)
#define ADC_MSB_ADDR    (0xF6)
#define CTRL_TEMP_CMND  (0x2E)
#define BMP180_READ     ((BMP180_ADDR << 1) | 1)  // Create read command
#define BMP180_WRITE    ((BMP180_ADDR << 1) | 0)  // Create write command

/* --- Servo timing constants --- */
#define SERVO_PULSE_MAX (2400)   // 2.4 ms pulse = 180 degrees
#define SERVO_PULSE_MIN (544)    // 0.544 ms pulse = 0 degrees
#define SERVO_PERIOD    (20000)  // 20ms full PWM period
#define MIN_ANGLE       (0)
#define MAX_ANGLE       (180)

/******************************************************************************
 *   Type declarations
 *******************************************************************************/

// Struct holding BMP180 calibration constants from memory
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

bmp180_calibration calib_data;   // Global calibration data for temperature sensor

/*** BONUS LED + SERVO GLOBALS ***/

/*** These are used by the timer interrupt to update LEDs. ***/
volatile uint8_t led_state = 0;    // LED mode (set by auto_brake)
volatile bool red_flash_on = false; // Used for flashing RED LED
volatile uint16_t led_timer_count = 0; // 1ms counter for timing 100ms LED updates

/*** These variables control the servo PWM wave (Bonus 2). ***/
volatile bool servo_pin_state = false;   // Tracks whether servo pin is ON or OFF
volatile uint16_t servo_on_time = 1500;  // Pulse width in microseconds (default = middle)

/******************************************************************************
 *   Local function prototypes
 *******************************************************************************/

static void auto_brake();       // LIDAR + LED logic
static void engine_temp();      // Not implemented
static void steering(int pos);  // Convert angle → servo pulse width

/******************************************************************************
 *   BONUS 2: SERVO PWM INTERRUPT (TIMER1)
 *******************************************************************************/

/*
This creates a 100% hardware-timer-driven PWM.
It toggles the servo pin ON for servo_on_time, then OFF for the rest of 20 ms.

No delay_us().
No blocking.
Servo + LiDAR work simultaneously.
*/

// Timer1 interrupt toggles the servo pin ON and OFF to create PWM
ISR(TIMER1_COMPA_vect)
{
    if (servo_pin_state) {
        gpio_write(GPIO_6, OFF);        // Turn OFF servo pin
        servo_pin_state = false;        // Track state
        OCR1A = (SERVO_PERIOD - servo_on_time) * 2;  // Time until next toggle (OFF time)
    }
    else {
        gpio_write(GPIO_6, ON);         // Turn ON servo pin
        servo_pin_state = true;         // Track state
        OCR1A = servo_on_time * 2;      // Time until next toggle (ON time)
    }
}

/******************************************************************************
 *   BONUS 1: LED TIMER INTERRUPT (TIMER2)
 *******************************************************************************/

/*
Runs every 1 ms -> counts to 100 -> flashes LEDs without blocking.

auto_brake() only sets led_state, and the interrupt updates LEDs.
*/

// Fires every 1 ms -> used to create 100ms LED behavior
ISR(TIMER2_COMPA_vect)
{
    led_timer_count++;              // Increments ms counter

    if (led_timer_count >= 100)     // 100 ms passed
    {
        led_timer_count = 0;        // Resets to repeat

        switch (led_state)
        {
            case 0: // OFF
                gpio_write(GPIO_11, OFF);   // BLUE LED
                gpio_write(GPIO_12, OFF);   // GREEN LED
                gpio_write(GPIO_13, OFF);   // RED LED
                break;

            case 1: // GREEN
                gpio_write(GPIO_11, OFF);
                gpio_write(GPIO_12, ON);    // GREEN ON
                gpio_write(GPIO_13, OFF);
                break;

            case 2: // YELLOW = red + green
                gpio_write(GPIO_11, OFF);
                gpio_write(GPIO_12, ON);
                gpio_write(GPIO_13, ON);
                break;

            case 3: // RED
                gpio_write(GPIO_11, OFF);
                gpio_write(GPIO_12, OFF);
                gpio_write(GPIO_13, ON);    // RED ON
                break;

            case 4: // FLASH RED
                red_flash_on = !red_flash_on;   // Toggle every 100ms
                gpio_write(GPIO_11, OFF);
                gpio_write(GPIO_12, OFF);
                gpio_write(GPIO_13, red_flash_on ? ON : OFF);
                break;
        }
    }
}

/******************************************************************************
 *   AUTO BRAKE FUNCTION
 *******************************************************************************/

static void auto_brake()
{
    uint8_t tfmini_frame[9];   // Buffer to store all 9 TFmini bytes
    uint16_t dist = 0;         // Distance reading
    static uint16_t prev_dist = 0; // For smoothing readings

    // --- Reads TFmini frame until header bytes 0x59 0x59 appear ---
    while (true)
    {
        if (ser_read() == 0x59 && ser_read() == 0x59)
        {
            // Reads remaining 7 bytes
            for (int i = 0; i < 7; i++)
                tfmini_frame[i + 2] = ser_read();

            // Inserts header bytes
            tfmini_frame[0] = 0x59;
            tfmini_frame[1] = 0x59;

            // Computes checksum (sum first 8 bytes)
            uint16_t sum = 0;
            for (int i = 0; i < 8; i++)
                sum += tfmini_frame[i];

            // Validates checksum
            if ((uint8_t)sum == tfmini_frame[8])
            {
                // Extracts distance low + high bytes
                dist = tfmini_frame[2] | (tfmini_frame[3] << 8);
                break;
            }
        }
    }

    // Filters invalid or huge values
    if (dist == 0 || dist > 1200)
        return;

    // This is for simple smoothing to ignore rapid jumps
    if (prev_dist != 0)
    {
        if (abs((int)dist - (int)prev_dist) > 200)
            dist = prev_dist;
        else
            dist = (dist + prev_dist) / 2;
    }
    prev_dist = dist;

    // Prints to serial
    ser_printf("Distance: %d cm\n", dist);

    // LED mode logic (ISR updates GPIO)
    if (dist > 200)
        led_state = 1;       // GREEN
    else if (dist > 100)
        led_state = 2;       // YELLOW
    else if (dist > 60)
        led_state = 3;       // RED
    else
        led_state = 4;       // FLASHING RED
}

/******************************************************************************
 *   ENGINE TEMP (Bonus - optional)
 *******************************************************************************/

static void engine_temp() { }

/******************************************************************************
 *   STEERING FUNCTION
 *******************************************************************************/

 /*
This converts an angle (0–180°) into a pulse width (544–2400 µs).
The servo ISR uses this pulse width on the next cycle.
 */
static void steering(int pos)
{
    // Clamps angle to valid range
    if (pos < MIN_ANGLE) pos = MIN_ANGLE;
    if (pos > MAX_ANGLE) pos = MAX_ANGLE;

    // Converts angle -> pulse width using linear interpolation
    servo_on_time =
        (((unsigned long)pos * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / 180)
        + SERVO_PULSE_MIN;
}

/******************************************************************************
 *   SETUP
 *******************************************************************************/
void setup()
{
    uart_init();      // Initializes UART communication
    i2c_init();       // Initializes I2C module

    // Configures GPIO pins for LEDs & servo
    gpio_mode(GPIO_13, GPIO_OUTPUT); // RED LED
    gpio_mode(GPIO_12, GPIO_OUTPUT); // GREEN LED
    gpio_mode(GPIO_11, GPIO_OUTPUT); // BLUE LED
    gpio_mode(GPIO_6, GPIO_OUTPUT);  // SERVO PWM output

    ser_printf("System Initialized\n");

    // ===== TIMER1 — Servo PWM interrupt =====
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;                 // Resets timer
    OCR1A = SERVO_PERIOD * 2;  // Initial compare value
    TCCR1B |= (1 << WGM12);    // CTC mode
    TCCR1B |= (1 << CS11);     // Prescaler = 8
    TIMSK1 |= (1 << OCIE1A);   // Enables interrupt

    // ===== TIMER2 — LED timer (1ms tick) =====
    TCCR2A = (1 << WGM21);     // CTC mode
    TCCR2B = (1 << CS22);      // Prescaler = 64
    OCR2A = 249;               // Compare for 1ms
    TIMSK2 |= (1 << OCIE2A);   // Enables interrupt
}

/******************************************************************************
 *   LOOP
 *******************************************************************************/
void loop()
{
    // Predefined list of angles for servo to rotate
    static int angle_values[] = {10, 25, 75, 45, 100, 40, 125, 15, 150, 50, 170};
    const int num_angles = sizeof(angle_values) / sizeof(angle_values[0]);

    // Goes through the given angles
    for (int i = 0; i < num_angles; i++)
    {
        int angle = angle_values[i];

        // Repeats 50 times -> gradual motion + ongoing auto brake readings
        for (int j = 0; j < 50; j++)
        {
            auto_brake();   // Reads distance + updates LEDs
            steering(angle); // Updates servo pulse width
        }
    }
}
