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
#define BMP180_ADDR     (0x77)
#define BASE_CAL_ADDR   (0xAA)
#define ID_REG_ADDR     (0xD0)
#define CTRL_MEAS_ADDR  (0xF4)
#define ADC_MSB_ADDR    (0xF6)
#define CTRL_TEMP_CMND  (0x2E)
#define BMP180_READ     ((BMP180_ADDR << 1) | 1)
#define BMP180_WRITE    ((BMP180_ADDR << 1) | 0)

/* Servo constants */
#define SERVO_PULSE_MAX (2400)
#define SERVO_PULSE_MIN (544)
#define SERVO_PERIOD    (20000)
#define MIN_ANGLE       (0)
#define MAX_ANGLE       (180)

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

/*** BONUS LED + SERVO GLOBALS ***/
volatile uint8_t led_state = 0;    // 0=OFF, 1=GREEN, 2=YELLOW, 3=RED, 4=FLASH
volatile bool red_flash_on = false;
volatile uint16_t led_timer_count = 0;

volatile bool servo_pin_state = false;
volatile uint16_t servo_on_time = 1500;  // microseconds

/******************************************************************************
 *   Local function prototypes
 *******************************************************************************/
static void auto_brake();
static void engine_temp();
static void steering(int pos);

/******************************************************************************
 *   BONUS 2: SERVO PWM INTERRUPT (TIMER1)
 *******************************************************************************/
ISR(TIMER1_COMPA_vect)
{
    if (servo_pin_state) {
        gpio_write(GPIO_6, OFF);
        servo_pin_state = false;
        OCR1A = (SERVO_PERIOD - servo_on_time) * 2;
    } 
    else {
        gpio_write(GPIO_6, ON);
        servo_pin_state = true;
        OCR1A = servo_on_time * 2;
    }
}

/******************************************************************************
 *   BONUS 1: LED TIMER INTERRUPT (TIMER2)
 *******************************************************************************/
ISR(TIMER2_COMPA_vect)
{
    led_timer_count++;

    if (led_timer_count >= 100) { // 100 × 1ms = 100ms
        led_timer_count = 0;

        switch (led_state)
        {
            case 0: // OFF
                gpio_write(GPIO_11, OFF);
                gpio_write(GPIO_12, OFF);
                gpio_write(GPIO_13, OFF);
                break;

            case 1: // GREEN
                gpio_write(GPIO_11, OFF);
                gpio_write(GPIO_12, ON);
                gpio_write(GPIO_13, OFF);
                break;

            case 2: // YELLOW
                gpio_write(GPIO_11, OFF);
                gpio_write(GPIO_12, ON);
                gpio_write(GPIO_13, ON);
                break;

            case 3: // RED
                gpio_write(GPIO_11, OFF);
                gpio_write(GPIO_12, OFF);
                gpio_write(GPIO_13, ON);
                break;

            case 4: // FLASH RED
                red_flash_on = !red_flash_on;
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
    uint8_t tfmini_frame[9];
    uint16_t dist = 0;
    static uint16_t prev_dist = 0;

    // Read until valid TFmini frame
    while (true)
    {
        if (ser_read() == 0x59 && ser_read() == 0x59)
        {
            for (int i = 0; i < 7; i++)
                tfmini_frame[i + 2] = ser_read();

            tfmini_frame[0] = 0x59;
            tfmini_frame[1] = 0x59;

            uint16_t sum = 0;
            for (int i = 0; i < 8; i++)
                sum += tfmini_frame[i];

            if ((uint8_t)sum == tfmini_frame[8])
            {
                dist = tfmini_frame[2] | (tfmini_frame[3] << 8);
                break;
            }
        }
    }

    // Filter garbage
    if (dist == 0 || dist > 1200)
        return;

    // Smooth reading
    if (prev_dist != 0)
    {
        if (abs((int)dist - (int)prev_dist) > 200)
            dist = prev_dist;
        else
            dist = (dist + prev_dist) / 2;
    }
    prev_dist = dist;

    ser_printf("Distance: %d cm\n", dist);

    // Set LED state (ISR handles actual GPIO)
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
static void engine_temp() {}

/******************************************************************************
 *   STEERING FUNCTION
 *******************************************************************************/
static void steering(int pos)
{
    if (pos < MIN_ANGLE) pos = MIN_ANGLE;
    if (pos > MAX_ANGLE) pos = MAX_ANGLE;

    // Convert angle to pulse width
    servo_on_time =
        (((unsigned long)pos * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / 180)
        + SERVO_PULSE_MIN;
}

/******************************************************************************
 *   SETUP
 *******************************************************************************/
void setup() 
{
    uart_init();
    i2c_init();

    gpio_mode(GPIO_13, GPIO_OUTPUT); // RED
    gpio_mode(GPIO_12, GPIO_OUTPUT); // GREEN
    gpio_mode(GPIO_11, GPIO_OUTPUT); // BLUE
    gpio_mode(GPIO_6, GPIO_OUTPUT);  // SERVO

    ser_printf("System Initialized\n");

    // ===== TIMER1 — SERVO PWM (20ms) =====
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = SERVO_PERIOD * 2;
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11);
    TIMSK1 |= (1 << OCIE1A);

    // ===== TIMER2 — LED TIMER =====
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22);
    OCR2A = 249;
    TIMSK2 |= (1 << OCIE2A);
}

/******************************************************************************
 *   LOOP
 *******************************************************************************/
void loop() 
{
    static int angle_values[] = {10, 25, 75, 45, 100, 40, 125, 15, 150, 50, 170};
    const int num_angles = sizeof(angle_values) / sizeof(angle_values[0]);

    for (int i = 0; i < num_angles; i++)
    {
        int angle = angle_values[i];

        for (int j = 0; j < 50; j++)   // speed control
        {
            auto_brake();
            steering(angle);
        }
    }
}
