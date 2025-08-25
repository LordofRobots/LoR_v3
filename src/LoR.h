/**
 * @file LoR.h
 *
 * Description:
 * This header file is part of the LoR (Lord of Robots) library, designed to provide robust and flexible control
 * systems for robotics applications. It defines the LoRClass for managing complex operations such as motor control
 * and serial communications on ESP32 platforms, facilitating tasks like PWM management and real-time response handling.
 *
 * Key Features:
 * - Motor and Servo Pin Definitions: Facilitates precise control over motor and servo setups.
 * - PWM Configuration: Enables fine-tuning of PWM channels for optimal motor operation.
 * - LoRClass: Serves as the core class for initializing and managing robotic functionalities.
 *
 * Components:
 * - LoRClass: Manages initialization and execution of tasks for serial communication and motor control using FreeRTOS.
 * - Motor and IO Definitions: Provides essential pin configurations for interfacing with hardware.
 *
 * Usage:
 * Include this header in any project files that interact directly with motor controls or require serial communication
 * features provided by the LoR library. Ensure compatibility with ESP32 hardware platform and configure the Arduino
 * environment accordingly.
 *
 * Authors:
 * Dave Barratt
 * Khushi Tailor
 *
 * Date:
 * June 2, 2024
 */

#define LOR_H

#include <Arduino.h>
#include <Bluepad32.h>
#include <Adafruit_NeoPixel.h>

// IO Interface Definitions:
// Defines pin numbers for various outputs and inputs used throughout the robotic system.
#define LED_COUNT 36          // Number of LEDs in the NeoPixel strip
#define LED_DataPin 12        // Data pin for LED output
#define SwitchPin 34          // Input pin for a switch
#define channel1Pin 16        // General purpose I/O pin
#define channel2Pin 17
#define channel3Pin 21
#define channel4Pin 22
#define servo1Pin 16          // Servo control pins
#define servo2Pin 17
#define servo3Pin 21
#define servo4Pin 22

// Motor Pin Definitions:
// Assigns specific GPIO pins to control individual motor drivers.
#define motorPin_M1_A 26
#define motorPin_M1_B 18
#define motorPin_M2_A 14
#define motorPin_M2_B 5
#define motorPin_M3_A 15
#define motorPin_M3_B 33
#define motorPin_M4_A 23
#define motorPin_M4_B 19
#define motorPin_M5_A 25
#define motorPin_M5_B 27
#define motorPin_M6_A 4
#define motorPin_M6_B 32
#define MotorEnablePin 13     // Pin to enable/disable motor operation

#define M1 0
#define M2 1
#define M3 2
#define M4 3
#define M5 4
#define M6 5


// Extern declarations for motor pins arrays to be used across multiple files.
extern const int motorPins_A[];
extern const int motorPins_B[];

// Motor PWM Channel Configuration:
// Defines the PWM channels associated with each motor control pin.
extern const int MOTOR_PWM_Channel_A[];
extern const int MOTOR_PWM_Channel_B[];

// PWM operational parameters:
// Configures the frequency and resolution of PWM signals for motor control.
// Motor PWM Configuration Definitions
extern const int PWM_FREQUENCY;
extern const int PWM_RESOLUTION;
extern const int MAX_SPEED;
extern const int MIN_STARTING_SPEED;
extern const int STOP;

const int Motor_M1_A = 0;
const int Motor_M1_B = 1;
const int Motor_M2_A = 2;
const int Motor_M2_B = 3;
const int Motor_M3_A = 4;
const int Motor_M3_B = 5;
const int Motor_M4_A = 6;
const int Motor_M4_B = 7;
const int Motor_M5_A = 8;
const int Motor_M5_B = 9;
const int Motor_M6_A = 10;
const int Motor_M6_B = 11;

// Define PWM parameters
const double SERVO_PWM_FREQUENCY = 50;   // Frequency of PWM signal (50Hz)
const double SERVO_PWM_RESOLUTION = 12;  // Resolution of PWM signal (12-bit, giving 4095 levels)
const double minPulseWidth = 500;
const double maxPulseWidth = 2500;
const int SERVO_PWM_MIN = int(minPulseWidth / ((1000000 / SERVO_PWM_FREQUENCY) / 4095));
const int SERVO_PWM_MAX = int(maxPulseWidth / ((1000000 / SERVO_PWM_FREQUENCY) / 4095));
const int Servo_Pin[] = { 16, 17, 21, 22 };
const int Servo_CH[] = { 12, 13, 14, 15 };

/**
 * Class LoRClass
 * Core class for managing robotics operations, handling tasks such as motor control,
 * PWM configuration, and serial communication.
 */
class LoRClass {
public:
    LoRClass(uint16_t stackSize = 2048, uint8_t priority = 0);  // Constructor with default stack size and priority

    void begin();  // Starts the necessary tasks and configurations

    // Motor and GPIO configuration functions:
    void INIT_GPIO();    // Initializes all GPIO pins used by the system
    void INIT_PWM();     // Sets up PWM channels for motor control
    void Start_Tone();   // Generates a start-up tone to indicate readiness
    void INIT_BluePad32();
	
	// Initializes Servo motors attached to the LoR_Core AUX_IO Ports.
	void INIT_Servo();
	
	//This servo function performs the required processes to move a RC serco to a desired position, 0 to 180 degrees
	void Servo_SetPosition(int Servo_ID, int Position_Degrees);

    // Adding ControllerManager properties and methods
    ControllerPtr myController = nullptr;
    static void onConnectedController(ControllerPtr ctl);
    static void onDisconnectedController(ControllerPtr ctl);
	
    // Motor control functions:
	
 //Controls the rate of change in motor speed to smooth transitions.
 //This function helps in preventing abrupt changes in motor speed which can be mechanically stressful.
 //
 //@param inputTarget The desired motor speed.
 //@param inputCurrent The current motor speed.
 //@param Rate The acceleration of the change in speed. low value = slow. high value = fast
 //@return The new motor speed adjusted according to the slew rate.
    int SlewRateFunction(int Input_Target, int Input_Current, int Rate);  // Adjusts motor speed to target gradually

//  Output (this is motor speed) = -512 to 512, M_Output (this is which motor output used) = M1 to M6
    void Set_Motor_Output(int Output, int M_Output);  // Sets the motor output based on control inputs




private:
    static void handleSerialInput(void* parameter);  // Handles incoming serial commands
    static const String targetWord;  // Target keyword for special serial commands

    uint16_t stackSize;  // Stack size for FreeRTOS tasks
    uint8_t priority;    // Task priority
	

};

extern LoRClass LoR;  // Extern instance of LoRClass
