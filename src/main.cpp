/*
This is for the L298N motor driver and the INA219 current sensor.
It also uses the switch bounce library
This is set up for WEMOS D1 Mini Lite
*/

#include <JDGlobal.h>
#include <elapsedMillis.h>
#include "Adafruit_INA219.h"
#include <Bounce2.h>
#include <L298N.h>

// L298N
// D0 on L298n = D0 (GPIO16 )
// D1 on L298n = D5 (GPIO14 )

// INA219
// SCL = Pin D1  (GPIO 5)
// SDA = Pin D2  (GPIO 4)

// Switch = Pin D3 (GPIO 0) Internal 10k pull-up

// Create one motor instance
L298N motor(PIN_D0, PIN_D5);

// Create one INA219 instance
Adafruit_INA219 ina219;

// Create one Bounce instance
Bounce limitSwitch = Bounce(); // Instantiate a Bounce object

// Define some steppers and the pins the will use
// AccelStepper stepper1(AccelStepper::FULL4WIRE, PIN_D0, PIN_D6, PIN_D5, PIN_D7); // NOTE!!! Pin order for ULN2003 driver board is IN1, IN3, IN2, IN4

// Timer
elapsedMillis millisTimer; // Keep track of print time

/*
#define kDoOneRevolution 2048             // 2048 steps per revolution for 28BYJ-48
#define kTargetSteps kDoOneRevolution * 16 // 16 revolutions .. moves linear actuator arm 2 cm (8 threads per cm)
#define kMaxSpeed 600                     // Maximum speed in steps per second
*/
void setup()
{
    Serial.begin(115200);
    // Wait for Serial Monitor to be opened
    while (!Serial)
    {
        // do nothing
    }

    // Setup the INA219

    if (!ina219.begin())
    {
        Serial.println("Failed to find INA219 chip");
        while (1)
        {
            delay(10);
        }
    }

    // Setup the limiter switch
    // initialize the pushbutton pin as an pull-up input
    // the pull-up input pin will be HIGH when the switch is open and LOW when the switch is closed.
    limitSwitch.attach(PIN_D3, INPUT_PULLUP); // USE INTERNAL PULL-UP
    limitSwitch.interval(5);                  // interval in ms
}

void loop()
{
    float shuntvoltage = 0;
    float busvoltage = 0;
    float current_mA = 0;
    float loadvoltage = 0;
    float power_mW = 0;
    // create a buffer for sprintf
    char buffer[200];

    limitSwitch.update(); // Update the Bounce instance (YOU MUST DO THIS EVERY LOOP)
    if (limitSwitch.fell())
    {
        Serial.println("switch fell");
    }
    if (limitSwitch.rose())
    {
        Serial.println("switch rose");
    }

    if (millisTimer >= 3000)
    {
        millisTimer = 0;
        // float mSpeed = stepper1.speed();

        // sprintf(buffer, "Speed: %3.2f , distToGo: %ld , currentPos: %ld , targetPos: %ld --- ", mSpeed, stepper1.distanceToGo(), stepper1.currentPosition(), stepper1.targetPosition());
        // Serial.print(buffer);

        shuntvoltage = ina219.getShuntVoltage_mV();
        busvoltage = ina219.getBusVoltage_V();
        current_mA = ina219.getCurrent_mA();
        power_mW = ina219.getPower_mW();
        loadvoltage = busvoltage + (shuntvoltage / 1000);

        // Use sprintf to print the shuntvoltage, busvoltage, current_mA, power_mW and loadvoltage to the buffer

        sprintf(buffer, "shuntV:   %3.2f , BusV: %3.2f ,LoadV: %3.2f, Current_mA: %3.2f ,Power mW: %3.2f  \n", shuntvoltage, busvoltage, loadvoltage, current_mA, power_mW);

        Serial.print(buffer);
    }

    // Tell the motor to go forward (may depend by your wiring)
    motor.forward();
    // print the motor status in the serial monitor
    Serial.print("Is moving = ");
    Serial.println(motor.isMoving());

    delay(3000);

    // Stop
    motor.stop();

    // Alternative method:
    // motor.run(L298N::STOP);

    Serial.print("Is moving = ");
    Serial.println(motor.isMoving());

    delay(3000);

    // Tell the motor to go back (may depend by your wiring)
    motor.backward();

    // Alternative method:
    // motor.run(L298N::BACKWARD);

    Serial.print("Is moving = ");
    Serial.println(motor.isMoving());

    delay(3000);

    // Stop
    motor.stop();

    Serial.print("Is moving = ");
    Serial.println(motor.isMoving());

    delay(3000);
}