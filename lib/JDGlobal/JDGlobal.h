#ifndef JD_GLOBAL_H
#define JD_GLOBAL_H

#define JD_GLOBAL_VERSION 2

#ifndef Arduino_h
    #include <Arduino.h>
#endif
/*
Determines whether the library is active.
If set to 1, ArduinoTrace is active, and prints traces to the serial port.
Set this value to 0 to disable all the traces at once.
*/
#define ARDUINOTRACE_ENABLE 0
#include <ArduinoTrace.h>

// LED blink counts
#define SetupStarted 1
#define SetupCompleted 2
#define LoopStarted 3
#define LoopCompleted 4
#define kErrWiFiFailure 7
#define kErrWiFiGood 8
#define kErrAzServoFailure 9
#define kErrElServoFailure 10
#define kErrGPSInitFailure 11
#define kErrGPSReadFailure 12
#define kWaitingOnGPSFix 13
#define kErrNTPFailure 14
#define kErrNTPSuccess 15

#define PIN_LOW 0  // Pin LOW
#define PIN_HIGH 1 // Pin HIGH

#ifdef ESP8266_CLOCK // ESP8266
    // Wemos D1 Mini Lite
    // See https://docs.wemos.cc/en/latest/d1/d1_mini_lite.html
    #define ESP8266_LED_BUILTIN 2 // Pin D4 mapped to pin GPIO2 of esp8266, control on-board LED

    #define PIN_D0 16 // Pin D0 mapped to pin GPIO16
    #define PIN_D1 5  // Pin D1mapped to pin GPIO5                  //SCL
    #define PIN_D2 4  // Pin D2 mapped to pin GPIO4                 //SDA
    #define PIN_D3 0  // Pin D3 mapped to pin GPIO0                 // 10k Pullup
    #define PIN_D4 2  // Pin D4 mapped to pin GPIO2                 //Builtin LED
    #define PIN_D5 14 // Pin D5 mapped to pin GPIO14                //SCK
    #define PIN_D6 12 // Pin D6 mapped to pin GPIO12                //MISO
    #define PIN_D7 13 // Pin D7 mapped to pin GPIO13                //MOSI
    #define PIN_D8 15 // Pin D8 mapped to pin GPIO15                // 10k Pull down, SS

#endif

#ifdef ESP32
    // See file .../hardware/espressif/esp32/variants/(esp32|doitESP32devkitV1)/pins_arduino.h

    #define ESP32_LED_BUILTIN 2 // Pin D2 mapped to pin GPIO2/ADC12 of ESP32, control on-board LED

    #define PIN_D0 0   // Pin D0 mapped to pin GPIO0/BOOT/ADC11/TOUCH1 of ESP32               (do not use as Boot button)
    #define PIN_D1 1   // Pin D1 mapped to pin GPIO1/TX0 of ESP32                             (use as TX pin)
    #define PIN_D2 2   // Pin D2 mapped to pin GPIO2/ADC12/TOUCH2 of ESP32                    (do not use as it is a strapping pin)
    #define PIN_D3 3   // Pin D3 mapped to pin GPIO3/RX0 of ESP32                             (use as RX pin)
    #define PIN_D4 4   // Pin D4 mapped to pin GPIO4/ADC10/TOUCH0 of ESP32                    (USE THIS)
    #define PIN_D5 5   // Pin D5 mapped to pin GPIO5/SPISS/VSPI_SS of ESP32                   (USE THIS)
    #define PIN_D6 6   // Pin D6 mapped to pin GPIO6/FLASH_SCK of ESP32                       (do not use as flash)
    #define PIN_D7 7   // Pin D7 mapped to pin GPIO7/FLASH_D0 of ESP32                        (do not use as flash)
    #define PIN_D8 8   // Pin D8 mapped to pin GPIO8/FLASH_D1 of ESP32                        (do not use as flash)
    #define PIN_D9 9   // Pin D9 mapped to pin GPIO9/FLASH_D2 of ESP32                        (do not use as flash)
    #define PIN_D10 10 // Pin D10 mapped to pin GPIO10/FLASH_D3 of ESP32                    (do not use as flash)
    #define PIN_D11 11 // Pin D11 mapped to pin GPIO11/FLASH_CMD of ESP32                   (do not use as flash)
    #define PIN_D12 12 // Pin D12 mapped to pin GPIO12/HSPI_MISO/ADC15/TOUCH5/TDI of ESP32  (do not use - platform io debugger)
    #define PIN_D13 13 // Pin D13 mapped to pin GPIO13/HSPI_MOSI/ADC14/TOUCH4/TCK of ESP32  (do not use - platform io debugger)
    #define PIN_D14 14 // Pin D14 mapped to pin GPIO14/HSPI_SCK/ADC16/TOUCH6/TMS of ESP32   (do not use - platform io debugger)
    #define PIN_D15 15 // Pin D15 mapped to pin GPIO15/HSPI_SS/ADC13/TOUCH3/TDO of ESP32    (do not use - platform io debugger)
    #define PIN_D16 16 // Pin D16 mapped to pin GPIO16/TX2 of ESP32                         (USE THIS)
    #define PIN_D17 17 // Pin D17 mapped to pin GPIO17/RX2 of ESP32                         (USE THIS)
    #define PIN_D18 18 // Pin D18 mapped to pin GPIO18/VSPI_SCK of ESP32                    (USE THIS)
    #define PIN_D19 19 // Pin D19 mapped to pin GPIO19/VSPI_MISO of ESP32                   (USE THIS)
    // Pin 20 doe not exist
    #define PIN_D21 21 // Pin D21 mapped to pin GPIO21/SDA of ESP32                         (keep use only for i2c SDA)
    #define PIN_D22 22 // Pin D22 mapped to pin GPIO22/SCL of ESP32                         (keep use only for i2c SCL)
    #define PIN_D23 23 // Pin D23 mapped to pin GPIO23/VSPI_MOSI of ESP32                   (do not use)
    // Pin 24 does not exist
    #define PIN_D25 25 // Pin D25 mapped to pin GPIO25/ADC18/DAC1 of ESP32                  (USE THIS)
    #define PIN_D26 26 // Pin D26 mapped to pin GPIO26/ADC19/DAC2 of ESP32                  (USE THIS)
    #define PIN_D27 27 // Pin D27 mapped to pin GPIO27/ADC17/TOUCH7 of ESP32                (USE THIS)
    // Pin 28 does not exist
    // Pin 29 does not exist
    // Pin 30 does not exist
    // Pin 31 does not exist
    #define PIN_D32 32 // Pin D32 mapped to pin GPIO32/ADC4/TOUCH9 of ESP32                 (USE THIS) (USE for ADC)
    #define PIN_D33 33 // Pin D33 mapped to pin GPIO33/ADC5/TOUCH8 of ESP32                 (USE THIS) (USE for ADC)
    #define PIN_D34 34 // Pin D34 mapped to pin GPIO34/ADC6 of ESP32                        (input only) (USE for ADC)

    // Only GPIO pin < 34 can be used as output. Pins >= 34 can be only inputs
    // See .../cores/esp32/esp32-hal-gpio.h/c
    // #define digitalPinIsValid(pin)          ((pin) < 40 && esp32_gpioMux[(pin)].reg)
    // #define digitalPinCanOutput(pin)        ((pin) < 34 && esp32_gpioMux[(pin)].reg)
    // #define digitalPinToRtcPin(pin)         (((pin) < 40)?esp32_gpioMux[(pin)].rtc:-1)
    // #define digitalPinToAnalogChannel(pin)  (((pin) < 40)?esp32_gpioMux[(pin)].adc:-1)
    // #define digitalPinToTouchChannel(pin)   (((pin) < 40)?esp32_gpioMux[(pin)].touch:-1)
    // #define digitalPinToDacChannel(pin)     (((pin) == 25)?0:((pin) == 26)?1:-1)

    #define PIN_D35 35 // Pin D35 mapped to pin GPIO35/ADC7 of ESP32                        (input only) (USE for ADC)
    #define PIN_D36 36 // Pin D36 mapped to pin GPIO36/ADC0/SVP of ESP32                    (input only) (USE for ADC or internal Hall sensor)
    // Pin 37 does not exist
    // Pin 38 does not exist
    #define PIN_D39 39 // Pin D39 mapped to pin GPIO39/ADC3/SVN of ESP32                    (input only) (USE for ADC or internal Hall sensor)

    #define PIN_RX0 3 // Pin RX0 mapped to pin GPIO3/RX0 of ESP32
    #define PIN_TX0 1 // Pin TX0 mapped to pin GPIO1/TX0 of ESP32

    #define PIN_SCL 22 // Pin SCL mapped to pin GPIO22/SCL of ESP32
    #define PIN_SDA 21 // Pin SDA mapped to pin GPIO21/SDA of ESP32
#endif

#endif // JD_GLOBAL_H
