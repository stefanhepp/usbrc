// Program used to test the USB Joystick library when used as 
// a Flight Controller on the Arduino Leonardo or Arduino 
// Micro.
//
// Matthew Heironimus
// 2016-05-29 - Original Version
//------------------------------------------------------------

#include <stdint.h>

#include <Joystick.h>

#include <sbus.h>

/*
 * Setup USB Joystick with 11 axis, 5 buttons.
 */
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 5, 0, 
  true, true, true, true, true, true, 
  true, true, true, true, true);

static const int AXIS_MIN = 0;
static const int AXIS_MAX = 2047; // 65535;
static const int BTN_MID = 1023; // 32768;

static const int RXLED = 17;

static const bool DEBUG = true;

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1);
/* SBUS data */
bfs::SbusData data;

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(RXLED, HIGH);

    Serial1.begin(100000, SERIAL_8E2);
    Serial1.setTimeout(5);

    Joystick.setXAxisRange(AXIS_MIN, AXIS_MAX);
    Joystick.setYAxisRange(AXIS_MIN, AXIS_MAX);
    Joystick.setZAxisRange(AXIS_MIN, AXIS_MAX);
    Joystick.setRxAxisRange(AXIS_MIN, AXIS_MAX);
    Joystick.setRyAxisRange(AXIS_MIN, AXIS_MAX);
    Joystick.setRzAxisRange(AXIS_MIN, AXIS_MAX);

    Joystick.setRudderRange(AXIS_MIN, AXIS_MAX);
    Joystick.setThrottleRange(AXIS_MIN, AXIS_MAX);
    Joystick.setAcceleratorRange(AXIS_MIN, AXIS_MAX);
    Joystick.setBrakeRange(AXIS_MIN, AXIS_MAX);
    Joystick.setSteeringRange(AXIS_MIN, AXIS_MAX);
    
    Joystick.begin(false);

    Serial1.begin(100000, SERIAL_8E2);

    sbus_rx.Begin();
}

void loop() {

    // Turn indicator light on.
    //digitalWrite(RXLED, (millis() / 1000) % 2);

    while (Serial1.available()) {
        Serial.print(" ");
        int data = Serial1.read();
        if (data == 0x0f) {
            Serial.print("\n");
        }
        Serial.print(data, 16);
    }

    /*
    if (sbus_rx.Read()) {
        
        data = sbus_rx.data();
        
        for (int8_t i = 0; i < data.NUM_CH; i++) {
            Serial.print(data.ch[i]);
            Serial.print("\t");
        }
        
        Serial.print(data.lost_frame);
        Serial.print("\t");
        Serial.println(data.failsafe);
    }
        */
}

