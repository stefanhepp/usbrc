// Program used to test the USB Joystick library when used as 
// a Flight Controller on the Arduino Leonardo or Arduino 
// Micro.
//
// Matthew Heironimus
// 2016-05-29 - Original Version
//------------------------------------------------------------

#include <stdint.h>

#include <Joystick.h>

#include "spm_srxl.h"

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

static const uint8_t PIN_BIND = A0;

static const uint32_t UniqueID = 0x53652254;
static const int BusID = 0;

static const int RXLED = 17;

static const bool DEBUG = true;

// UART receive buffer
uint8_t rxBuffer[2 * SRXL_MAX_BUFFER_SIZE];
uint8_t rxBufferIndex = 0;

unsigned long lastRxTime = 0;

int lastBindButtonState = HIGH;

void uartTransmit(uint8_t uartNum, uint8_t* pBuffer, uint8_t bytesToSend)
{
    Serial1.write(pBuffer, bytesToSend);

    if (DEBUG) {
        Serial.print("W: "); Serial.print(bytesToSend); Serial.print(" -");
        for (int i = 0; i < bytesToSend; i++) {
            Serial.print(" ");
            Serial.print(pBuffer[i], 16);
        }
        Serial.print("\n");
    }
}

// User-defined routine to populate telemetry data
void userProvidedFillSrxlTelemetry(SrxlTelemetryData* pTelemetry)
{
    memset(pTelemetry->raw, 0, 16);
    srxlTelemData.sensorID = 0;
}

// User-defined routine to use the provided channel data from the SRXL bus master
void userProvidedReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafeData)
{
    if (DEBUG) {
        Serial.print("msk: "); Serial.print(srxlChData.mask, 16); Serial.print(" ");
        Serial.print(srxlChData.values[0]); Serial.print(" "); Serial.print(srxlChData.values[1]); Serial.print(" ");
        Serial.print(srxlChData.values[2]); Serial.print(" "); Serial.print(srxlChData.values[3]); Serial.print(" ");
        Serial.print(srxlChData.values[4]); Serial.print(" "); Serial.print(srxlChData.values[5]); Serial.print(" ");
        Serial.print(srxlChData.values[6]); Serial.print(" "); Serial.print(srxlChData.values[7]); Serial.print(" ");
        Serial.print(srxlChData.values[8]); Serial.print(" "); Serial.print(srxlChData.values[9]); Serial.print(" ");
        Serial.print(srxlChData.values[10]); Serial.print(" "); Serial.print(srxlChData.values[11]); Serial.print(" ");
        Serial.print(srxlChData.values[12]); Serial.print(" "); Serial.print(srxlChData.values[13]); Serial.print("\n");
    }

    if (srxlChData.mask & (1<< 0)) Joystick.setXAxis(srxlChData.values[0] / 32);
    if (srxlChData.mask & (1<< 1)) Joystick.setYAxis(srxlChData.values[1] / 32);
    if (srxlChData.mask & (1<< 2)) Joystick.setZAxis(srxlChData.values[2] / 32);
    if (srxlChData.mask & (1<< 3)) Joystick.setRxAxis(srxlChData.values[3] / 32);
    if (srxlChData.mask & (1<< 4)) Joystick.setRyAxis(srxlChData.values[4] / 32);
    if (srxlChData.mask & (1<< 5)) Joystick.setRzAxis(srxlChData.values[5] / 32);

    if (srxlChData.mask & (1<< 6)) Joystick.setThrottle(srxlChData.values[6] / 32);
    if (srxlChData.mask & (1<< 7)) Joystick.setRudder(srxlChData.values[7] / 32);
    if (srxlChData.mask & (1<< 8)) Joystick.setSteering(srxlChData.values[8] / 32);
    if (srxlChData.mask & (1<< 9)) Joystick.setAccelerator(srxlChData.values[9] / 32);
    if (srxlChData.mask & (1<< 10)) Joystick.setBrake(srxlChData.values[10] / 32);

    if (srxlChData.mask & (1<<11)) Joystick.setButton(0, srxlChData.values[11] < 32768 ? 0 : 1);
    if (srxlChData.mask & (1<<12)) Joystick.setButton(1, srxlChData.values[12] < 32768 ? 0 : 1);
    if (srxlChData.mask & (1<<13)) Joystick.setButton(2, srxlChData.values[13] < 32768 ? 0 : 1);
    if (srxlChData.mask & (1<<14)) Joystick.setButton(3, srxlChData.values[14] < 32768 ? 0 : 1);
    if (srxlChData.mask & (1<<15)) Joystick.setButton(4, srxlChData.values[15] < 32768 ? 0 : 1);

    // RSSI and frame loss data are also available:
    //if(srxlChData.rssi < -85 || (srxlChData.rssi > 0 && srxlChData.rssi < 10))
    //    EnterLongRangeModeForExample();

    Joystick.sendState();

    if (DEBUG) {
        Serial.print("Update Joystick\n");
    }
}


void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_BIND, INPUT_PULLUP);

    digitalWrite(RXLED, HIGH);

    Serial1.begin(115200, SERIAL_8N1);
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

    srxlInitDevice(SRXL_DEVICE_ID, SRXL_DEVICE_PRIORITY, SRXL_DEVICE_INFO, UniqueID);

    srxlInitBus(BusID, 1, SRXL_SUPPORTED_BAUD_RATES);
}

void loop() {

    // Turn indicator light on.
    //digitalWrite(RXLED, (millis() / 1000) % 2);

    if (Serial1.available()) {
        rxBufferIndex += Serial1.readBytes(&rxBuffer[rxBufferIndex], 2*SRXL_MAX_BUFFER_SIZE - rxBufferIndex);
        
        if (DEBUG) {
            Serial.print("R: "); Serial.print(rxBufferIndex); Serial.print(" - ");
            for (int i = 0; i < rxBufferIndex; i++) {
                Serial.print(rxBuffer[i], 16); Serial.print(" ");
            }
            Serial.print("\n");
        }
    }

    // find header
    int frameStart = 0;
    while (rxBuffer[frameStart] != SPEKTRUM_SRXL_ID && frameStart < rxBufferIndex) {
        frameStart++;
    }

    if (frameStart >= rxBufferIndex || frameStart > SRXL_MAX_BUFFER_SIZE) {
        // No header found, or buffer full of garbage. Reset buffer, clear all data
        rxBufferIndex = 0;
        frameStart = 0;
    }

    if (rxBufferIndex - frameStart > 4) {

        // Get length of the message
        int packetLength = rxBuffer[frameStart + 2];

        if (DEBUG) {
            Serial.print("F: "); Serial.print(frameStart); Serial.print(" - "); Serial.print(packetLength); Serial.print("\n");
        }

        if (rxBufferIndex >= packetLength) {
            
            if (rxBuffer[frameStart+1] == 0xCD && (rxBuffer[frameStart+8] & 0x01) && rxBuffer[frameStart+3] == 0) {
                Serial.print("Ch0: ");
                Serial.print(rxBuffer[frameStart+8], 16); Serial.print(" - ");

                Serial.print(rxBuffer[frameStart+12], 16); Serial.print(" ");
                Serial.print(rxBuffer[frameStart+13], 16);  Serial.print(" ");

                Serial.print(rxBuffer[frameStart+14], 16);  Serial.print(" ");
                Serial.print(rxBuffer[frameStart+15], 16);  Serial.print(" ");
                Serial.print("\n");
            }

            // Try to parse SRXL packet -- this internally calls srxlRun() after packet is parsed and reset timeout
            if(srxlParsePacket(BusID, rxBuffer, packetLength)) {
                digitalWrite(RXLED, LOW);
                // Move any remaining bytes to beginning of buffer (usually 0)
                rxBufferIndex -= packetLength + frameStart;
                memmove(rxBuffer, &rxBuffer[packetLength + frameStart], rxBufferIndex);

                if (DEBUG) {
                    Serial.print("OK\n");
                }

            } else {
                rxBufferIndex = 0;

                if (DEBUG) {
                    Serial.print("NOK\n");
                }
            }

            // Reset timeout
            lastRxTime = millis();
        }
    }

    // Check for timeout
    unsigned long now = millis();

    if (now < lastRxTime) {
      lastRxTime = 0;
    }
    if (now - lastRxTime >= 5) {
        // Tell SRXL state machine that 5 more milliseconds have passed since packet received
        srxlRun(0, now - lastRxTime);
        rxBufferIndex = 0;
        lastRxTime = now;

        digitalWrite(RXLED, HIGH);
    }

    // Check bind button
    int bind = digitalRead(PIN_BIND);

    if (bind == LOW && lastBindButtonState == HIGH) {
        //srxlEnterBind(DSMX_11MS, true);
    }

    lastBindButtonState = bind;
}

