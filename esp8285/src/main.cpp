#include <Arduino.h>
#include <RadioLib.h>

#include "LowPassFilter.h"

// The RadioMaster BR3 has the following connections:
// Serial_RX pin:  3
// Serial_TX pin:  1
// Radio_DIO0 pin: 4
// Radio_DIO1 pin:  5
// Radio_MISO pin: 12
// Radio_MOSI pin: 13
// Radio_NSS pin:  15
// Radio_RST pin:   2
// Radio_SCK pin:  14
// Radio_DCDC: true
// Radio_RFO_HF: true
// Ant_Ctrl:    9
// Power_TXEN: 10
// Power_LNA_GAIN: 16
// Power_MIN:  3
// Power_HIGH: 5
// Power_MAX:  5
// Power_DEFAULT: 3
// Power_CONTROL: 0
// Power_VALUES: [0,4,10]
// LED: 16
// Button: 0
//
// It uses a Sige SE2435l 422cv

#define ANT_CTRL 9
#define POWER_TXEN 10
#define POWER_LNA_GAIN 16
#define POWER_MIN 3
#define POWER_HIGH 5
#define POWER_MAX 5
#define POWER_DEFAULT 3
#define POWER_CONTROL 0
#define POWER_VALUES {0, 4, 10}
#define LED 16
#define BUTTON 0

static inline void IRAM_ATTR switchAntenna();

// Antenna Data
#define GPIO_PIN_ANT_CTRL 9
static bool CURRENT_ANTENNA_LEFT = true;
static LPF LEFT_RSSI(3, 5);
static LPF RIGHT_RSSI(3, 5);


// SX1278 has the following connections:
// NSS pin:   15
// DIO0 pin:  4
// RESET pin: 2
// DIO1 pin:  5
const static uint32_t cs = 15;
const static uint32_t irq = 4;
const static uint32_t rst = 2;
const static uint32_t gpio = 5;
SX1278 radio = new Module(cs, irq, rst, gpio);

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
//   ICACHE_RAM_ATTR
    IRAM_ATTR
#endif
void setFlag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
}

const static float FREQ = 905200000;
const static float band_width = 250.0;
const static uint8_t spreading_factor = 9;
const static uint8_t coding_rate = 7;
const static uint8_t sync_word = 0x12;
const static uint8_t power = 10;
const static uint16_t preamble_length = 4;
const static uint8_t lna_gain = 0;

void setup() {
    Serial.begin(9600);

    // Setup external power amplifier
    pinMode(POWER_TXEN, OUTPUT);
    pinMode(POWER_LNA_GAIN, OUTPUT);

    Serial.print(F("[SX1278] Initializing ... "));
    int state = radio.begin(
        FREQ, 
        band_width,
        spreading_factor,
        coding_rate,
        sync_word,
        power,
        preamble_length,
        lna_gain
    );
    if (state == RADIOLIB_ERR_NONE) {

        Serial.println(F("success!"));

    } else {

        Serial.print(F("failed, code "));
        
        Serial.println(state);

        while (true) { 
            delay(10); 
        }
    }
    
    // set the function that will be called
    // when packet transmission is finished
    radio.setPacketSentAction(setFlag);

    // start transmitting the first packet
    Serial.print(F("[SX1278] Sending first packet ... "));

    // you can transmit C-string or Arduino string up to
    // 255 characters long
    transmissionState = radio.startTransmit("Hello World!");

    // you can also transmit byte array up to 255 bytes long
    /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
    transmissionState = radio.startTransmit(byteArr, 8);
    */

    LEFT_RSSI.init(0);
    RIGHT_RSSI.init(0);
}

// counter to keep track of transmitted packets
int count = 0;

void loop() {

    if (Serial.available() > 0) {
        String str = Serial.readString();
        Serial.print(F("[SX1278] Sending another packet ... "));
        transmissionState = radio.startTransmit(str);
    }

    // check if the previous transmission finished
    if(transmittedFlag) {
        // reset flag
        transmittedFlag = false;

        if (transmissionState == RADIOLIB_ERR_NONE) {
            // packet was successfully sent
            Serial.println(F("transmission finished!"));

            // NOTE: when using interrupt-driven transmit method,
            //       it is not possible to automatically measure
            //       transmission data rate using getDataRate()
        } else {
            Serial.print(F("failed, code "));
            Serial.println(transmissionState);
        }

        // clean up after transmission is finished
        // this will ensure transmitter is disabled,
        // RF switch is powered down etc.
        radio.finishTransmit();

        // wait a second before transmitting again
        delay(1000);

        // send another one
        Serial.print(F("[SX1278] Sending another packet ... "));

        // you can transmit C-string or Arduino string up to
        // 255 characters long
        String str = "Hello World! #" + String(count++);
        transmissionState = radio.startTransmit(str);
        if (CURRENT_ANTENNA_LEFT) {
            LEFT_RSSI.update(radio.getRSSI());
        } else {
            RIGHT_RSSI.update(radio.getRSSI());
        }

        // Switch antenna


        // you can also transmit byte array up to 255 bytes long
        /*
        byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                            0x89, 0xAB, 0xCD, 0xEF};
        transmissionState = radio.startTransmit(byteArr, 8);
        */
    }
}

static inline void IRAM_ATTR switchAntenna()
{
    CURRENT_ANTENNA_LEFT = !CURRENT_ANTENNA_LEFT;
    (CURRENT_ANTENNA_LEFT) ? LEFT_RSSI.reset() : RIGHT_RSSI.reset();
    digitalWrite(GPIO_PIN_ANT_CTRL, CURRENT_ANTENNA_LEFT);
}

void setTXPower(int power)
{
    if (power < 0 || power > 20) {
        Serial.println(F("[SX1278] Invalid TX power!"));
        return;
    }
    radio.setOutputPower(power);
}