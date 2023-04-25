#ifndef LOCAL_COMMON_h
#define LOCAL_COMMON_h

#include <Arduino.h>
#include <RP2040.h>
#include <RF24.h>

#define DEBUG_PRINT_INTERVAL 3000

#define ACHAN1_PIN A0
#define ACHAN2_PIN A1
#define ACHAN3_PIN A2
#define ACHAN4_PIN A3

#define DCHAN1_PIN D13
#define DCHAN2_PIN D24
#define DCHAN3_PIN D9
#define DCHAN4_PIN D8
#define DCHAN5_PIN D7
#define DCHAN6_PIN D25

// RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
// Lower PA level when using amplified model
#define RADIO_PA_PLUS_LEVEL RF24_PA_MIN
#define RADIO_PA_STD_LEVEL RF24_PA_HIGH
#define RADIO_RETRIES 15
#define RADIO_RETRY_DELAY 2
#define RADIO_LNA_ENABLE 1
#define RADIO_AUTO_ACK true
#define RADIO_DYNAMIC_PAYLOAD true
#define RADIO_PIPE 1
#define RADIO_DEFAULT_CHANNEL 110
#define RADIO_BAUD_RATE_ENUM_VAL 2 // (2) represents 250 kbps
#define RADIO_MAX_TIME_SINCE_LAST_RECEPTION 2000
#define RADIO_PAYLOAD_SIZE 7
#define RADIO_NUM_ANALOG_CHANNELS 4
#define RADIO_NUM_DIGITAL_CHANNELS 6
#define RADIO_CH1_BYTE 0
#define RADIO_CH2_BYTE 1
#define RADIO_CH3_BYTE 2
#define RADIO_CH4_BYTE 3
#define RADIO_SGN_BYTE 4
#define RADIO_DI_BYTE 5

#define RADIO_PROT_MAGIC_BYTE 6
#define RADIO_MAGIC_BITS 0
#define RADIO_MAGIC_BITS_MASK 0xf
#define RADIO_MAGIC_VALUE 5
#define RADIO_PROT_BITS 4
#define RADIO_PROT_BITS_MASK 0xf
#define RADIO_PROT_VALUE 1

#define RADIO_RESPONSE_PAYLOAD_SIZE 6
#define RESPONSE_BATTERY_BYTE 0
#define RESPONSE_MODE_BYTE 1
#define RESPONSE_FMODE_BYTE 2
#define RESPONSE_FVEL_BYTE 3
#define RESPONSE_PING0_BYTE 4
#define RESPONSE_PING1_BYTE 5
#define LOW_BATTERY_LEVEL_CV 700
#define CRITICAL_BATTERY_LEVEL_CV 680
#define BATTERY_CAL_FACTOR 3.14f
#define ADC_AREF_FACTOR 3.3f

#define MODE_BATTLE 1
#define MODE_SENTRY 2
#define MODE_DRIVING 3
#define FMODE_SINGLE 1
#define FMODE_3BURST 2
#define FVEL_LOW 1
#define FVEL_HIGH 2


// RADIO Analog Sign and Digital Input bit definition
#define RADIO_CH1_SIGN_BIT 0
#define RADIO_CH2_SIGN_BIT 1
#define RADIO_CH3_SIGN_BIT 2
#define RADIO_CH4_SIGN_BIT 3
#define RADIO_DI1_BIT 0
#define RADIO_DI2_BIT 1
#define RADIO_DI3_BIT 2
#define RADIO_DI4_BIT 3
#define RADIO_DI5_BIT 4
#define RADIO_DI6_BIT 5

#define ANALOG_READ_RESOLUTION_BITS 8
#define ADDL_AN_DEADBAND 4
#define ANALOG_READ_MAX_VAL ((0x1 << ANALOG_READ_RESOLUTION_BITS)-1)

struct analogChannel {
  uint8_t pin;
  uint8_t txValueBytePosition;
  uint8_t txSignBytePosition;
  uint8_t txSignBitPosition;
  uint8_t txValue;
  uint8_t txSignValueIsNegative;
  uint8_t rawValue;
  uint8_t deadMax;
  uint8_t deadMin;
  bool flipAxis;
  bool isCenterZero;
};

struct digitalChannel {
  uint8_t pin;
  uint8_t txValueBytePosition;
  uint8_t txValueBitPosition;
  uint8_t txDebouncedValue;
  uint8_t rawValue;
  bool invertValue;
};


//RadioCommAddress through which two modules communicate.
extern const byte RadioCommAddress[];
extern uint8_t radio_data[];
extern uint8_t radio_response_data[];
extern analogChannel analogChannels[];
extern digitalChannel digitalChannels[];
void initAnalogChannelStructs(void);
void initDigitalChannelStructs(void);
float getVoltsFromRawBatVal(uint16_t);
void doCommonRadioConfig(RF24 &);

#endif