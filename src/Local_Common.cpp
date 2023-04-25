#include "Local_Common.h"

uint8_t radio_data[RADIO_PAYLOAD_SIZE] = {};
uint8_t radio_response_data[RADIO_RESPONSE_PAYLOAD_SIZE] = {};
analogChannel analogChannels[RADIO_NUM_ANALOG_CHANNELS] = {};
digitalChannel digitalChannels[RADIO_NUM_DIGITAL_CHANNELS] = {};
const byte RadioCommAddress[6] = "00001";

void initAnalogChannelStructs(){
  analogChannels[0].pin = ACHAN1_PIN;
  analogChannels[0].txValueBytePosition = RADIO_CH1_BYTE;
  analogChannels[0].txSignBytePosition = RADIO_SGN_BYTE;
  analogChannels[0].txSignBitPosition = RADIO_CH1_SIGN_BIT;
  analogChannels[0].isCenterZero = true;
  analogChannels[0].flipAxis = true;
  analogChannels[0].deadMax = 0;
  analogChannels[0].deadMin = ANALOG_READ_MAX_VAL;

  analogChannels[1].pin = ACHAN2_PIN;
  analogChannels[1].txValueBytePosition = RADIO_CH2_BYTE;
  analogChannels[1].txSignBytePosition = RADIO_SGN_BYTE;
  analogChannels[1].txSignBitPosition = RADIO_CH2_SIGN_BIT;
  analogChannels[1].isCenterZero = true;
  analogChannels[1].flipAxis = false;
  analogChannels[1].deadMax = 0;
  analogChannels[1].deadMin = ANALOG_READ_MAX_VAL;

  analogChannels[2].pin = ACHAN3_PIN;
  analogChannels[2].txValueBytePosition = RADIO_CH3_BYTE;
  analogChannels[2].txSignBytePosition = RADIO_SGN_BYTE;
  analogChannels[2].txSignBitPosition = RADIO_CH3_SIGN_BIT;
  analogChannels[2].isCenterZero = true;
  analogChannels[2].flipAxis = false;
  analogChannels[2].deadMax = 0;
  analogChannels[2].deadMin = ANALOG_READ_MAX_VAL;

  analogChannels[3].pin = ACHAN4_PIN;
  analogChannels[3].txValueBytePosition = RADIO_CH4_BYTE;
  analogChannels[3].txSignBytePosition = RADIO_SGN_BYTE;
  analogChannels[3].txSignBitPosition = RADIO_CH4_SIGN_BIT;
  analogChannels[3].isCenterZero = true;
  analogChannels[3].flipAxis = false;
  analogChannels[3].deadMax = 0;
  analogChannels[3].deadMin = ANALOG_READ_MAX_VAL;
}
void initDigitalChannelStructs(){
  digitalChannels[0].pin = DCHAN1_PIN;
  digitalChannels[0].txValueBytePosition = RADIO_DI_BYTE;
  digitalChannels[0].txValueBitPosition = RADIO_DI1_BIT;
  digitalChannels[0].invertValue = true;

  digitalChannels[1].pin = DCHAN2_PIN;
  digitalChannels[1].txValueBytePosition = RADIO_DI_BYTE;
  digitalChannels[1].txValueBitPosition = RADIO_DI2_BIT;
  digitalChannels[1].invertValue = true;
  
  digitalChannels[2].pin = DCHAN3_PIN;
  digitalChannels[2].txValueBytePosition = RADIO_DI_BYTE;
  digitalChannels[2].txValueBitPosition = RADIO_DI3_BIT;
  digitalChannels[2].invertValue = true;
  
  digitalChannels[3].pin = DCHAN4_PIN;
  digitalChannels[3].txValueBytePosition = RADIO_DI_BYTE;
  digitalChannels[3].txValueBitPosition = RADIO_DI4_BIT;
  digitalChannels[3].invertValue = true;

  digitalChannels[4].pin = DCHAN5_PIN;
  digitalChannels[4].txValueBytePosition = RADIO_DI_BYTE;
  digitalChannels[4].txValueBitPosition = RADIO_DI5_BIT;
  digitalChannels[4].invertValue = true;

  digitalChannels[5].pin = DCHAN6_PIN;
  digitalChannels[5].txValueBytePosition = RADIO_DI_BYTE;
  digitalChannels[5].txValueBitPosition = RADIO_DI6_BIT;
  digitalChannels[5].invertValue = true;
}
float getVoltsFromRawBatVal(uint16_t rawBatVal){
    return (rawBatVal * ADC_AREF_FACTOR * BATTERY_CAL_FACTOR)/ANALOG_READ_MAX_VAL;
}
void doCommonRadioConfig(RF24& radio){
    if(radio.isPVariant()){
        radio.setPALevel(RADIO_PA_PLUS_LEVEL, RADIO_LNA_ENABLE);
    }
    else{
        radio.setPALevel(RADIO_PA_STD_LEVEL);
    }
    radio.setAutoAck(RADIO_AUTO_ACK);
    radio.setRetries(RADIO_RETRY_DELAY, RADIO_RETRIES);
    if(RADIO_DYNAMIC_PAYLOAD){
        radio.enableDynamicPayloads();
        radio.enableAckPayload();
    }
    else{
        radio.disableDynamicPayloads();
        radio.disableAckPayload();
        radio.setPayloadSize(RADIO_PAYLOAD_SIZE);
    }
    radio.setDataRate((rf24_datarate_e)RADIO_BAUD_RATE_ENUM_VAL);
    radio.setChannel(RADIO_DEFAULT_CHANNEL);
}
