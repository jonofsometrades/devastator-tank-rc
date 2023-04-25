#include "Local_Common.h"
#include "Events.h"
#include <Arduino.h>
#include <RP2040.h>
#include <SPI.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

#define DEBUG_ENABLED
#define DISPLAY_UPDATE_INTERVAL 250
#define RADIO_TX_INTERVAL 10
#define LINE1_STATUS "A0:% 4d A2:% 4d\n"
#define LINE2_STATUS "A1:% 4d A3:% 4d\n"
#define LINE_DEAD_ZONE "A%d max:% 3d min:% 3d\n"
#define TX_STAT  "Tx Fail: % 5.2f%%\n"

// Radio related defines
#define RADIO_CS_PIN D1
#define RADIO_CE_PIN D0

unsigned long loop_start_time;
unsigned long last_loop_start_time;
unsigned long last_print_time;
unsigned long last_display_update_time;
unsigned long radio_last_received;
bool debug_print_this_loop;
unsigned long tx_fail_counter = 0;
unsigned long rx_payload_counter = 0;
unsigned long tx_attempt_counter = 0;
float tx_percent_fail_disp = 0.0;
uint8_t ledstat = 0;
uint8_t request_channel = 0;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

Adafruit_NeoPixel led = Adafruit_NeoPixel(1, PIN_NEOPIXEL , NEO_GRB + NEO_KHZ800);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

RF24 radio(RADIO_CS_PIN, RADIO_CE_PIN);
EventScheduler scheduler;
#define IMG_BIO_WIDTH 48
#define IMG_BIO_HEIGHT 48


// '480px-Biohazard_symbol', 48x48px
const unsigned char biohazard [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x01, 
	0x80, 0x01, 0x80, 0x00, 0x00, 0x03, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x06, 0x00, 0x00, 0x60, 0x00, 
	0x00, 0x0e, 0x00, 0x00, 0x70, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x70, 0x00, 0x00, 0x1c, 0x00, 0x00, 
	0x38, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x38, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 
	0x01, 0x80, 0x3c, 0x00, 0x00, 0x3e, 0x1f, 0xf8, 0x7c, 0x00, 0x00, 0x3e, 0x7f, 0xfe, 0x7c, 0x00, 
	0x00, 0x3e, 0x7f, 0xfe, 0x7c, 0x00, 0x00, 0x3f, 0x70, 0x0e, 0xfc, 0x00, 0x00, 0x7f, 0x80, 0x01, 
	0xfe, 0x00, 0x01, 0xff, 0xc0, 0x03, 0xff, 0x80, 0x03, 0xff, 0xe0, 0x07, 0xff, 0xc0, 0x07, 0xff, 
	0xfc, 0x3f, 0xff, 0xe0, 0x0f, 0xff, 0xfe, 0x7f, 0xff, 0xf0, 0x1f, 0x81, 0xfe, 0x7f, 0x81, 0xf8, 
	0x1e, 0x0c, 0x7c, 0x3e, 0x30, 0x78, 0x3c, 0x0e, 0x38, 0x1c, 0x70, 0x3c, 0x38, 0x0e, 0x18, 0x18, 
	0x70, 0x1c, 0x30, 0x0e, 0x00, 0x00, 0x70, 0x0c, 0x20, 0x0e, 0x06, 0x60, 0x70, 0x04, 0x60, 0x0e, 
	0x07, 0xe0, 0x70, 0x04, 0x60, 0x0f, 0x07, 0xe0, 0xf0, 0x06, 0x40, 0x07, 0x83, 0xc1, 0xe0, 0x02, 
	0x00, 0x03, 0xc3, 0xc3, 0xc0, 0x00, 0x00, 0x03, 0xe3, 0xc7, 0xc0, 0x00, 0x00, 0x01, 0xf3, 0xcf, 
	0x80, 0x00, 0x00, 0x00, 0xf7, 0xef, 0x00, 0x00, 0x00, 0x00, 0x37, 0xee, 0x00, 0x00, 0x00, 0x00, 
	0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 
	0x04, 0x00, 0x3f, 0xfc, 0x00, 0x20, 0x03, 0x00, 0xff, 0xff, 0x00, 0xc0, 0x00, 0xff, 0xfe, 0x7f, 
	0xff, 0x00, 0x00, 0x3f, 0xf8, 0x1f, 0xfc, 0x00, 0x00, 0x0f, 0xc0, 0x03, 0xf0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void initialDisplay(void) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.drawBitmap(
    0, 0, biohazard,
    IMG_BIO_WIDTH, IMG_BIO_HEIGHT, 1);
  display.setCursor(IMG_BIO_WIDTH+4, 8);
  display.setTextSize(2); // Draw 2X-scale text
  display.println(F("bio-"));
  display.setCursor(IMG_BIO_WIDTH+4, 28);
  display.println(F("hazard"));
  display.setTextSize(1);
  display.setCursor(0,(display.height()-10));
  display.println(F("Built for Liam Q."));
  display.display();      // Show initial text

}
void configureDisplay(){
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }
    display.setRotation(2);
    display.clearDisplay();

    initialDisplay();
}


void initChannelPins(){
  for (int i = 0; i < RADIO_NUM_ANALOG_CHANNELS; i++){
    pinMode(analogChannels[i].pin, INPUT);
  }
  for (int i = 0; i < RADIO_NUM_DIGITAL_CHANNELS; i++){
    pinMode(digitalChannels[i].pin, INPUT_PULLUP);
  }
}
void readAnalogValues(){
  for (int i = 0; i < RADIO_NUM_ANALOG_CHANNELS; i++){
    if(analogChannels[i].flipAxis){
      analogChannels[i].rawValue = (ANALOG_READ_MAX_VAL - analogRead(analogChannels[i].pin));
    }
    else{
      analogChannels[i].rawValue = analogRead(analogChannels[i].pin);
    }
  }
}
void initDeadZone(){
  for (int j = 0; j < 100; j++){
    readAnalogValues();
    for (int i = 0; i < RADIO_NUM_ANALOG_CHANNELS; i++){
      if (analogChannels[i].rawValue < analogChannels[i].deadMin)
        analogChannels[i].deadMin = analogChannels[i].rawValue;
      if (analogChannels[i].rawValue > analogChannels[i].deadMax)
        analogChannels[i].deadMax = analogChannels[i].rawValue;
    }
    delay(10);
  }
  for (int i = 0; i < RADIO_NUM_ANALOG_CHANNELS; i++){
    analogChannels[i].deadMin -= ADDL_AN_DEADBAND;
    analogChannels[i].deadMax += ADDL_AN_DEADBAND;
  }
}
void doInputConditioning(){
    uint8_t deadMin, deadMax, val;
    for (int i = 0; i < RADIO_NUM_ANALOG_CHANNELS; i++){
      val = analogChannels[i].txValue;
      deadMin = analogChannels[i].deadMin;
      deadMax = analogChannels[i].deadMax;
      analogChannels[i].txSignValueIsNegative = 0;
      if(analogChannels[i].isCenterZero){
        if(val >= deadMin && val <= deadMax){
          val = 0;
        }
        else if(val > deadMax){
          val = map(val, deadMax, ANALOG_READ_MAX_VAL, 0, 127);
        }
        else if (val < deadMin)
        {
          val = map(val, deadMin, 0, 0, 127);
          analogChannels[i].txSignValueIsNegative = 1;
        }
        analogChannels[i].txValue = val;
      }
    }
}
void PackRadioData(){
  radio_data[RADIO_PROT_MAGIC_BYTE] |= (RADIO_MAGIC_VALUE << RADIO_MAGIC_BITS);
  radio_data[RADIO_PROT_MAGIC_BYTE] |= (RADIO_PROT_VALUE << RADIO_PROT_BITS);
  for (int i = 0; i < RADIO_NUM_ANALOG_CHANNELS; i++){
    radio_data[analogChannels[i].txValueBytePosition] = analogChannels[i].txValue;
    radio_data[analogChannels[i].txSignBytePosition ] &= ~(1 << analogChannels[i].txSignBitPosition);
    radio_data[analogChannels[i].txSignBytePosition ] |= (analogChannels[i].txSignValueIsNegative << analogChannels[i].txSignBitPosition);
  }
  for (int i = 0; i < RADIO_NUM_DIGITAL_CHANNELS; i++){
    radio_data[digitalChannels[i].txValueBytePosition ] &= ~(1 << digitalChannels[i].txValueBitPosition);
    radio_data[digitalChannels[i].txValueBytePosition ] |= (digitalChannels[i].txDebouncedValue << digitalChannels[i].txValueBitPosition);
  }
}


void processAnalogValues(){
  for (int i = 0; i < RADIO_NUM_ANALOG_CHANNELS; i++){
    analogChannels[i].txValue = analogChannels[i].rawValue;
  }
}

void readDigitalValues(){
  for (int i = 0; i < RADIO_NUM_DIGITAL_CHANNELS; i++){
    digitalChannels[i].rawValue <<= 1;
    digitalChannels[i].rawValue |= digitalRead(digitalChannels[i].pin);
  }
}

void processDigitalValues(){
  for (int i = 0; i < RADIO_NUM_DIGITAL_CHANNELS; i++){
    if (digitalChannels[i].rawValue >= 0xF ){
      digitalChannels[i].txDebouncedValue = 1;
    }
    else{
      digitalChannels[i].txDebouncedValue = 0;
    }
  }
}

void printRawInputValues(){
  Serial.println();
  for (int i = 0; i < RADIO_NUM_ANALOG_CHANNELS; i++){
    Serial.printf("RA%d: %d\n", i, analogRead(analogChannels[i].pin));
  }
  for (int i = 0; i < RADIO_NUM_DIGITAL_CHANNELS; i++){
    Serial.printf("RD%d: %d\n", i, digitalRead(digitalChannels[i].pin));
  }
}
void printTxValues(){
  Serial.println();
  for (int i = 0; i < RADIO_NUM_ANALOG_CHANNELS; i++){
    Serial.printf("A%d: %d\n", i, analogChannels[i].txValue);
  }
  for (int i = 0; i < RADIO_NUM_DIGITAL_CHANNELS; i++){
    Serial.printf("D%d: %d\n", i, digitalChannels[i].txDebouncedValue);
  }
}

void configureRadio(){
  // Radio stuff -----------------------
  SPI.setSCK(SCK);
  SPI.setCS(RADIO_CS_PIN);
  SPI.setTX(MOSI);
  SPI.setRX(MISO);
  SPI.begin(true);
  if(radio.begin(&SPI)){
    Serial.println("Communicating with with radio");
  }
  else{
    Serial.println("Failed to communicate with radio!");
    while(1){
      Serial.println("Failed to communicate with radio!");
      delay(1000);
    };
  }

}
void finishRadioInit(){
  doCommonRadioConfig(radio);

  //Set module as transmitter
  radio.stopListening();
  radio.openWritingPipe(RadioCommAddress);

  // End radio stuff -------------------
}
void doChannelScan(){
    uint8_t carrier_test[128] = {1};
    uint8_t request_channel_inter = 255;
    radio.setAutoAck(false);
    radio.startListening();
    delayMicroseconds(100);
    radio.stopListening();
    delayMicroseconds(100);
    bool test_value = false;
    memset(carrier_test,0,sizeof(carrier_test));
    for (int j=0; j < 70; j++){
        for (int i=0; i < 128; i++){
            radio.setChannel(i);
            radio.startListening();
            delayMicroseconds(128);
            radio.stopListening();
            if(radio.isPVariant()){
                test_value = radio.testRPD();
            }
            else{
                test_value = radio.testCarrier();
            }
            if (test_value){
                ++carrier_test[i];
            }
        }
    }
    for (int i=0; i < 128; i++){
        if(carrier_test[i] < request_channel_inter){
            request_channel_inter = carrier_test[i];
        }
        Serial.printf("Channel %03d: %03d - ", i, carrier_test[i]);
        for (int j=0; j < carrier_test[i]; j++){
            Serial.print("*");
        }
        Serial.println("");
    }
    Serial.printf("Best channel: %d, PVarient?: %d\n", request_channel, radio.isPVariant());
}
void resetRadioData(unsigned int val){
  for(int i = 0; i < RADIO_PAYLOAD_SIZE; i++){
    radio_data[i] = val;
  }
}
void resetResponseData(){
    for (int i = 0; i < RADIO_RESPONSE_PAYLOAD_SIZE; i++)
    {
        radio_response_data[i] = 0;
    }
}
void updateDebugDisplay(){
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.printf(LINE1_STATUS,
        (int)analogChannels[0].txValue * (analogChannels[0].txSignValueIsNegative ? -1 : 1),
        (int)analogChannels[2].txValue * (analogChannels[2].txSignValueIsNegative ? -1 : 1)
    );
    display.printf(LINE2_STATUS,
        (int)analogChannels[1].txValue * (analogChannels[1].txSignValueIsNegative ? -1 : 1),
        (int)analogChannels[3].txValue * (analogChannels[3].txSignValueIsNegative ? -1 : 1)
    );
    display.print("D:");
    for(int i=0; i<RADIO_NUM_DIGITAL_CHANNELS; i++){
      display.printf("%01d", digitalChannels[i].txDebouncedValue);
    }
    display.printf(" CH:%03d D:%03d\n", RADIO_DEFAULT_CHANNEL, request_channel);
    display.printf("Tx Loop Time: %05d\n", (loop_start_time - last_loop_start_time));
    display.printf(TX_STAT, tx_percent_fail_disp);
    long dist = radio_response_data[RESPONSE_PING0_BYTE] + (radio_response_data[RESPONSE_PING1_BYTE]<<8);
    display.printf("Dist:% 5d\n", dist);
    display.setCursor(0, (display.height()-10));
    display.printf("Batt: %.02f (%d)\n", getVoltsFromRawBatVal(radio_response_data[RESPONSE_BATTERY_BYTE]), rx_payload_counter);
    display.display();
}
void updateRunningDisplay(){
    float batt = getVoltsFromRawBatVal(radio_response_data[RESPONSE_BATTERY_BYTE]);
    uint8_t mode = radio_response_data[RESPONSE_MODE_BYTE];
    uint8_t firing_mode = radio_response_data[RESPONSE_FMODE_BYTE];
    uint8_t firing_velocity = radio_response_data[RESPONSE_FVEL_BYTE];
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Mode: ");
    if(mode == MODE_DRIVING){
      display.print("Driving");
    }
    else if (mode == MODE_BATTLE){
      display.print("Battle");
    }
    display.println("");
    
    display.print("Firing Mode: ");
    if(firing_mode == FMODE_SINGLE){
      display.print("Single");
    }
    else if (firing_mode == FMODE_3BURST){
      display.print("Burst");
    }
    display.println("");
        display.print("Firing Velocity: ");
    if(firing_velocity == FVEL_LOW){
      display.print("Low");
    }
    else if (firing_velocity == FVEL_HIGH){
      display.print("High");
    }
    display.println("");

    // display.printf("Weapon: %s\n", (true ? "Armed!":"Offline"));
    display.setCursor(0, (display.height()-10));
    display.printf("Bat: %s (%.02fV)\n", ((batt < 0.1) ? "BAD" : ((batt > 7.0f) ? "Good":"LOW")), batt);
    display.display();
}
void processFailCounter(){
  tx_percent_fail_disp = 100 * (float)(tx_fail_counter)/(float)tx_attempt_counter;
  if(tx_attempt_counter > 1000){
    tx_attempt_counter = 1; // cheat to avoid dealing with div by 0
    tx_fail_counter = 0;
  }
}
void handleRadio(){
  PackRadioData();
  if(!radio.write(radio_data, (RADIO_PAYLOAD_SIZE * sizeof(uint8_t)))){
    tx_fail_counter += 1;
  }
  tx_attempt_counter += 1;
  if(radio.available()){
    if(rx_payload_counter > 999) rx_payload_counter = 1;
    else rx_payload_counter += 1;
    resetResponseData();
    radio.read(&radio_response_data, (RADIO_RESPONSE_PAYLOAD_SIZE * sizeof(uint8_t)));
  }
}
void updateDisplay(){
  if(digitalChannels[5].txDebouncedValue == 0){
    updateRunningDisplay();
  }
  else{
    updateDebugDisplay();
  }
}
void initScheduler(){
  scheduler.addEvent(DISPLAY_UPDATE_INTERVAL, true, updateDisplay);
  scheduler.addEvent(RADIO_TX_INTERVAL, true, handleRadio);
  scheduler.addEvent(1000, true, processFailCounter);
}
void setup() {
  Serial.begin(9600);
  led.begin();
  led.setPixelColor(0, led.Color(255,0,0));
  led.setBrightness(1);
  led.show();
  configureDisplay();

  // Global var init
  // 8 bits of resolution should be fine
  // (RP2040 Analog has issues with higher precision anyway)
  analogReadResolution(ANALOG_READ_RESOLUTION_BITS);
  initAnalogChannelStructs();
  initDigitalChannelStructs();
  initChannelPins();
  resetResponseData();
  delay(10);
  initDeadZone();
  loop_start_time = millis();
  last_print_time = loop_start_time;
  last_loop_start_time = loop_start_time;
  last_display_update_time = loop_start_time;
  debug_print_this_loop = false;

  tx_fail_counter = 0;
  tx_attempt_counter = 0;
  rx_payload_counter = 0;
  // // end global init
  configureRadio();
  doChannelScan();
  finishRadioInit(); 
  initScheduler();

}

void loop() {
  readAnalogValues();
  readDigitalValues();
  processAnalogValues();
  processDigitalValues();
  doInputConditioning();
  scheduler.run();
}

