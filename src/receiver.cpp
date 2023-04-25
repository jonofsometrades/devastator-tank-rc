#include "Local_Common.h"
#include "Events.h"
#include <Arduino.h>
#include <RP2040.h>
#include <cstdio>
#include <string>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

#define DEBUG_ENABLED
#define LOOP_DELAY 10
#define NUM_SERVOS 4
#define SERVO_LA 0
#define SERVO_DF 1
#define SERVO_ESC_L 2
#define SERVO_ESC_R 3
#define SERVO_LA_PIN 4u
#define SERVO_DF_PIN 5u
#define SERVO_ESC_LW_LEFT_PIN 6u
#define SERVO_ESC_LW_RIGHT_PIN 7u
#define SERVO_DF_SENS_PIN 10u
#define DRV_8833_U1_IN1_PIN 3
#define DRV_8833_U1_IN2_PIN 2
#define DRV_8833_U2_IN1_PIN 0
#define DRV_8833_U2_IN2_PIN 1
#define MOTOR_MIN_PWM 120
#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2
#define MOTOR_BOTH 3
#define PWM_FREQ_HZ 100000
#define PWM_RANGE 255
#define MAX_VELOCITY_MAG 127
#define MAX_ANALOG_MAG 127
#define MOTOR_LEFT_RADIO_CHANNEL 3
#define MOTOR_RIGHT_RADIO_CHANNEL 0
#define MOTOR_STEER_LR_RADIO_CHANNEL 2
#define LAUNCH_ANGLE_RADIO_CHANNEL 0
#define FIRE_RADIO_DCHANNEL 1
#define MODE_RADIO_DCHANNEL 2
#define FMODE_RADIO_DCHANNEL 3
#define FVAL_RADIO_DCHANNEL 4
#define ESC_FIRING_SPEED_HIGH 100
#define ESC_FIRING_SPEED_LOW 50
// Radio related defines
#define RADIO_CS_PIN PIN_SPI0_SS
#define RADIO_CE_PIN 22u
#define BATTERY_VOLTAGE_PIN A0
#define PING_TRIG_PIN 12u
#define PING_RESP_PIN 11u

EventScheduler scheduler;
struct StateMach
{
    enum State
    {
        RESTART='R',
        WAITING='W',
        TRIGGERED='T',
        START_ESCS='E',
        START_DF='S',
        WAITING_FOR_DF_SENS_ON='O',
        WAITING_FOR_DF_SENS_OFF='F',
        FIRE_STOPPING='D'
    };
    volatile State currentState;
    unsigned long nextStateStartTimeDelay;
    unsigned long timeStart;
};
struct ServoData
{
    uint8_t servoPin;
    float remoteCommandValue;
    float scaledRemoteValue;
    float currentValue;
    float lastValue;
    uint8_t initPosition;
    uint8_t minPosition;
    uint8_t maxPosition;
    float updateNewValFactor;
    bool useMotionSmoothing;
    bool usePositionAdjust;
    Servo servo;
    uint8_t attachSuccess = 0;
    bool isEsc = false;
};

StateMach firingSM = {};
ServoData servosData[NUM_SERVOS] = {};
bool motorsEnabled = false;
uint8_t rawBatteryVolts = 0;
unsigned long loop_start_time;
unsigned long rx_msg_counter = 0;
unsigned long rx_unpack_counter = 0;
unsigned long last_print_time;
unsigned long ping_duration = 0;
uint8_t current_mode = MODE_DRIVING;
uint8_t firing_velocity = FVEL_LOW;
uint8_t firing_mode = FMODE_SINGLE;
uint8_t firing_counter = 0;
bool unpackSuccess = false;
unsigned long radio_last_received;
bool debug_print_this_loop;
unsigned long tx_fail_counter = 0;

// // https://lastminuteengineers.com/nrf24l01-arduino-wireless-communication/
// //create an RF24 object
RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);

void initPingPins(){
    pinMode(PING_RESP_PIN, INPUT);
    pinMode(PING_TRIG_PIN, OUTPUT);
    digitalWrite(PING_TRIG_PIN, 0);
}
void doPing(){
     // Clears the trigPin
  digitalWrite(PING_TRIG_PIN, LOW);
  delayMicroseconds(10);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(PING_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(PING_TRIG_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  ping_duration = pulseIn(PING_RESP_PIN, HIGH, 10000);
}
void initBatteryVoltPin()
{
    analogReadResolution(ANALOG_READ_RESOLUTION_BITS);
    pinMode(BATTERY_VOLTAGE_PIN, INPUT);
}
void updateBatteryRawVolts()
{
    rawBatteryVolts = analogRead(BATTERY_VOLTAGE_PIN);
}
void packAckPayload()
{
    radio_response_data[RESPONSE_BATTERY_BYTE] = rawBatteryVolts + 1;
    radio_response_data[RESPONSE_PING0_BYTE] = (ping_duration & 0xff);
    radio_response_data[RESPONSE_PING1_BYTE] = ((ping_duration>>8) & 0xff);
    radio_response_data[RESPONSE_MODE_BYTE] = current_mode;
    radio_response_data[RESPONSE_FMODE_BYTE] = firing_mode;
    radio_response_data[RESPONSE_FVEL_BYTE] = firing_velocity;
}
void setAckPayload()
{
    radio.writeAckPayload(RADIO_PIPE, radio_response_data, (RADIO_RESPONSE_PAYLOAD_SIZE * sizeof(uint8_t)));
}
void resetResponseData()
{
    for (int i = 0; i < RADIO_RESPONSE_PAYLOAD_SIZE; i++)
    {
        radio_response_data[i] = 0;
    }
}

void updateModes(){
    if(digitalChannels[MODE_RADIO_DCHANNEL].txDebouncedValue){
        current_mode = MODE_BATTLE;
    }
    else{
        current_mode = MODE_DRIVING;
    }
    if(digitalChannels[FMODE_RADIO_DCHANNEL].txDebouncedValue){
        firing_mode = FMODE_SINGLE;
    }
    else{
        firing_mode = FMODE_3BURST;
    }
    if(digitalChannels[FVAL_RADIO_DCHANNEL].txDebouncedValue){
        firing_velocity = FVEL_HIGH;
    }
    else{
        firing_velocity = FVEL_LOW;
    }
}

bool unPackRadioData()
{
    if (((radio_data[RADIO_PROT_MAGIC_BYTE] >> RADIO_MAGIC_BITS) & RADIO_MAGIC_BITS_MASK) == RADIO_MAGIC_VALUE &&
        ((radio_data[RADIO_PROT_MAGIC_BYTE] >> RADIO_PROT_BITS) & RADIO_PROT_BITS_MASK) == RADIO_PROT_VALUE)
    {
        for (int i = 0; i < RADIO_NUM_ANALOG_CHANNELS; i++)
        {
            analogChannels[i].txValue = radio_data[analogChannels[i].txValueBytePosition];
            analogChannels[i].txSignValueIsNegative = 0x1 & (radio_data[analogChannels[i].txSignBytePosition] >> analogChannels[i].txSignBitPosition);
        }
        for (int i = 0; i < RADIO_NUM_DIGITAL_CHANNELS; i++)
        {
            digitalChannels[i].txDebouncedValue = 0x1 & (radio_data[digitalChannels[i].txValueBytePosition] >> digitalChannels[i].txValueBitPosition);
        }
        return true;
    }
    return false;
}
void motorBrake(uint8_t motorNum)
{
    if (motorNum == MOTOR_LEFT || motorNum == MOTOR_BOTH)
    {
        analogWrite(DRV_8833_U1_IN1_PIN, HIGH);
        analogWrite(DRV_8833_U1_IN2_PIN, HIGH);
    }
    if (motorNum == MOTOR_RIGHT || motorNum == MOTOR_BOTH)
    {
        analogWrite(DRV_8833_U2_IN1_PIN, HIGH);
        analogWrite(DRV_8833_U2_IN2_PIN, HIGH);
    }
}
void motorCoast(uint8_t motorNum)
{
    if (motorNum == MOTOR_LEFT || motorNum == MOTOR_BOTH)
    {
        analogWrite(DRV_8833_U1_IN1_PIN, LOW);
        analogWrite(DRV_8833_U1_IN2_PIN, LOW);
    }
    if (motorNum == MOTOR_RIGHT || motorNum == MOTOR_BOTH)
    {
        analogWrite(DRV_8833_U2_IN1_PIN, LOW);
        analogWrite(DRV_8833_U2_IN2_PIN, LOW);
    }
}
void initMotorPins()
{
    pinMode(DRV_8833_U1_IN1_PIN, OUTPUT);
    pinMode(DRV_8833_U1_IN2_PIN, OUTPUT);
    pinMode(DRV_8833_U2_IN1_PIN, OUTPUT);
    pinMode(DRV_8833_U2_IN2_PIN, OUTPUT);
    analogWriteFreq(PWM_FREQ_HZ);
    analogWriteRange(PWM_RANGE);
    analogWrite(DRV_8833_U1_IN1_PIN, 0);
    analogWrite(DRV_8833_U1_IN2_PIN, 0);
    analogWrite(DRV_8833_U2_IN1_PIN, 0);
    analogWrite(DRV_8833_U2_IN2_PIN, 0);
}
void doSteering(int8_t x, int8_t y, int8_t &motorLeft, int8_t &motorRight) {
  // Calculate the raw motor speeds
  int16_t v = (int16_t)y;
  int16_t w = (int16_t)x;

  // Calculate the motor commands
  int16_t motorLeftRaw = v + w;
  int16_t motorRightRaw = v - w;

  // Constrain the raw commands to the motor command range
  motorLeft = constrain(motorLeftRaw, -127, 127);
  motorRight = constrain(motorRightRaw, -127, 127);
}

void motorGo(uint8_t motorNum, int velocity)
{
    uint16_t pwmVal;
    if (velocity == 0)
    {
        pwmVal = 0;
    }
    else
    {
        pwmVal = (uint16_t)map(abs(velocity), 0, MAX_VELOCITY_MAG, MOTOR_MIN_PWM, PWM_RANGE);
    }
    if (pwmVal > PWM_RANGE)
    {
        pwmVal = PWM_RANGE;
    }
    if (motorNum == MOTOR_LEFT || motorNum == MOTOR_BOTH)
    {
        if (velocity > 0)
        {
            analogWrite(DRV_8833_U1_IN1_PIN, 0);
            analogWrite(DRV_8833_U1_IN2_PIN, pwmVal);
        }
        else
        {
            analogWrite(DRV_8833_U1_IN2_PIN, 0);
            analogWrite(DRV_8833_U1_IN1_PIN, pwmVal);
        }
    }
    if (motorNum == MOTOR_RIGHT || motorNum == MOTOR_BOTH)
    {
        if (velocity > 0)
        {
            analogWrite(DRV_8833_U2_IN1_PIN, 0);
            analogWrite(DRV_8833_U2_IN2_PIN, pwmVal);
        }
        else
        {
            analogWrite(DRV_8833_U2_IN2_PIN, 0);
            analogWrite(DRV_8833_U2_IN1_PIN, pwmVal);
        }
    }
}

void updateMotorCommand()
{
    if (motorsEnabled)
    {
        int16_t motorLeftVelocity = 0;
        int16_t motorRightVelocity = 0;
        if(current_mode == MODE_DRIVING){
            motorLeftVelocity = (analogChannels[MOTOR_LEFT_RADIO_CHANNEL].txValue * (analogChannels[MOTOR_LEFT_RADIO_CHANNEL].txSignValueIsNegative ? -1 : 1));
            motorRightVelocity = (analogChannels[MOTOR_RIGHT_RADIO_CHANNEL].txValue * (analogChannels[MOTOR_RIGHT_RADIO_CHANNEL].txSignValueIsNegative ? -1 : 1));
        }
        else if(current_mode == MODE_BATTLE){
            int16_t v = (analogChannels[MOTOR_LEFT_RADIO_CHANNEL].txValue * (analogChannels[MOTOR_LEFT_RADIO_CHANNEL].txSignValueIsNegative ? -1 : 1));
            int16_t w = (analogChannels[MOTOR_STEER_LR_RADIO_CHANNEL].txValue * (analogChannels[MOTOR_STEER_LR_RADIO_CHANNEL].txSignValueIsNegative ? -1 : 1));

            motorLeftVelocity = v - w;
            motorRightVelocity = v + w;
        }
        motorGo(MOTOR_LEFT, constrain(motorLeftVelocity, -127, 127));
        motorGo(MOTOR_RIGHT, constrain(motorRightVelocity, -127, 127));
    }
    else
    {
        motorCoast(MOTOR_BOTH);
    }
}

void blinkLED()
{
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void printRawInputValues()
{
    Serial.println();
    for (int i = 0; i < RADIO_NUM_ANALOG_CHANNELS; i++)
    {
        Serial.printf("A%d: %d\n", i, (analogChannels[i].txValue * (analogChannels[i].txSignValueIsNegative ? -1 : 1)));
    }
    for (int i = 0; i < RADIO_NUM_DIGITAL_CHANNELS; i++)
    {
        Serial.printf("D%d: %d\n", i, digitalChannels[i].txDebouncedValue);
    }
}

void configureRadio()
{
    // Radio stuff -----------------------
    SPI.setSCK(PIN_SPI0_SCK);
    SPI.setCS(RADIO_CS_PIN);
    SPI.setTX(PIN_SPI0_MOSI);
    SPI.setRX(PIN_SPI0_MISO);
    SPI.begin(true);
    if (radio.begin(&SPI))
    {
        Serial.println("Communicating with with radio");
    }
    else
    {
        Serial.println("Failed to communicate with radio!");
        while (1)
        {
            Serial.println("Failed to communicate with radio!");
            delay(1000);
        };
    }

    doCommonRadioConfig(radio);
    // set the RadioCommAddress
    radio.openReadingPipe(RADIO_PIPE, RadioCommAddress);
    // Set module as receiver
    radio.startListening();
    // End radio stuff -------------------
}
void resetRadioData(unsigned int val)
{
    for (int i = 0; i < RADIO_PAYLOAD_SIZE; i++)
    {
        radio_data[i] = val;
    }
}
void toggleLed()
{
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void initServos()
{
    servosData[SERVO_LA].servoPin = SERVO_LA_PIN;
    servosData[SERVO_LA].initPosition = 100;
    servosData[SERVO_LA].currentValue = servosData[SERVO_LA].initPosition;
    servosData[SERVO_LA].lastValue = servosData[SERVO_LA].initPosition;
    servosData[SERVO_LA].remoteCommandValue = servosData[SERVO_LA].initPosition;
    servosData[SERVO_LA].minPosition = 60;
    servosData[SERVO_LA].maxPosition = 180;
    servosData[SERVO_LA].updateNewValFactor = 0.01;
    servosData[SERVO_LA].useMotionSmoothing = false;
    servosData[SERVO_LA].usePositionAdjust = true;

    servosData[SERVO_DF].servoPin = SERVO_DF_PIN;
    servosData[SERVO_DF].initPosition = 90;
    servosData[SERVO_DF].currentValue = servosData[SERVO_DF].initPosition;
    servosData[SERVO_DF].lastValue = servosData[SERVO_DF].initPosition;
    servosData[SERVO_DF].remoteCommandValue = servosData[SERVO_DF].initPosition;
    servosData[SERVO_DF].minPosition = 90;
    servosData[SERVO_DF].maxPosition = 180;//TODO: set to correct speed
    servosData[SERVO_DF].updateNewValFactor = 1.0;
    servosData[SERVO_DF].useMotionSmoothing = false;
    servosData[SERVO_LA].usePositionAdjust = false;

    servosData[SERVO_ESC_L].servoPin = SERVO_ESC_LW_LEFT_PIN;
    servosData[SERVO_ESC_L].initPosition = 180;
    servosData[SERVO_ESC_L].currentValue = servosData[SERVO_ESC_L].initPosition;
    servosData[SERVO_ESC_L].lastValue = servosData[SERVO_ESC_L].initPosition;
    servosData[SERVO_ESC_L].remoteCommandValue = servosData[SERVO_ESC_L].initPosition;
    servosData[SERVO_ESC_L].minPosition = 0;
    servosData[SERVO_ESC_L].maxPosition = 180;
    servosData[SERVO_ESC_L].updateNewValFactor = 1.0;
    servosData[SERVO_ESC_L].useMotionSmoothing = false;
    servosData[SERVO_LA].usePositionAdjust = false;
    servosData[SERVO_ESC_L].isEsc = true;

    servosData[SERVO_ESC_R].servoPin = SERVO_ESC_LW_RIGHT_PIN;
    servosData[SERVO_ESC_R].initPosition = 180;
    servosData[SERVO_ESC_R].currentValue = servosData[SERVO_ESC_R].initPosition;
    servosData[SERVO_ESC_R].lastValue = servosData[SERVO_ESC_R].initPosition;
    servosData[SERVO_ESC_R].remoteCommandValue = servosData[SERVO_ESC_R].initPosition;
    servosData[SERVO_ESC_R].minPosition = 0;
    servosData[SERVO_ESC_R].maxPosition = 180;
    servosData[SERVO_ESC_R].updateNewValFactor = 1.0;
    servosData[SERVO_ESC_R].useMotionSmoothing = false;
    servosData[SERVO_LA].usePositionAdjust = false;
    servosData[SERVO_ESC_R].isEsc = true;
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        servosData[i].servo.write(servosData[i].initPosition);
        servosData[i].attachSuccess = servosData[i].servo.attach(servosData[i].servoPin);
        servosData[i].servo.write(servosData[i].initPosition);
    }
}
void initFiringSM()
{
    firingSM.currentState = firingSM.WAITING;
    pinMode(SERVO_DF_SENS_PIN, INPUT_PULLUP);
    firingSM.nextStateStartTimeDelay = 0;
    firingSM.timeStart = 0;
}
void processFiringState()
{
    if((!motorsEnabled || current_mode == MODE_DRIVING) && firingSM.currentState != firingSM.WAITING){
        firingSM.currentState = firingSM.RESTART;
    }
    else if(millis() < (firingSM.timeStart + firingSM.nextStateStartTimeDelay)){
        // bail as we haven't waited long enough
        return;
    }
    
    // Serial.println((char)firingSM.currentState);
    switch (firingSM.currentState)
    {
    case firingSM.RESTART:
        servosData[SERVO_ESC_L].servo.write(servosData[SERVO_ESC_L].minPosition);
        servosData[SERVO_ESC_R].servo.write(servosData[SERVO_ESC_R].minPosition);
        servosData[SERVO_DF].servo.write(servosData[SERVO_DF].minPosition);
        firing_counter = 0;
        firingSM.currentState = firingSM.WAITING;
    // intentional fallthrough
    case firingSM.WAITING:
        if(!digitalChannels[FIRE_RADIO_DCHANNEL].txDebouncedValue){
            firingSM.currentState = firingSM.TRIGGERED;
        }
        break;
    case firingSM.TRIGGERED:
        firingSM.currentState = firingSM.START_ESCS;
    // intentional fallthrough
    case firingSM.START_ESCS:
        servosData[SERVO_ESC_L].servo.write(min(servosData[SERVO_ESC_L].maxPosition, ((firing_velocity == FVEL_HIGH) ? ESC_FIRING_SPEED_HIGH:ESC_FIRING_SPEED_LOW)));
        servosData[SERVO_ESC_R].servo.write(min(servosData[SERVO_ESC_R].maxPosition, ((firing_velocity == FVEL_HIGH) ? ESC_FIRING_SPEED_HIGH:ESC_FIRING_SPEED_LOW)));
        firingSM.currentState = firingSM.START_DF;
        firingSM.nextStateStartTimeDelay = 10;
        firingSM.timeStart = millis();
        break;
    case firingSM.START_DF:
        firing_counter++;
        servosData[SERVO_DF].servo.write(servosData[SERVO_DF].maxPosition);
        firingSM.currentState = firingSM.WAITING_FOR_DF_SENS_OFF;
        firingSM.nextStateStartTimeDelay = 0;
        firingSM.timeStart = millis();
        break;
    case firingSM.WAITING_FOR_DF_SENS_OFF:
        if(!digitalRead(SERVO_DF_SENS_PIN)){
            firingSM.currentState = firingSM.WAITING_FOR_DF_SENS_ON;
            firingSM.nextStateStartTimeDelay = 0;
            firingSM.timeStart = millis();
        }
        break;
    case firingSM.WAITING_FOR_DF_SENS_ON:
        if(digitalRead(SERVO_DF_SENS_PIN)){
            firingSM.currentState = firingSM.FIRE_STOPPING;
            firingSM.nextStateStartTimeDelay = 0;
            firingSM.timeStart = millis();
        }
        break;
    case firingSM.FIRE_STOPPING:
        servosData[SERVO_DF].servo.write(servosData[SERVO_DF].minPosition);
        firingSM.nextStateStartTimeDelay = 500;
        firingSM.timeStart = millis();
        if(firing_mode == FMODE_SINGLE || (firing_mode == FMODE_3BURST && firing_counter >= 3)){
            firingSM.currentState = firingSM.RESTART;
        }
        else{
            firingSM.currentState = firingSM.START_DF;
        }
        break;
    
    default:
        break;
    }
}
void debugPrint()
{
    // Serial.printf("%d msg.  ", rx_msg_counter);
    // Serial.print("RP: ");
    // for (int i = 0; i < RADIO_PAYLOAD_SIZE; i++)
    // {
    //     Serial.print(radio_data[i], 16);
    //     Serial.print(" ");
    // }
    // Serial.println("");
    Serial.printf("RC: %0.3f, CV: %0.3f\n", servosData[SERVO_LA].remoteCommandValue, servosData[SERVO_LA].currentValue);
    // Serial.printf("Mode: %d, FMode: %d, FVel: %d\n", current_mode, firing_mode, firing_velocity);
    // Serial.printf("LA CV: %0.2f, RC: %0.2f, ScaledRemote: %0.2f, state: %c, dur: %d, dist: %d\n", servosData[SERVO_LA].currentValue, servosData[SERVO_LA].remoteCommandValue, servosData[SERVO_LA].scaledRemoteValue, firingSM.currentState, ping_duration, (ping_duration>>5));
}
void commandLaServo()
{
    if(current_mode == MODE_BATTLE){
        servosData[SERVO_LA].remoteCommandValue = (analogChannels[LAUNCH_ANGLE_RADIO_CHANNEL].txValue * (analogChannels[LAUNCH_ANGLE_RADIO_CHANNEL].txSignValueIsNegative ? 1 : -1));
        if (abs(servosData[SERVO_LA].remoteCommandValue) > 2){
            servosData[SERVO_LA].currentValue = ((servosData[SERVO_LA].remoteCommandValue * servosData[SERVO_LA].updateNewValFactor) + servosData[SERVO_LA].lastValue );
            servosData[SERVO_LA].currentValue = constrain(servosData[SERVO_LA].currentValue, servosData[SERVO_LA].minPosition, servosData[SERVO_LA].maxPosition );
            servosData[SERVO_LA].servo.write(servosData[SERVO_LA].currentValue);
            servosData[SERVO_LA].lastValue = servosData[SERVO_LA].currentValue;
        }
        
    }
}

void initEscs(){
    delay(1000);
    for (int i = 0; i < NUM_SERVOS; i++){
        if (servosData[i].isEsc){
            servosData[i].servo.write(servosData[i].minPosition);
        }
    }
}
void setSafeValues(){
#pragma message("Add Safe Values")
}
void calcMotorEnableTimeout(){
    motorsEnabled = ((loop_start_time - radio_last_received) < RADIO_MAX_TIME_SINCE_LAST_RECEPTION);
}
void initEvents(){
    scheduler.addEvent(5, true, commandLaServo);
    scheduler.addEvent(10, true, updateMotorCommand);
    scheduler.addEvent(1000, true, toggleLed);
    scheduler.addEvent(5, true, processFiringState);
    scheduler.addEvent(1000, true, calcMotorEnableTimeout);
    scheduler.addEvent(1000, true, updateBatteryRawVolts);
    scheduler.addEvent(500, true, doPing);
    scheduler.addEvent(1000, true, debugPrint);
    // scheduler.addEvent(1000, true, printRawInputValues);
}
void setup()
{
    initBatteryVoltPin();
    initFiringSM();
    initServos();
    // while(getVoltsFromRawBatVal(rawBatteryVolts) < 4){
    //     updateBatteryRawVolts();
    // }
    initEscs();
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    initAnalogChannelStructs();
    initDigitalChannelStructs();
    initMotorPins();
    initPingPins();
    
    Serial.begin(9600);
    loop_start_time = millis();
    radio_last_received = loop_start_time - (RADIO_MAX_TIME_SINCE_LAST_RECEPTION*2);

    rx_msg_counter = 0;
    // // end global init
    resetRadioData(0);
    configureRadio();
    motorsEnabled = false;
    Serial.setTimeout(10000);
    digitalWrite(LED_BUILTIN, LOW);
    initEvents();
}
void loop()
{
    loop_start_time = millis();
    while (radio.available())
    {
        radio_last_received = loop_start_time;
        resetRadioData(0);
        radio.read(&radio_data, (RADIO_PAYLOAD_SIZE * sizeof(uint8_t)));
        unpackSuccess = unPackRadioData();
        updateModes();
        // ACK Payload will be sent after the NEXT reception.
        // Setting here to prevent constantly jamming values in ack buffer (max 3)
        resetResponseData();
        packAckPayload();
        setAckPayload();
        if(rx_msg_counter > 1000) rx_msg_counter = 0;
        rx_msg_counter++;
    }
    
    // Do events
    scheduler.run();
}
