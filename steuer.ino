/*Autor: Yannick Wunderle
   Datum: 14.03.2021
   Funktion: Steuergerät DEMM mit 3kW 120kV Brushless


   RX Gelb
   TX Grün
   5V Braun
   SDA Schwarz
   SCL Weiß

   VESC Library Timeout von 100 auf 5 und wieder zurück auf 100.
   Wire.h Library Timeout von 50 auf 5ms
*/
#include <Wire.h>
#include "vesc_uart/VescUart.h"
#include "soc/i2c_struct.h"

//PINS
#define FIELDCOIL_PIN 32
#define BEEPER_PIN 23
#define RELAIS_PIN 22
#define BRAKE_PIN_L 19
#define BRAKE_PIN_R 18
#define THROTTLE_PIN 33
#define SDA_PIN 21
#define SCL_PIN 5

//VESC Motor and Battery
#define BATTERY_CELLS 15
#define LOW_THROTTLE_VALUE 660
#define HIGH_THROTTLE_VALUE 4060
#define MAX_CURRENT 120
#define BRAKE_CURRENT_WEAK 30
#define BRAKE_CURRENT_MEDIUM 40
#define BRAKE_CURRENT_HARD 100

#define CYCLE_TIME 30
#define FAULT_TIME 1000

//Gearing
#define Z1 26
#define Z2 171
#define MagnetPairs 5
#define TacPerErev 6 //constant of the vesc, three states going through two magnets (one pole pair)

//Makes ESP go silent if there is no communication instead of beeping forever and shutting everything off. Useful for testing with vesc tool.
#define DEBUG (1)

#define I2C_ADDRESS 8

const float WHEEL_CIR = 1.66; //m
const float LOW_VOLTAGE = 3.5;
const float PRE_CHARGE_DROPOFF = 2; // when charging the capacitors through the resistor the voltage should at least reach the minimum voltage minus this value
unsigned long previousMillisFault = 0;
unsigned long previousMillisPrint = 0;
unsigned long previousMillisCycle = 0;

const long erotPerKm = MagnetPairs*1000L*Z2/Z1/WHEEL_CIR;


// setting PWM properties

float distance = 0;

VescUart UART;

void storeValues();

void setup() {
    pinMode(FIELDCOIL_PIN, OUTPUT);
    pinMode(BEEPER_PIN, OUTPUT);
    pinMode(RELAIS_PIN, OUTPUT);

    pinMode(THROTTLE_PIN, INPUT);
    pinMode(BRAKE_PIN_L, INPUT);
    pinMode(BRAKE_PIN_R, INPUT);

    digitalWrite(FIELDCOIL_PIN, LOW);
    digitalWrite(BEEPER_PIN, LOW);
    digitalWrite(RELAIS_PIN, HIGH);

    Serial2.begin(115200);
    Serial.begin(115200);


    while (!Serial2) ; //VESC

    UART.setSerialPort(&Serial2);
    digitalWrite(BEEPER_PIN, HIGH);
    delay(3);
    digitalWrite(BEEPER_PIN, LOW);

    Serial.println("waiting for vesc comm");
    while (!UART.getVescValues()) {
        delay(100);
    }

    Serial.println("waiting for pre charge voltage");
    while (UART.data.inpVoltage < BATTERY_CELLS * LOW_VOLTAGE - PRE_CHARGE_DROPOFF) {
        UART.getVescValues();
        delay(100);
    }
    Serial.println("turning on");

    digitalWrite(RELAIS_PIN, HIGH);

    digitalWrite(BEEPER_PIN, HIGH);
    delay(6);
    digitalWrite(BEEPER_PIN, LOW);

    Wire.onRequest(storeValues);
    Serial.println(Wire.begin(I2C_ADDRESS, SDA_PIN, SCL_PIN, 400000L));
}

void storeValues(){
    const int baseInputVoltage = 2;
    const int baseMotorTemp = -20;
    const int AhFactor = 10;
    const int voltFactor = 100;
    const int kmhFactor = 3;
    const int kmFactor = 100;

    const int msgLen = 9;

    uint16_t km = constrain((long)UART.data.tachometer/TacPerErev*100/erotPerKm, 0, 0xFFFF);
    uint16_t kmabs = constrain((long)UART.data.tachometer/TacPerErev/erotPerKm, 0, 0xFFFF);

    static uint8_t tt = 0;
    tt++;

    uint8_t msg[msgLen+1] = {
            constrain(abs(UART.data.avgInputCurrent), 0, 255),
            tt,
            km >> 8,
            constrain((UART.data.ampHours*AhFactor), 0, 255),
            kmabs,
            kmabs >> 8,
            constrain((UART.data.inpVoltage/BATTERY_CELLS-baseInputVoltage)*voltFactor, 0, 255),
            constrain(UART.data.rpm*3*60/erotPerKm, 0, 255),
            constrain(UART.data.tempMotor-baseMotorTemp, 0, 255),
            0 //csum
    };

    for(int i = 0; i < msgLen; i++){
        msg[msgLen] += msg[i];
    }

    static int busBusyCycles = 0;
    if(I2C0.status_reg.scl_main_state_last == 4){
        Serial.println(4);
        Serial.println(busBusyCycles);
        Serial.println(I2C0.status_reg.bus_busy);
        Serial.println(Wire.slaveWrite(msg, sizeof(msg)));
        if(busBusyCycles > 10){
            I2C0.command.opcode = 3;
            I2C0.command.opcode = 4;
//      Wire.end();
//      Wire.begin(I2C_ADDRESS, SDA_PIN, SCL_PIN, 400000L);
        }
        busBusyCycles++;

    }
    else{
        busBusyCycles = 0;}
    if(!Wire.slaveWrite(msg, sizeof(msg)) && I2C0.status_reg.bus_busy){
        Serial.println("slavewrite is broken");
        I2C0.command.opcode = 3;
        I2C0.command.opcode = 4;
//      Wire.end();
//      Wire.begin(I2C_ADDRESS, SDA_PIN, SCL_PIN, 400000L);
        if(!Wire.slaveWrite(msg, sizeof(msg))){
            Serial.println("slavewrite is broken");
        }
    }

}



void PrintValues() {

    if (millis() - previousMillisPrint > 500) {
        previousMillisPrint = millis();

        Serial.print("Distance: ");
        Serial.println(UART.data.tachometerAbs);
        /*
          Serial.print("Speed: ");
          /*
          Serial.print("RPM: ");
          Serial.println(UART.data.rpm);
          Serial.print("Spannung: ");
          Serial.println(UART.data.inpVoltage);
          Serial.print("Ah: ");
          Serial.println(UART.data.ampHours);
          Serial.print("Distance: ");
          Serial.println(UART.data.tachometerAbs);
          Serial.print("Speed: ");
          Serial.println(UART.data.tachometer);
          Serial.print("CurrentRAW: ");
          Serial.println(analogRead(THROTTLE_PIN));
          //Serial.print("CurrentIn: ");
          //Serial.println(current);
        */
        Serial.print("Distance in Km: ");
        Serial.println(distance);

    }
}

void fault() {
    if (millis() - previousMillisFault > FAULT_TIME) {
        UART.setCurrent(float(0));
        //  digitalWrite(BEEPER_PIN, HIGH);
    }

}

bool getIsBreaking() {
    bool ret = (!digitalRead(BRAKE_PIN_L) or !digitalRead(BRAKE_PIN_R));
    return ret;
}

void SetGas() {
    int current = analogRead(THROTTLE_PIN);
    //Serial.println(current);
    current = constrain(current, LOW_THROTTLE_VALUE, HIGH_THROTTLE_VALUE);
    current = map(current, LOW_THROTTLE_VALUE , HIGH_THROTTLE_VALUE, 0, MAX_CURRENT);
    UART.setCurrent((float)current);
}

void SetBreak() {
    float brakeCurrent = 0;
    bool Brake_L = digitalRead(BRAKE_PIN_L);
    bool Brake_R = digitalRead(BRAKE_PIN_R);

    if (!Brake_L and Brake_R) {
        brakeCurrent = BRAKE_CURRENT_WEAK;
    }
    if (Brake_L and !Brake_R) {
        brakeCurrent = BRAKE_CURRENT_MEDIUM;
    }
    if (!Brake_L and !Brake_R) {
        brakeCurrent = BRAKE_CURRENT_HARD;
    }
    UART.setBrakeCurrent(brakeCurrent);
}


void loop() {
    previousMillisCycle = millis();
    static unsigned long lastValues = millis();
    //Serial.println(analogRead(THROTTLE_PIN)); for reading min and max throttle

    const int max_retries = 1;


    for (int retry = 0; retry < max_retries; retry++) {
        if (getIsBreaking()) {
            SetBreak();
        }
        else {
            SetGas();
        }
        Serial.println("gedding");
        if (UART.getVescValues()) {
            lastValues = millis();
            Serial.println("stor");
            storeValues();
            Serial.println("stord");

            if (UART.data.inpVoltage < BATTERY_CELLS * LOW_VOLTAGE) {
                digitalWrite(BEEPER_PIN, HIGH);
                digitalWrite(RELAIS_PIN, LOW);
            }
            break;
        }
    }

    if(0){//millis()-lastValues > 10000){
        digitalWrite(BEEPER_PIN, HIGH);
        if(!DEBUG){
            digitalWrite(RELAIS_PIN, LOW);
        }else{
            delay(100);
            digitalWrite(BEEPER_PIN, LOW);
            while(1);
        }
    }

    while (millis() - previousMillisCycle < CYCLE_TIME);
}