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
#include "soc/dport_reg.h"
#include "soc/i2c_reg.h"
#include "esp32-hal-i2c.h"

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
#define DEBUG (0)

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

SemaphoreHandle_t storeMutex = NULL;

void storeValues();
void resetBus();

void setup() {
    pinMode(FIELDCOIL_PIN, OUTPUT);
    pinMode(BEEPER_PIN, OUTPUT);
    pinMode(RELAIS_PIN, OUTPUT);

    pinMode(THROTTLE_PIN, INPUT);
    pinMode(BRAKE_PIN_L, INPUT);
    pinMode(BRAKE_PIN_R, INPUT);

    digitalWrite(FIELDCOIL_PIN, LOW);
    digitalWrite(BEEPER_PIN, LOW);

    storeMutex = xSemaphoreCreateMutex();

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

    Wire.onRequest(gotRequest);
    Serial.println(Wire.begin(I2C_ADDRESS, SDA_PIN, SCL_PIN, 400000L));
}

void gotRequest() {
    Serial.println("Got rq");
    storeValues();

}

void resetBus(){
    Serial.println("performing proper reset");
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG,DPORT_I2C_EXT0_RST); //reset hardware
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG,DPORT_I2C_EXT0_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG,DPORT_I2C_EXT0_RST);//  release reset
    I2C0.ctr.val = 0;
    I2C0.ctr.ms_mode = 1;
    I2C0.ctr.sda_force_out = 1 ;
    I2C0.ctr.scl_force_out = 1 ;
    I2C0.ctr.clk_en = 1;
    I2C0.fifo_conf.fifo_addr_cfg_en = 0;

    //the max clock number of receiving  a data
    I2C0.timeout.tout = 400000L;//clocks max=1048575
    //disable apb nonfifo access
    I2C0.fifo_conf.nonfifo_en = 0;

    I2C0.slave_addr.val = I2C_ADDRESS;
    I2C0.slave_addr.en_10bit = false;

    uint32_t period = (APB_CLK_FREQ/400000L) / 2;
    uint32_t halfPeriod = period/2;
    uint32_t quarterPeriod = period/4;

    //the clock num during SCL is low level
    I2C0.scl_low_period.period = period;
    //the clock num during SCL is high level
    I2C0.scl_high_period.period = period;

    //the clock num between the negedge of SDA and negedge of SCL for start mark
    I2C0.scl_start_hold.time = halfPeriod;
    //the clock num between the posedge of SCL and the negedge of SDA for restart mark
    I2C0.scl_rstart_setup.time = halfPeriod;

    //the clock num after the STOP bit's posedge
    I2C0.scl_stop_hold.time = halfPeriod;
    //the clock num between the posedge of SCL and the posedge of SDA
    I2C0.scl_stop_setup.time = halfPeriod;

    //the clock num I2C used to hold the data after the negedge of SCL.
    I2C0.sda_hold.time = quarterPeriod;
    //the clock num I2C used to sample data on SDA after the posedge of SCL
    I2C0.sda_sample.time = quarterPeriod;

    I2C0.int_ena.val = (I2C_RXFIFO_FULL_INT_ENA_M | I2C_TRANS_COMPLETE_INT_ENA_M);
    Serial.println("ending");
    Wire.end();
    Serial.print("ended");
    if (Wire.begin(SDA_PIN, SCL_PIN, 400000L) == ESP_OK) {
        Serial.println("master mode init worked");
    }
    Serial.print("begun");
    Wire.end();
    Serial.print("ended");
    while(!Wire.begin(I2C_ADDRESS, SDA_PIN, SCL_PIN, 400000L));
    Serial.print("begun");



    uint8_t sda = SDA_PIN;
    uint8_t scl = SCL_PIN;

    digitalWrite(sda,HIGH);
    digitalWrite(scl,HIGH);
    pinMode(sda,PULLUP|OPEN_DRAIN|OUTPUT|INPUT);
    pinMode(scl,PULLUP|OPEN_DRAIN|OUTPUT|INPUT);


    if(!digitalRead(sda)||!digitalRead(scl)){ // bus in busy state
        log_e("invalid state sda=%d, scl=%d\n",digitalRead(sda),digitalRead(scl));
        digitalWrite(sda,HIGH);
        digitalWrite(scl,HIGH);
        delayMicroseconds(5);
        digitalWrite(sda,LOW);
        for(uint8_t a=0; a<9;a++){
            delayMicroseconds(5);
            digitalWrite(scl,LOW);
            delayMicroseconds(5);
            digitalWrite(scl,HIGH);
        }
        delayMicroseconds(5);
        digitalWrite(sda,HIGH);
    }

    digitalWrite(sda, HIGH);
    pinMode(sda, OPEN_DRAIN | PULLUP | INPUT | OUTPUT );
    pinMatrixOutAttach(sda, I2CEXT0_SDA_OUT_IDX, false, false);
    pinMatrixInAttach(sda, I2CEXT0_SDA_OUT_IDX, false);

    digitalWrite(scl, HIGH);
    pinMode(scl, OPEN_DRAIN | PULLUP | INPUT | OUTPUT);
    pinMatrixOutAttach(scl, I2CEXT0_SCL_OUT_IDX, false, false);
    pinMatrixInAttach(scl, I2CEXT0_SCL_OUT_IDX, false);
}


void storeValues() {
    if( xSemaphoreTake( storeMutex, portMAX_DELAY ) != pdTRUE ){
        Serial.println("mutex unavailable");
        return;
    }

    const int baseInputVoltage = 2;
    const int baseMotorTemp = -20;
    const int AhFactor = 10;
    const int voltFactor = 100;
    const int kmhFactor = 3;
    const int kmFactor = 100;

    const int msgLen = 9;

    uint16_t km = constrain((long)UART.data.tachometer/TacPerErev*100/erotPerKm, 0, 0xFFFF);
    uint16_t kmabs = constrain((long)UART.data.tachometer/TacPerErev/erotPerKm, 0, 0xFFFF);

    uint8_t msg[msgLen+1] = {
            constrain(abs(UART.data.avgInputCurrent), 0, 255),
            km,
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
    if (I2C0.status_reg.bus_busy) {
        if (busBusyCycles > 50){
            resetBus();
            busBusyCycles = 0;
        }
        busBusyCycles++;

    }
    else {
        busBusyCycles = 0;
    }

    Wire.slaveWrite(msg, sizeof(msg));

    xSemaphoreGive( storeMutex );
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
        if (UART.getVescValues()) {
            lastValues = millis();

            storeValues();

            if (UART.data.inpVoltage < BATTERY_CELLS * LOW_VOLTAGE) {
                digitalWrite(BEEPER_PIN, HIGH);
                digitalWrite(RELAIS_PIN, LOW);
            }
            break;
        }
    }

    if(millis()-lastValues > 30000){
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