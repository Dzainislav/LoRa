/*******************************************************************************
 * The Things Network - OTAA Feather M0
 * 
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 * 
 * Reference: https://github.com/mcci-catena/arduino-lmic
 *
 * Modified for DS18B20 - average measured values and Low Power Mode, 26. 7. 2022
 * and for Tipping bucket rain gauge - measuring amount of precipitation using
 * interupt, 29.8.202
 *******************************************************************************/

#include <lmic.h>   //library used for LoRa frequency channels set up
#include <hal/hal.h>
#include <SPI.h>

#include <RTCZero.h>                  // RTCZero (Real Time Clock) used for sleep
RTCZero rtc;

#include <OneWire.h> 
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 12               // Pin 12 on the Adafruit  
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#include <CayenneLPP.h>               // Cayenne Low Power Payload (LPP)
CayenneLPP lpp(51);

//-------------------------------------------- Sleep time -------------------------------------------------------

const byte seconds = 55;

//---------------------------------------------------------------------------------------------------------------

// Saved time
byte savedSeconds = 0;

// Sleep counter
unsigned int countSleeps = 1;                 // Counting sleeps
unsigned int sendDataEvery = 10;              // Send data every 10 sleep

// DS18B20
float temp = 0.0;                             // Variable for temperature
float numberOfSamples = 0;                    // Variable for number of measured samples by DS18B20

// Tipping bucket water gauge
const int rainPin = 10;                       // Sensor connected to digital pin 10
float rain = 0.0;                             // Variable for amount of precipitation
bool rainWoke = false;

// First
bool isFirst = true;

// RTC
bool isSetRTC = true;
unsigned long previousMillis = 0;

// Jobs
static osjob_t measurejob, sendjob;


//--------------------------------------- Here change your keys -------------------------------------------------
//in TTS click on the "toggle array formating" button -> swithc byte order to "lsb" -> click on the "Coppy to clipboard" button -> paste it between the brackets
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };   // AppEUI, LSB
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}                        
//in TTS click on the "toggle array formating" button -> swithc byte order to "lsb" -> click on the "Coppy to clipboard" button -> paste it between the brackets
static const u1_t PROGMEM DEVEUI[8]={ 0x18, 0x31, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };   // DevEUI, LSB
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
//in TTS click on the "toggle array formating" button -> swithc byte order to "msb" -> click on the "Coppy to clipboard" button -> paste it between the brackets
static const u1_t PROGMEM APPKEY[16] = { 0xB9, 0xD9, 0xC7, 0xAF, 0x32, 0xC8, 0x58, 0x4A, 0x76, 0xBF, 0x53, 0x8A, 0x13, 0x7C, 0xFF, 0x6E };  // AppKey, MSB
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

//---------------------------------------------------------------------------------------------------------------

const lmic_pinmap lmic_pins = {                 // Pin mapping for the Adafruit Feather M0
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,
    .spi_freq = 8000000,
};

void onEvent (ev_t ev) { //event branching for some occasions, is used for calling different functions, onEven is called after downlink
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            break;
        case EV_BEACON_FOUND:
            break;
        case EV_BEACON_MISSED:
            break;
        case EV_BEACON_TRACKED:
            break;
        case EV_JOINING:
            break;
        case EV_JOINED:
            LMIC_setLinkCheckMode(0);             // Disable link check validation
            LMIC.dn2Dr = DR_SF9;                  // TTS uses SF9 for its RX2 window.
            LMIC_setDrTxpow(DR_SF9,14);           // Set data rate and transmit power for uplink
            LMIC_setAdrMode(0);                   // Adaptive data rate disabled
            break;
        case EV_JOIN_FAILED:
            break;
        case EV_REJOIN_FAILED:
            break;
        case EV_TXCOMPLETE:
            resetValues();                        // Reset values
            os_setCallback(&measurejob, do_measure);
            break;
        case EV_LOST_TSYNC:
            break;
        case EV_RESET:
            break;
        case EV_RXCOMPLETE:
            break;
        case EV_LINK_DEAD:
            break;
        case EV_LINK_ALIVE:
            break;
        case EV_TXSTART:
            break;
        case EV_TXCANCELED:
            break;
        case EV_RXSTART:
            break;
        case EV_JOIN_TXCOMPLETE:
            break;
        default:
            break;
    }
}


void do_send(osjob_t* j){

    if(isFirst){
        lpp.reset();
        lpp.addDigitalInput(51, 1);                                   // First value
        
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);        // Prepare upstream data transmission at the next possible time.
        isFirst = false;
    }else{
    
        lpp.reset();
        lpp.addTemperature(1, temp / numberOfSamples);                // Add the average temperature into channel 1
        lpp.addAnalogInput(2, rain);                                  // Add the amount of precipitation into channel 2
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);        // Prepare upstream data transmission at the next possible time.
    }
}


void resetValues() {              // Reset values
    temp = 0.0;
    rain = 0.0;
    numberOfSamples = 0;

    rainWoke = false;
    isSetRTC = true;
}


void measureTemp(){
    sensors.requestTemperatures();                          // Send the command to get temperature readings from DS18B20
    temp = temp + sensors.getTempCByIndex(0);               // Add the current temperature to all measured temperatures
    
    numberOfSamples++;                                      // Add measured sample
}


void measureRain(){
    rainWoke = true;
}

void setRTCAlarm(byte setNewSeconds){
    rtc.setTime(0, 0, setNewSeconds);
    rtc.setAlarmTime(0, 0, seconds);
}

void goSleep(){
    rtc.standbyMode();                                      // Sleep

    savedSeconds = rtc.getSeconds();

    previousMillis = millis();
    while(millis() - previousMillis <= 1000){}              // Delay(1000)
}


void do_measure(osjob_t* j){

    if(isSetRTC){
        setRTCAlarm(0);
        isSetRTC = false;
    }else{                                                    // rainWoke
        setRTCAlarm(savedSeconds);
    }
    
    goSleep();
    
    if(!rainWoke){

        measureTemp();
    
        if(countSleeps % sendDataEvery == 0){             // Every 10 sleep (10 minutes)
            isSetRTC = true;
            os_setCallback(&sendjob, do_send);            // Call do_send
            countSleeps = 0;                              // Zero sleep counter
        }
        countSleeps++; 
        
        if((countSleeps - 1) % sendDataEvery != 0){       // When is called do_send, don't call do_measure
            isSetRTC = true;
            os_setCallback(&measurejob, do_measure);
        }
    }else{
        rainWoke = false;
        rain = rain + 0.30;                                     // Adding 0,3 mm to total amount of rain
        os_setCallback(&measurejob, do_measure);
    }
}


void setup() {
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(rainPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rainPin), measureRain, LOW);  //after overturning the the tipping bucket the magnetic switch sends a value and sleep will be interupted
    
    os_init();
    LMIC_reset();

    //EU868 nastavení frekvenčních pásem
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

    LMIC_setLinkCheckMode(0);         // Disable link check validation
    LMIC.dn2Dr = DR_SF9;              // TTS uses SF9 for its RX2 window.
    LMIC_setDrTxpow(DR_SF9,14);       // Set data rate and transmit power for uplink 7 až 12
    LMIC_setAdrMode(0);               // Adaptive data rate disabled

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); 

    rtc.begin();                      // RTCZero inicializace
    rtc.enableAlarm(rtc.MATCH_SS);    //nastavení alarmu v sekundách

    sensors.begin();                  // DS18B20 inicializace

    do_send(&sendjob);                // Start sendjob
}

void loop() {
    os_runloop_once();
}
