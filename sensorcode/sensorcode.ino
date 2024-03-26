// LORAWAN
#include <Wire.h>
#include <cmath> //standard libary to give some important math functions
#include "Arduino.h"
//#include "Adafruit_MCP9808.h"
#ifdef COMPILE_REGRESSION_TEST
#define FILLMEIN 0
#else
#define FILLMEIN (#Don't edit this stuff. Fill in the appropriate FILLMEIN values.)
#warning "You must fill in your keys with the right values from the TTN control panel"
#endif
#include <Arduino_LoRaWAN_ttn.h>
#include <lmic.h>
#include <hal/hal.h>
#include "keys.h"
// HX711
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#endif
//----------LORAWAN START-----------------------------------------------------------------------------
//PANE103395_VD thermistor = PANE103395_VD();
uint64_t lastTime = 0;
uint32_t bufferLength = 8;
static uint8_t messageBuffer[8] = {0, 1, 2, 3, 4, 5, 6, 7};
struct __attribute__((__packed__)) pkt_fmt{
 float temp1;
 float temp2;
 float weight1;
 float weight2;

 float weight3;
 float battery;
};
pkt_fmt myPkt;
#ifdef __cplusplus
extern "C"{
#endif
void myStatusCallback(void * data, bool success){
 if(success)
 Serial.println("Succeeded!");
 else
 Serial.println("Failed!");
}
#ifdef __cplusplus
}
#endif
class cMyLoRaWAN : public Arduino_LoRaWAN_ttn {
 public:
 cMyLoRaWAN() {};
 protected:
 // you'll need to provide implementations for each of the following.
 virtual bool GetOtaaProvisioningInfo(Arduino_LoRaWAN::OtaaProvisioningInfo*) override;
 virtual void NetSaveSessionInfo(const SessionInfo &Info, const uint8_t *pExtraInfo, size_t nExtraInfo) override;
 virtual void NetSaveSessionState(const SessionState &State) override;
 virtual bool NetGetSessionState(SessionState &State) override;
 virtual bool GetAbpProvisioningInfo(Arduino_LoRaWAN::AbpProvisioningInfo*) override;
};
// set up the data structures.
cMyLoRaWAN myLoRaWAN {};

// The pinmap. This form is convenient if the LMIC library
// doesn't support your board and you don't want to add the
// configuration to the library (perhaps you're just testing).
// This pinmap matches the FeatherM0 LoRa. See the arduino-lmic
// docs for more info on how to set this up.
const cMyLoRaWAN::lmic_pinmap myPinMap = {
 .nss = 8,
 .rxtx = cMyLoRaWAN::lmic_pinmap::LMIC_UNUSED_PIN,
 .rst = 4,
 .dio = { 3, 6, cMyLoRaWAN::lmic_pinmap::LMIC_UNUSED_PIN },
 .rxtx_rx_active = 0,
 .rssi_cal = 0,
 .spi_freq = 8000000,
};
//----------LORAWAN END-----------------------------------------------------------------------------
//----------HX711 START-----------------------------------------------------------------------------
float weights[4];
//11 9 and 5 are PWM pins; sck needs to be pwm
//pins:
const int HX711_dout_1 = 13; //mcu > HX711 no 1 dout pin
const int HX711_sck_1 = 5; //mcu > HX711 no 1 sck pin
const int HX711_dout_2 = 10; //mcu > HX711 no 2 dout pin
const int HX711_sck_2 = 11; //mcu > HX711 no 2 sck pin
const int HX711_dout_3 = 12; //mcu > HX711 no 3 dout pin
const int HX711_sck_3 = 9; //mcu > HX711 no 3 sck pin
//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); //HX711 2
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck_3); //HX711 3
unsigned long t = 0;
//----------HX711 END-----------------------------------------------------------------------------

//----------THERMISTOR START---------------------------------------------------------------------------
/*
 Get the "unknown" resistor using voltages and the known resistance.
 This function assumes **voltages**, not raw values from analogRead.
 On Feather M0+ LoRa with default settings, pass vdiv = (3.3/1024)*analogRead(pin).
*/
static inline double getUnknownResistor(double v, double vdiv, double known_r) {
 return known_r * vdiv / (v - vdiv);
}
/**
 This function uses information from the PANE 103395 datasheet, rev 0.
 Returns NAN if the resistance ratio is outside of the specified resistance
 ratios on the datasheet.
 Numeric values come from the datasheet.
 r_r25 is the ratio of the thermistor resistance to the thermistor resistance
 at 25 degrees C. In other words, it is r/10,000 since the thermistor is a
 10k thermistor.
*/
static double tempFromResistance(double r_r25) {
 double a, b, c, d;
 if (r_r25 > 66.97) {
 return NAN; //"Not a Number" ~= undefined. We're outside of the datasheet range!
 }
 else if (r_r25 > 3.279) {
 // Coefficients taken from datasheet
 a = 3.357296E-03;
 b = 2.508334E-04;
 c = 4.189372E-06;
 d = -6.240867E-08;
 }
 else if (r_r25 > 0.3507) {
 a = 3.354016E-03;
 b = 2.541522E-04;
 c = 3.730922E-06;

 d = -7.881561E-08;
 }
 else if (r_r25 > 0.0637) {
 a = 3.361395E-03;
 b = 2.582266E-04;
 c = 5.885012E-07;
 d = -2.823586E-08;
 }
 else if (r_r25 > 0.0169) {
 a = 3.351295E-03;
 b = 2.500181E-04;
 c = -1.7255607E-07;
 d = -4.356943E-08;
 }
 else {
 return NAN;
 }
 // Steinhart-Hart equation taken from datasheet (don't need to learn)
 return 1 / (a + b * log(r_r25) + c * b * pow(log(r_r25), 2) + d * pow(log(r_r25), 3));
}
double getTemperature1() {
 //10000 below is from the 10k fixed resistor
 double thermRes1 = getUnknownResistor(3.3, (3.3 / 1024) * analogRead(A0), 10000);
 //10000 below is the nominal thermistor resistance at 25 degrees C
 return tempFromResistance(thermRes1 / 10000);
}
double getTemperature2() {
 //10000 below is from the 10k fixed resistor
 double thermRes2 = getUnknownResistor(3.3, (3.3 / 1024) * analogRead(A1), 10000);
 //10000 below is the nominal thermistor resistance at 25 degrees C
 return tempFromResistance(thermRes2 / 10000);
}

//----------THERMISTOR END-----------------------------------------------------------------------------
//----------CHECK BATTERY STATUS START---------------------------------------------------------
int value = 0;
float voltage;
float perc;
double getBattery() {
 value = analogRead(A0);
 voltage = value * 5.0 / 1023;
 perc = map(voltage, 3.6, 4.2, 0, 100);
 return perc;
}
//----------CHECK BATTERY STATUS END------------------------------------------------------------
void setup() {
 Serial.begin(57600); delay(10);
 Serial.println();
 Serial.println("Starting...");
 //----------HX711 START-----------------------------------------------------------------------------
 float calibrationValue_1; // calibration value load cell 1
 float calibrationValue_2; // calibration value load cell 2
 float calibrationValue_3; // calibration value load cell 3
 calibrationValue_1 = 7982.29; // uncomment this if you want to set this value in the sketch
 calibrationValue_2 = 3081.25; // uncomment this if you want to set this value in the sketch
 calibrationValue_3 = 2827.08; // uncomment this if you want to set this value in the sketch
 LoadCell_1.begin();
 LoadCell_2.begin();
 LoadCell_3.begin();
 LoadCell_1.setReverseOutput();
 LoadCell_2.setReverseOutput();
 LoadCell_3.setReverseOutput();
 unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time

 boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
 byte loadcell_1_rdy = 0;
 byte loadcell_2_rdy = 0;
 byte loadcell_3_rdy = 0;
 while ((loadcell_1_rdy + loadcell_2_rdy + loadcell_3_rdy) < 3) { //run startup, stabilization and tare, both modules simultaniously
 if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
 if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
 if (!loadcell_3_rdy) loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
 }
 if (LoadCell_1.getTareTimeoutFlag()) {
 Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
 }
 if (LoadCell_2.getTareTimeoutFlag()) {
 Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
 }
 if (LoadCell_3.getTareTimeoutFlag()) {
 Serial.println("Timeout, check MCU>HX711 no.3 wiring and pin designations");
 }
 LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value (float)
 LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value (float)
 LoadCell_3.setCalFactor(calibrationValue_3); // user set calibration value (float)
 Serial.println("Startup is complete");
 //----------HX711 END-----------------------------------------------------------------------------
 //----------LORAWAN START-----------------------------------------------------------------------------
 // simply pass the pinmap to the begin() method.
 Serial.begin(115200);
 {
 uint64_t lt = millis();
 while (!Serial && millis() - lt < 5000);
 }
 myLoRaWAN.begin(myPinMap);
 lastTime = millis();
 Serial.println("Serial begin");

 if (myLoRaWAN.IsProvisioned())
 Serial.println("Provisioned for something");
 else
 Serial.println("Not provisioned.");
 LMIC.datarate = 10;
 myLoRaWAN.SendBuffer((uint8_t *) &myPkt, sizeof(myPkt), myStatusCallback, NULL, false, 1);
}
 //----------LORAWAN END-----------------------------------------------------------------------------
 //----------HX711 START-----------------------------------------------------------------------------
void loop() {
 static boolean newDataReady = 0;
 const int serialPrintInterval = 0; //increase value to slow down serial print activity
 // check for new data/start next conversion:
 if (LoadCell_1.update()) newDataReady = true;
 LoadCell_2.update();
 if (LoadCell_2.update() && LoadCell_2.update()) newDataReady = true;
 LoadCell_3.update();
 // receive command from serial terminal, send 't' to initiate tare operation:
 if (Serial.available() > 0) {
 char inByte = Serial.read();
 if (inByte == 't') {
 LoadCell_1.tareNoDelay();
 LoadCell_2.tareNoDelay();
 LoadCell_3.tareNoDelay();
 }
 }
 //check if last tare operation is complete
 if (LoadCell_1.getTareStatus() == true) {
 Serial.println("Tare load cell 1 complete");
 }
 if (LoadCell_2.getTareStatus() == true) {
 Serial.println("Tare load cell 2 complete");
 }

 if (LoadCell_3.getTareStatus() == true) {
 Serial.println("Tare load cell 3 complete");
 }
 //----------HX711 END-----------------------------------------------------------------------------
 //----------LORAWAN START-----------------------------------------------------------------------------
 myLoRaWAN.loop();
 if (millis() - lastTime > 60000) {
 myPkt.temp1 = (float)getTemperature1();
 myPkt.temp2 = (float)getTemperature2();
 myPkt.weight1 = LoadCell_1.getData();
 myPkt.weight2 = LoadCell_2.getData();
 myPkt.weight3 = LoadCell_3.getData();
 myPkt.battery = (float)getBattery();
 Serial.println(myPkt.temp1);
 Serial.println(myPkt.temp2);
 Serial.println(myPkt.weight1);
 Serial.println(myPkt.weight2);
 Serial.println(myPkt.weight3);
 Serial.println(myPkt.battery);
 messageBuffer[0]++;
 myLoRaWAN.SendBuffer((uint8_t *) &myPkt, sizeof(myPkt), myStatusCallback, NULL, false, 1);
 lastTime = millis();
 }
}
 //----------LORAWAN END-----------------------------------------------------------------------------
//----------LORAWAN START-----------------------------------------------------------------------------
bool
cMyLoRaWAN::GetOtaaProvisioningInfo(
 OtaaProvisioningInfo *pInfo
) {
 if (pInfo) {
 memcpy_P(pInfo->AppEUI, APPEUI, 8);
 memcpy_P(pInfo->DevEUI, DEVEUI, 8);

 memcpy_P(pInfo->AppKey, APPKEY, 16);
 }
 return true;
}
void
cMyLoRaWAN::NetSaveSessionInfo(
 const SessionInfo &Info,
 const uint8_t *pExtraInfo,
 size_t nExtraInfo
) {
 // save Info somewhere.
}
void
cMyLoRaWAN::NetSaveSessionInfo(
    const SessionInfo &Info,
    const uint8_t *pExtraInfo,
    size_t nExtraInfo
    ) {
    // save Info somewhere.
    theFram.saveField(McciCatena::cFramStorage::kDevAddr, Info.V2.DevAddr);
    theFram.saveField(McciCatena::cFramStorage::kNetID, Info.V2.NetID);
    theFram.saveField(McciCatena::cFramStorage::kNwkSKey, Info.V2.NwkSKey);
    theFram.saveField(McciCatena::cFramStorage::kAppSKey, Info.V2.AppSKey);
  

}

void
cMyLoRaWAN::NetSaveSessionState(const SessionState &State) {
    // save State somwwhere. Note that it's often the same;
    // often only the frame counters change.
    theFram.saveField(McciCatena::cFramStorage::kLmicSessionState, State);

}

bool
cMyLoRaWAN::NetGetSessionState(SessionState &State) {
    // either fetch SessionState from somewhere and return true or...
    return theFram.getField(McciCatena::cFramStorage::kLmicSessionState, State);
}

bool
cMyLoRaWAN::GetAbpProvisioningInfo(Arduino_LoRaWAN::AbpProvisioningInfo* Info){
  //either get ABP provisioning info from somewhere and return true or...
  if (!Info) return false;//Library calls with null pointer sometimes
   SessionState temporaryState;
   if (!theFram.getField(McciCatena::cFramStorage::kLmicSessionState, temporaryState)) return false;
   if (!theFram.getField(McciCatena::cFramStorage::kNwkSKey, Info->NwkSKey)) return false;
   if (!theFram.getField(McciCatena::cFramStorage::kAppSKey, Info->AppSKey)) return false;
   if (!theFram.getField(McciCatena::cFramStorage::kDevAddr, Info->DevAddr)) return false;
   //Use the temporary SessionState to get important values from it.

   Info->FCntUp = temporaryState.V1.FCntUp;
   Info->FCntDown = temporaryState.V1.FCntDown;

   return true;





}

//----------LORAWAN END-----------------------------------------------------------------------------