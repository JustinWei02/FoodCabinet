#ifdef COMPILE_REGRESSION_TEST
#define FILLMEIN 0
#else
#define FILLMEIN (#Don't edit this stuff. Fill in the appropriate FILLMEIN values.)
#warning "You must fill in your keys with the right values from the TTN control panel"
#endif

#include "Catena_Fram32k.h"
#include <Arduino_LoRaWAN_ttn.h>
#include <lmic.h>
#include <hal/hal.h>
#include "keys.h"
#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include "memorysaver.h"
#include <ArduinoLowPower.h>

#define BMPIMAGEOFFSET 66

uint64_t lastTime = 0;
uint64_t timer = 0; //timer for when door is closed
uint32_t bufferLength = 8;
static uint8_t messageBuffer[8] = {0, 1, 2, 3, 4, 5, 6, 7};

struct __attribute__((__packed__)) Capacity{
  float foodPercentage1;
  float foodPercentage2;
};

Capacity myfoodAmount; 

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
    bool begin(const Arduino_LoRaWAN::lmic_pinmap& map);
    cMyLoRaWAN() {};
    using Super = Arduino_LoRaWAN_ttn;
    McciCatena::cFram32k theFram; 

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


// Ensure the correct camera module is defined in your memorysaver.h
#if !(defined OV2640_MINI_2MP)
#error "Camera module not supported!"
#endif


// Define CS pins for two cameras
const int CS_CAM1 = 12; // CS pin for camera 1
const int CS_CAM2 = 10; // CS pin for camera 2

const int doorPin = 11; // pin for magnetic door sensor

ArduCAM myCAM1(OV2640, CS_CAM1); // Initialize camera 1
ArduCAM myCAM2(OV2640, CS_CAM2); // Initialize camera 2

int pwmPin = 13;


bool doorChanged = false; // Flag to track door state change
unsigned long doorChangeTime = 0; // Time of last door state change
const unsigned long doorTimeout = 10000; // Timeout in milliseconds (1 minute)
bool doorOpen=false; //is the door open

void setup() {
  Wire.begin();
  
  Serial.begin(115200);
  while (!Serial); // Wait for Serial port to connect.

  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  pinMode(CS_CAM1, OUTPUT);
  digitalWrite(CS_CAM1, HIGH);
  pinMode(CS_CAM2, OUTPUT);
  digitalWrite(CS_CAM2, HIGH);

  pinMode(pwmPin, OUTPUT);

  SPI.begin();
  myCAM1.write_reg(ARDUCHIP_TEST1, 0x55);
  myCAM2.write_reg(ARDUCHIP_TEST1, 0x55);


  uint8_t temp1 = myCAM1.read_reg(ARDUCHIP_TEST1);
  uint8_t temp2 = myCAM2.read_reg(ARDUCHIP_TEST1);

  if (temp1 != 0x55 || temp2 != 0x55) {
    Serial.println("SPI Error with cameras!");
    while (1);
  }

  Serial.println("SPI Success with cameras");




  Serial.begin(115200);
   {
      uint64_t lt = millis();
    while(!Serial && millis() - lt < 5000);
    }
    myLoRaWAN.begin(myPinMap);
    lastTime = millis();
    Serial.println("Serial begin");

    if(myLoRaWAN.IsProvisioned())
      Serial.println("Provisioned for something");
    else
      Serial.println("Not provisioned.");
    

    //Load data into the data structure.
    float foodPercentage1 = 0;
    float foodPercentage2 = 0;
    memcpy(&myfoodAmount.foodPercentage1, &foodPercentage1, 4);
    memcpy(&myfoodAmount.foodPercentage2, &foodPercentage2, 4);
    myLoRaWAN.SendBuffer((uint8_t *) &myfoodAmount, sizeof(myfoodAmount), myStatusCallback, NULL, false, 1);
    

  // Initialize cameras
  myCAM1.set_format(BMP);
  myCAM1.InitCAM();
  myCAM1.OV2640_set_JPEG_size(OV2640_160x120);
  myCAM1.clear_fifo_flag();

  myCAM2.set_format(BMP);
  myCAM2.InitCAM();
  myCAM2.OV2640_set_JPEG_size(OV2640_160x120);
  myCAM2.clear_fifo_flag();

  pinMode(doorPin, INPUT_PULLUP); // Set door status pin as input with pull-up resistor
  LowPower.attachInterruptWakeup(doorPin, doorStateChanged, CHANGE); // Attach wakeup interrupt to door status change

  Serial.println("System going to sleep...");
  LowPower.sleep();
  delay(100); // Give some time for serial to print
}

void loop() {
  // Check if door state changed
  myLoRaWAN.loop();

  if (doorChanged) {
    doorChanged = false;
    timer = 0; // restart the timer each time the door is opened or closed
    if (digitalRead(doorPin)==0){
      doorOpen=false;
      Serial.println("door is closed");
      timer = millis(); // start timer to track how long it's been since the door closed
      }
    else{
      doorOpen=true;
      Serial.println("door is open");
      }
  }
  
  // Only takes picture if it's been 10 seconds after door closed
  if (timer != 0 && millis() - timer > 10000){
    // If the door is closed retrieve image and calculate black pixel
    if (!doorOpen) {

      //First Camera Image Capture
      Serial.println("Capturing Image1");
      
      analogWrite(pwmPin, 10); // Example: Turn on a device
      delay(5000); // Wait 5 seconds for the device to stabilize

      myCAM1.flush_fifo();
      myCAM1.clear_fifo_flag();
      myCAM1.start_capture();

      while (!myCAM1.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
      uint32_t length1 = myCAM1.read_fifo_length();
      if (length1 > MAX_FIFO_SIZE || length1 == 0) {
        myCAM1.clear_fifo_flag();
        return;
      }

      // Read BMP data
      myCAM1.CS_LOW();
      myCAM1.set_fifo_burst();
      uint8_t VH1, VL1;
      uint32_t count1 = 0, blackPixels1 = 0, totalPixels1 = 0;

      // Assuming BMP data begins after a specific header length
      for (int i = 0; i < BMPIMAGEOFFSET; i++) SPI.transfer(0x00); // Skip BMP header

      // Read every fourth pixel to simulate downscaling
      for (int i = 0; i < 160; i++) {
        for (int j = 0; j < 120; j++) {
          VH1 = SPI.transfer(0x00); // Higher byte
          VL1 = SPI.transfer(0x00); // Lower byte

          if ((totalPixels1++ % 4) == 0) { // Sample every fourth pixel
            // Convert two bytes to a single 16-bit pixel value (RGB565)
            uint16_t pixel = (VH1 << 8) | VL1;
            uint8_t r = (pixel >> 11) & 0x1F; // Extracting red component
            uint8_t g = (pixel >> 5) & 0x3F; // Extracting green component
            uint8_t b = pixel & 0x1F; // Extracting blue component

            // Check if the pixel is black or very close to black
            if (r <= 24 && g <= 24 && b <= 24) { // Thresholds for each component to be considered black
              blackPixels1++;
            }
            count1++;
          }
        }
      }

      myCAM1.CS_HIGH();
      myCAM1.clear_fifo_flag();

      // Calculate and print the percentage of black pixels
      float blackPercentage1 = (blackPixels1 / (float)count1) * 100;
      Serial.print("Black Pixels1: ");
      Serial.print(blackPercentage1);
      Serial.println("%");

      analogWrite(pwmPin, 0); // Example: Turn off the device

      float foodPercentage1 = 100 - blackPercentage1;
      memcpy(&myfoodAmount.foodPercentage1, &foodPercentage1, 4);
      myLoRaWAN.SendBuffer((uint8_t *) &myfoodAmount, sizeof(myfoodAmount), myStatusCallback, NULL, false, 1);


      //Second Camera image capture
      delay(3000);
      Serial.println("Capturing Image2");
      
      analogWrite(pwmPin, 10); // Example: Turn on a device
      delay(5000); // Wait 5 seconds for the device to stabilize

      myCAM2.flush_fifo();
      myCAM2.clear_fifo_flag();
      myCAM2.start_capture();

      while (!myCAM2.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
      uint32_t length2 = myCAM2.read_fifo_length();
      if (length2 > MAX_FIFO_SIZE || length2 == 0) {
        myCAM2.clear_fifo_flag();
        return;
      }

      // Read BMP data
      myCAM2.CS_LOW();
      myCAM2.set_fifo_burst();
      uint8_t VH2, VL2;
      uint32_t count2 = 0, blackPixels2 = 0, totalPixels2 = 0;

      // Assuming BMP data begins after a specific header length
      for (int i = 0; i < BMPIMAGEOFFSET; i++) SPI.transfer(0x00); // Skip BMP header

      // Read every fourth pixel to simulate downscaling
      for (int i = 0; i < 160; i++) {
        for (int j = 0; j < 120; j++) {
          VH2 = SPI.transfer(0x00); // Higher byte
          VL2 = SPI.transfer(0x00); // Lower byte

          if ((totalPixels2++ % 4) == 0) { // Sample every fourth pixel
            // Convert two bytes to a single 16-bit pixel value (RGB565)
            uint16_t pixel = (VH2 << 8) | VL2;
            uint8_t r = (pixel >> 11) & 0x1F; // Extracting red component
            uint8_t g = (pixel >> 5) & 0x3F; // Extracting green component
            uint8_t b = pixel & 0x1F; // Extracting blue component

            // Check if the pixel is black or very close to black
            if (r <= 24 && g <= 24 && b <= 24) { // Thresholds for each component to be considered black
              blackPixels2++;
            }
            count2++;
          }
        }
      }

      myCAM2.CS_HIGH();
      myCAM2.clear_fifo_flag();

      // Calculate and print the percentage of black pixels
      float blackPercentage2 = (blackPixels2 / (float)count2) * 100;
      Serial.print("Black Pixels2: ");
      Serial.print(blackPercentage2);
      Serial.println("%");

      analogWrite(pwmPin, 0); // Example: Turn off the device

      float foodPercentage2 = 100 - blackPercentage2;


      memcpy(&myfoodAmount.foodPercentage2, &foodPercentage2, 4);
      myLoRaWAN.SendBuffer((uint8_t *) &myfoodAmount, sizeof(myfoodAmount), myStatusCallback, NULL, false, 1);
    }
    timer = 0; // reset timer
    
    Serial.println("System going to sleep...");
    LowPower.sleep();
  }
}

void doorStateChanged() {
  doorChanged = true;
  doorChangeTime = millis();
}

bool cMyLoRaWAN::begin(const Arduino_LoRaWAN::lmic_pinmap&  map){
     if(!theFram.begin()){
        Serial.println("Fram begin fail");
      }
      if(
      !theFram.initialize()){
        Serial.println("Fram not valid");
      }
      if (!Super::begin(map))
        return false;
      return true;
                }

bool
cMyLoRaWAN::GetOtaaProvisioningInfo(
    OtaaProvisioningInfo *pInfo
    ) {
      if (pInfo){
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
   if (!theFram.getField(McciCatena::cFramStorage::kNetID, Info->NetID)) return false;
   if (!theFram.getField(McciCatena::cFramStorage::kNwkSKey, Info->NwkSKey)) return false;
   if (!theFram.getField(McciCatena::cFramStorage::kAppSKey, Info->AppSKey)) return false;
   if (!theFram.getField(McciCatena::cFramStorage::kDevAddr, Info->DevAddr)) return false;
   //Use the temporary SessionState to get important values from it.

   Info->FCntUp = temporaryState.V1.FCntUp;
   Info->FCntDown = temporaryState.V1.FCntDown;

   return true;


}


