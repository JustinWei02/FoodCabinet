#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include "memorysaver.h"

#define BMPIMAGEOFFSET 66

// Ensure the correct camera module is defined in your memorysaver.h
#if !(defined OV2640_MINI_2MP)
  #error "Camera module not supported!"
#endif

const int CS = 12; // CS pin for ArduCAM




ArduCAM myCAM( OV2640, CS ); // Assuming using OV2640

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial); // Wait for Serial port to connect.

  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  SPI.begin();
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  uint8_t temp = myCAM.read_reg(ARDUCHIP_TEST1);
  if (temp != 0x55){
    Serial.println("SPI Error!");
    while(1);
  }

  // Initialize camera
  myCAM.set_format(BMP);
  myCAM.InitCAM();
  myCAM.OV2640_set_JPEG_size(OV2640_160x120); // Set to smallest resolution
  myCAM.clear_fifo_flag();
}

void loop() {
  // Capture image every 5 seconds
  delay(5000);

  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();

  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  uint32_t length = myCAM.read_fifo_length();
  if(length > MAX_FIFO_SIZE || length == 0) {
    myCAM.clear_fifo_flag();
    return;
  }

  // Read BMP data
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  uint8_t VH, VL;
  uint32_t count = 0, matchingPixels = 0, totalPixels = 0;

  // Assuming BMP data begins after a specific header length
  for (int i = 0; i < BMPIMAGEOFFSET; i++) SPI.transfer(0x00); // Skip BMP header

  // Read every fourth pixel to simulate downscaling
  for (int i = 0; i < 160; i++) {
    for (int j = 0; j < 120; j++) {
      VH = SPI.transfer(0x00); // Higher byte
      VL = SPI.transfer(0x00); // Lower byte

      if ((totalPixels++ % 4) == 0) { // Sample every fourth pixel
        // Convert two bytes to a single 16-bit pixel value (RGB565)
        uint16_t pixel = (VH << 8) | VL;
        uint8_t r = (pixel >> 11) & 0x1F; // Extracting red component
        uint8_t g = (pixel >> 5) & 0x3F; // Extracting green component
        uint8_t b = pixel & 0x1F; // Extracting blue component

        // Check if the pixel matches the target color within a tolerance
        if (abs(r - 13) <= 12 && abs(g - 41) <= 12 && abs(b - 27) <= 12) {
          matchingPixels++;
        }
        count++;
      }
    }
  }

  myCAM.CS_HIGH();
  myCAM.clear_fifo_flag();

  // Calculate and print the percentage of matching pixels
  float matchingPercentage = (matchingPixels / (float)count) * 100;
  Serial.print("Matching Pixels: ");
  Serial.print(matchingPercentage);
  Serial.println("%");
}
