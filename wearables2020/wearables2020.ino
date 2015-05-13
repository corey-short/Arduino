/*************************************************************************************** 
  Arduino code for Provocation 3: Hertzian Armor -
  Critical Making Spring 2015

  Designed specifically to work with the following Adafruit products:
  ----> https://www.adafruit.com/products/1469
  ----> https://www.adafruit.com/products/1376

  Modified code sample implemented by Kevin Townsend & Limor Fried for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
  
  Author: Corey Short
  Date: 05/05/2015
 ***************************************************************************************/

#include <Adafruit_CC3000.h>
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"

#define NUM_PIXELS 21             // The number of NeoPixels in our strip

/* These are the interrupt and control pins */
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
#define STRIP_PIN             6  // NeoPixel strip pin
#define ADAFRUIT_CC3000_VBAT  5  // VBAT can be any digital pin
#define ADAFRUIT_CC3000_CS    10 // CS can be any pin

int *lightShowData = 0;          // Pointer to WiFi scan results. Used to map RGB values to NeoPixel colors

/* Use hardware SPI for the remaining pins
 * On an UNO, SCK pin = 13, MISO pin = 12, and MOSI pin = 11
 */
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed but DI
                                         
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, STRIP_PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

/* Our setup code:
 * Initialization   - Initialize our CC3000 WiFi Breakout
 * SSID Scan        - We compute a scan for WiFi connections around us
 * Compute Averages - Compute RSSI, and Security type averages
 * Disconnect       - Disconnect CC3000
 * Initialization   - Initialize NeoPixel strip
 */
void setup(void) {
  
  initCC3000();                  // Initialize CC3000
  lightShowData = scanWiFi();    // & computeAverages. uint16_t list = scanWiFi() ?
  disconnectCC3000();            // Clean up after yourself and disconnect the CC3000
                                 // It would be nice to power down the CC3000 between scans

}

/* Our loop code:
 * startLightShow   - Start our NeoPixel light show. Repeat until delay over.
 * delay            - This will continue for 5 minutes before doing another WiFi scan.
 */
void loop(void) {
  
  initNeoPixelStrip();           // Initialize NeoPixel strip
  
  startLightShow(lightShowData);
  delay(30000);
  setup();
}

/* Light up our NeoPixel strip based on scanWiFi() results */
void startLightShow(int *p) {
  
  Serial.print(F("P        : "));
  Serial.println(p[0]);
  Serial.println(p[1]);
  Serial.println(p[2]);
  Serial.println(p[3]);
  
  
  uint32_t red = 255 * (p[1] / 100.0);
  uint32_t green = 255 * (p[2] / 100.0);
  uint32_t blue = 255 * (p[3] / 100.0);
  
  Serial.print(F("Colors        : "));
  Serial.println(red);
  Serial.println(green);
  Serial.println(blue);
 
  if (p[0] > 60) {
    colorWipe(strip.Color(red, green, blue), 100);
  }
  else if (p[0] >= 50 && p[0] <= 60) {
    colorWipe2(strip.Color(red, green, blue), 100);
  }
  else {
    colorWipe3(strip.Color(red, green, blue), 100); 
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  uint16_t length = strip.numPixels();
  for (uint16_t i=length-1; i > 0; i--) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

// Fill the dots one after the other with a color
void colorWipe2(uint32_t c, uint8_t wait) {   
  uint16_t length = strip.numPixels();
  for (uint16_t i=length-1; i >= length-14; i--) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

// Fill the dots one after the other with a color
void colorWipe3(uint32_t c, uint8_t wait) {   
  uint16_t length = strip.numPixels();
  for(uint16_t i=length-1; i >= length-7; i--) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

/* Get the SSID list. We use this to compute the average WiFi strength around us. */
int * scanWiFi() {
#ifndef CC3000_TINY_DRIVER
  int *result = listSSIDResults();
#endif
  return result;
}  

/* Initialize CC3000 WiFi Breakout */
void initCC3000() {
  Serial.begin(115200);
  Serial.println(F("Hello, CC3000!\n"));
  /* Display the driver */
  displayDriverMode();
  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  /* Initialise the CC3000 module */
  Serial.println(F("\nInitialising the CC3000 ..."));
  if (!cc3000.begin()) {
    Serial.println(F("Unable to initialise the CC3000! Check your wiring?"));
    while(1);
  }
  /* Check firmware */
  uint16_t firmware = checkFirmwareVersion();
  if (firmware < 0x113) {
    Serial.println(F("Wrong firmware version!"));
    for(;;);
  }
  /* display MAC Address */
  displayMACAddress();
}

/* Disconnect CC3000 WiFi Breakout */
void disconnectCC3000() {
  Serial.println(F("\n\nClosing the connection"));
  cc3000.disconnect();
}

/* Initialize Adafruit NeoPixel strip */
void initNeoPixelStrip() {
  strip.begin();
//  strip.show(); // Initialize all pixels to 'off'
}  
  
/*! @brief  Displays the driver mode (tiny of normal), and the buffer size if tiny mode is not being used
    @note   The buffer size and driver mode are defined in cc3000_common.h */
void displayDriverMode(void) {
  Serial.print(F("RX Buffer : "));
  Serial.print(CC3000_RX_BUFFER_SIZE);
  Serial.println(F(" bytes"));
  Serial.print(F("TX Buffer : "));
  Serial.print(CC3000_TX_BUFFER_SIZE);
  Serial.println(F(" bytes"));
}

/*! @brief  Tries to read the CC3000's internal firmware patch ID */
uint16_t checkFirmwareVersion(void) {
  uint8_t major, minor;
  uint16_t version;
  
#ifndef CC3000_TINY_DRIVER  
  if(!cc3000.getFirmwareVersion(&major, &minor)) {
    Serial.println(F("Unable to retrieve the firmware version!\r\n"));
    version = 0;
  }
  else {
    Serial.print(F("Firmware V. : "));
    Serial.print(major); Serial.print(F(".")); Serial.println(minor);
    version = major; version <<= 8; version |= minor;
  }
#endif
  return version;
}

/*! @brief  Tries to read the 6-byte MAC address of the CC3000 module */
void displayMACAddress(void) {
  uint8_t macAddress[6];
  
  if(!cc3000.getMacAddress(macAddress)) {
    Serial.println(F("Unable to retrieve MAC Address!\r\n"));
  }
  else {
    Serial.print(F("MAC Address : "));
    cc3000.printHex((byte*)&macAddress, 6);
  }
}

/*! @brief  Begins an SSID scan and prints out all the visible networks */
int * listSSIDResults() {
  uint32_t index = 0;
  uint8_t valid = 0;
  uint8_t rssi = 0;
  uint8_t sec = 0;
  char ssidname[33];
  uint16_t rssiTotal = 0;
  int unsecTotal = 0;
  int secWPATotal = 0;
  int secWPA2Total = 0;
  int rssiAvg = 0;
  static int result[100];
  int numNetworks = 0;

  if (!cc3000.startSSIDscan(&index)) {
    Serial.println(F("SSID scan failed!"));
    return false;
  }

  Serial.print(F("Networks found: ")); Serial.println(index);
  Serial.println(F("================================================"));
  
  numNetworks = (int) index;
  while (index) {
    index--;

    valid = cc3000.getNextSSID(&rssi, &sec, ssidname);
    
    Serial.print(F("SSID Name    : ")); Serial.print(ssidname);
    Serial.println();
    Serial.print(F("RSSI         : "));
    Serial.println(rssi);
    Serial.print(F("Security Mode: "));
    Serial.println(sec);
    Serial.println();
    
    rssiTotal += rssi;
    if (sec == 3) {
      secWPA2Total += 1;
    }
    if (sec == 2 || sec == 1) {
      secWPATotal += 1;
    }
    if (sec == 0) {
      unsecTotal += 1;
    }
  }
  Serial.println(F("================================================"));
  Serial.print(F("RSSI Total        : "));
  Serial.println(rssiTotal);  
  Serial.print(F("Security Mode WP2A: "));
  Serial.println(secWPA2Total);
  Serial.print(F("Security Mode WPA : "));
  Serial.println(secWPATotal);
  Serial.print(F("Unsecured Mode    : "));
  Serial.println(unsecTotal);
  Serial.println();
  
  Serial.print(F("(int) RSSI total    : "));
  Serial.println((int) rssiTotal);
  Serial.print(F("(int) index    : "));
  Serial.println((int) index);
  
  rssiAvg = ((int) rssiTotal) / numNetworks;
  Serial.print(F("RSSI Avg    : "));
  Serial.println(rssiAvg);
  
  int red = ((float) secWPA2Total / (float) numNetworks) * 100;
  int green = ((float) unsecTotal / (float) numNetworks) * 100;
  int blue = ((float) secWPATotal / (float) numNetworks) * 100;

  result[0] = rssiAvg;
  result[1] = red;
  result[2] = green;
  result[3] = blue;
  
  cc3000.stopSSIDscan();

  Serial.print(F("Result        : "));
  Serial.println(result[0]);
  Serial.println(result[1]);
  Serial.println(result[2]);
  Serial.println(result[3]);
  
  return result;
}
