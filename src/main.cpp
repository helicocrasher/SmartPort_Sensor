#include <Arduino.h>

#include <stdint.h> 
#include <Stream.h> 
#include "hardware.h"
#include "SmartPortSlave.h"
//#include <Adafruit_INA3221.h>
//#include <Wire.h>


//#define HAL_UART_MODULE_ENABLED
#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION < 0x02110000)
//    #error "This sketch requires STM32 core version 2.11.0 or higher"
#endif
#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION == 0x02100100)
    #error "STM32 core version is 2.10.1"
#endif

void printSmartPortData();
void telemetryCarousel();

SmartPortSlave mySmartPortSlave; // Create an instance of the SmartPortSlave class

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
#ifdef TARGET_G431CB
  Serial_DBG.setTx(PC10);
  Serial_DBG.setRx(PC11);
  Serial_SP.setTx(PA9);
  Serial_SP.setRx(PA10);
#endif
#ifdef TARGET_G031F8
  Serial_DBG.setTx(PA2);
  Serial_DBG.setRx(PA3);
  Serial_SP.setTx(PB6);
  Serial_SP.setRx(PB7);
#endif
  Serial_DBG.begin(115200);      // Hardware Serial_DBG for debugging output
  delay(20);
  Serial_DBG.println("SmartPort Sensor Hub starting up");
  delay(10);
  mySmartPortSlave.begin(&Serial_SP, 0x6a, &Serial_DBG); // Initialize the SmartPort slave with the SmartPort serial stream, sensor ID and debug serial stream
} 

uint32_t i;
uint32_t LastTimeNow=0;
bool LedOn = true;
//char sniffedData[9]; // Buffer to store the sniffed data
char readBuffer[9]; // Buffer to store the received data from SmartPort serial
char iFormatedString[100]={0}; // Buffer for formatted strings for debugging output

void loop() {
  if (millis() - LastTimeNow > 500) {
    LastTimeNow = millis();
    snprintf(iFormatedString, sizeof(iFormatedString), "\r\nLoop: %13u", i);
    Serial_DBG.print(iFormatedString); // Print the loop count
    digitalWrite(LED_BUILTIN, LedOn ? HIGH : LOW);
    LedOn = !LedOn;
//    if (mySmartPortSlave.txBufferSent()) { // Check if the previous sensor data packet has been sent before preparing the next one
//      mySmartPortSlave.sendV_A3(i, 0x0900); // Send a new sensor value for testing with sensor ID 0x0900 (A3)
//    } 
//    mySmartPortSlave.sendV_A3(i, 0x0900); 
  } 
  i+=1;
  telemetryCarousel(); // Update the telemetry carousel to send different sensor values in a rotating manner
  mySmartPortSlave.update(); // Update the SmartPort slave to handle incoming data and send responses 
  if (mySmartPortSlave.sniffedDataAvailable()) { 
    mySmartPortSlave.getSniffedData(readBuffer); // Get the sniffed data and store it in the provided buffer
//    Serial_DBG.println("Sniffed data received:"); 
    printSmartPortData(); // Print the sniffed data in a formatted way
  }
//  delay(1); // Add a small delay to avoid overwhelming the CPU
}


void telemetryCarousel() {
static uint8_t carouselIndex = 0;
  static uint32_t lastUpdateTime = 0, timeSinceLastUpdate;
  timeSinceLastUpdate = millis() - lastUpdateTime;
  if (timeSinceLastUpdate <50) return;
  if (!mySmartPortSlave.txBufferSent()) return; 
  
  switch (carouselIndex) {
    case 0:
      mySmartPortSlave.sendV_A3(i, 0x0901); 
      break;
    case 1:
      mySmartPortSlave.sendCellVoltage(3700, 0, 0x00); 
      break;
    case 2:
      mySmartPortSlave.sendCellVoltage(3800, 1, 0x00); 
      break;
    case 3:
      mySmartPortSlave.sendCellVoltage(3900, 2, 0x00); 
      break;
    case 4:
      mySmartPortSlave.sendLON(-9.9*degree_in_minutes_div10k, 0x00); 
    case 5:
      mySmartPortSlave.sendLAT(-45*degree_in_minutes_div10k, 0x00); 
      break;
    case 6:
      mySmartPortSlave.sendDate(26, 12, 31, 0x00); 
      break;
    case 7:
      mySmartPortSlave.sendTime(23, 59, 59, 0x00); 
      break;
    case 8:
      mySmartPortSlave.sendAltitude(-5000, 0x00); 
      break;
    case 9:
      mySmartPortSlave.sendHeading(123, 0x00); 
      break;
    case 10:
      mySmartPortSlave.sendGnssAltitude(12300, 0x00); 
      break;
    case 11:  
      mySmartPortSlave.sendGnssSpeed(56700, 0x00); 
      break;
    default: break;
  }
  carouselIndex = (carouselIndex + 1) % 20; // Move to the next index in the carousel
  lastUpdateTime = millis();
}


void printSmartPortData() {
  // Print the SmartPort data stored in readBuffer
  if (readBuffer[1] == 0x10) { // Check if the second byte indicates a valid sensor package
    sprintf(iFormatedString, "\r\nr %02X: ", (uint8_t)readBuffer[0]);
    Serial_DBG.print(iFormatedString); // Print the first byte in hexadecimal format
    for (uint8_t i = 1; i < 9; i++) { // Print all bytes in the buffer in hexadecimal format
      sprintf(iFormatedString, "%02X, ", (uint8_t)readBuffer[i]);
      Serial_DBG.print(iFormatedString); // Print the received byte in hexadecimal format
    }
    /**/ 
    sprintf(iFormatedString, "    %02X: %02X, ", (uint8_t)readBuffer[0], (uint8_t)readBuffer[1]);
    Serial_DBG.print(iFormatedString); // Print the first byte in hexadecimal format
    uint16_t sensorID = (readBuffer[3] << 8) | readBuffer[2]; // Combine the first two bytes to get the sensor ID
    sprintf(iFormatedString, "%04X, ", sensorID);
    Serial_DBG.print(iFormatedString); // Print the sensor ID
    if (sensorID == 0x0850){ // For sensor ID 0x0850 Date and Time,
      sprintf(iFormatedString, "%02X, ", (uint8_t)readBuffer[4]); // Bufffer[4] =0 time =1 date =2
      Serial_DBG.print(iFormatedString); // Print the received byte in hexadecimal format
      uint32_t sensorValue =  readBuffer[5]; // Get the sensor value from the 5th byte
      sprintf(iFormatedString, "%02u:%02u:%02u, ", readBuffer[7], readBuffer[6], readBuffer[5]); // Format the sensor value as date/time (HH:MM:SS)/YY:MM:DD)
      Serial_DBG.print(iFormatedString); // Print the sensor value
    }
    else{ // For other sensor IDs, combine the next four bytes to get the sensor value
      uint32_t sensorValue = (readBuffer[7] << 24) | (readBuffer[6] << 16) | (readBuffer[5] << 8) | readBuffer[4]; // Combine the next four bytes to get the sensor value
      sprintf(iFormatedString, ",%011u, ", sensorValue); // 0x0820 = altitude in cm, 0x0830 = ground speed in cm/s, 0x0840 = heading  
      Serial_DBG.print(iFormatedString); // Print the sensor value  
      }
    sprintf(iFormatedString, "%02X,  ", (uint8_t)readBuffer[8]);
    Serial_DBG.print(iFormatedString); // Print the received byte in hexadecimal format
  
  /**/ 
  } 
}
