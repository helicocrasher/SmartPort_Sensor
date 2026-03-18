#include <Arduino.h>
//#include <HardwareSerial.h>
#include <stdint.h> 
#include <Stream.h> 
#include "hardware.h"
//#include <Adafruit_INA3221.h>
//#include <Wire.h>


//#define HAL_UART_MODULE_ENABLED
#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION < 0x02110000)
//    #error "This sketch requires STM32 core version 2.11.0 or higher"
#endif
#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION == 0x02100100)
    #error "STM32 core version is 2.10.1"
#endif

//#define mySP_ID 0xa1 // Sensor ID for SmartPort protocol incl Bit CRC ID=01
//#define mySP_ID 0xb7 // Sensor ID for SmartPort protocol incl Bit CRC ID=23
#define mySP_ID 0x6a // Sensor ID for SmartPort protocol incl Bit CRC ID=10
#define SP_START 0x07e // Start character for SmartPort protocol
#define SP_ESCAPE 0x07d // Escape character for SmartPort protocol
#define VALID_SENSOR_PACKAGE 0x10 // Valid sensor data indicator for SmartPort protocol
#define SENSOR_ID 0x0900 // Sensor ID for SmartPort protocol (Voltage sensor)
#define VALID_DATA_INTERVAL_MS 100 // Interval for valid data updates


void read_SmartPort(void);
bool check_am_I_addressed();
void sendMyNextSmartPortData(bool validData);
void outputSmartPortData(uint8_t character);
void printSmartPortData();
void clearReadBuffer();
void setSensorID(uint16_t sensorID);
void setSensorValue(uint32_t sensorValue);
void setCRC(void);
void prepare_SensorPacket(uint32_t packetNr);
void setSensorValue(uint8_t start, uint8_t length, uint32_t sensorValue);

uint32_t i=0, sensorPacketNr=0;
static char txPacket[8] = {VALID_SENSOR_PACKAGE , 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Buffer for sending SmartPort data
static char emptyPacket[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff}; // empty packet as no update

char  iFormatedString[8];
uint8_t readBufINdex = 0; // Index for the read buffer
static uint32_t *longData = 0; // Pointer for long data
uint32_t timeNow=0, LastTimeNow=0;


// put function declarations here:
//int myFunction(int, int);

void setup() {
// put your setup code here, to run once:
// int result = myFunction(2, 3);
  pinMode(LED_BUILTIN, OUTPUT);

//  Serial1.setTx(PB6);
//  Serial1.setRx(PB7);
  Serial_DBG.setTx(PC10);
  Serial_DBG.setRx(PC11);
  Serial_DBG.begin(115200);      // Hardware Serial_DBG for debugging output
  delay(200);
  Serial_DBG.println("SmartPort Sensor Hub starting up");
  Serial_SP.setHalfDuplex();  // Set Serial_SP to half-duplex mode
  Serial_SP.setTxInvert();    // Invert TX pin for Serial_SP
  Serial_SP.setRxInvert();    // Invert RX pin for Serial_SP
  Serial_SP.begin(57600);     // Hardware Serial_SP Smartport input
  delay(100);                 // Wait for Serial to be ready
  while(Serial_SP.available(  ) > 0) { // Clear any remaining data in the buffer
    Serial_SP.read();         // Clear any remaining data in the buffer
    Serial_DBG.print("r");
  }
  delay(10);
  setSensorID(SENSOR_ID); // Set the Sensor ID in the txPacket buffer;
  LastTimeNow=millis(); // 
} 

char c=0;

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, (i & 0x040000));
  i+=1;
  uint8_t avail = Serial_SP.available();
  if (avail > 0) {        // Check if data is available on Serial_SP
    c = Serial_SP.read(); // Read the data from Serial_SP
    if (c == SP_START) {  // Check if the first byte matches the start byte
//      Serial_DBG.print("SmartPort polled Address : ");
      if (check_am_I_addressed()) { // Check if I am addressd on SmartPort
        if (millis() - LastTimeNow < VALID_DATA_INTERVAL_MS) {
          sendMyNextSmartPortData(false);    // Send empty SmartPort data packet
        }
        else{
          LastTimeNow = millis();
          prepare_SensorPacket(sensorPacketNr);
          sendMyNextSmartPortData(true);     // Send the next SmartPort data packet
          sensorPacketNr++;
        }
      }
    }
    else {
//    Serial_DBG.println(c, HEX );
    }
  }
  else {
//    Serial_DBG.println("no char  "); 
  }
//  delay(8); // Add a small delay to avoid flooding the Serial port
//  Serial_DBG.print(".");
}


void prepare_SensorPacket(uint32_t packetNr) {
  // Prepare the SmartPort data packet with sensor value based on packet number
  uint32_t sensorValue = 0;
  switch (packetNr % 9) {
    case 0:
      setSensorID(0x0900); // A3 sensor ID
      setSensorValue (0, 4,  i);
      break;
    case 1:
      setSensorID(0x0300); // Cells sensor ID
      setSensorValue (0, 01,  0x00); // Subadress 0 = Cell1 voltage
      setSensorValue (1, 3,  0x0760); // 3 Byte cell voltage
      break;
    case 2:
      setSensorID(0x0300); // Cells sensor ID
      setSensorValue (0, 1,  0x01); // Subadress 1 = Cell2 voltage
      setSensorValue (1, 3,  0x0780); // 3 Byte cell voltage
      break;
    case 3:
      setSensorID(0x0300); // Cells sensor ID
      setSensorValue (0, 1,  0x02); // Subadress 2 = Cell3 voltage
      setSensorValue (1, 3,  0x07a0); // 3 Byte cell voltage
      break;
    case 4:
      setSensorID(0x0300); // Cells sensor ID
      setSensorValue (0, 1,  0x03); // Subadress 3 = Cell4 voltage
      setSensorValue (1, 3,  0x07c0); // 3 Byte cell voltage
      break;
    case 5:
      setSensorID(0x0300); // Cells sensor ID
      setSensorValue (0, 1,  0x04); // Subadress 4 = Cell5 voltage
      setSensorValue (1, 3,  0x07e0); // 3 Byte cell voltage
      break;
    case 6:
      setSensorID(0x0300); // Cells sensor ID
      setSensorValue (0, 1,  0x05); // Subadress 5 = Cell6 voltage
      setSensorValue (1, 3,  0x0800); // 3 Byte cell voltage
      break;
    case 7:
      setSensorID(0x0300); // Cells sensor ID
      setSensorValue (0, 1,  0x06); // Subadress 6 = Cell7 voltage
      setSensorValue (1, 3,  0x0820); // 3 Byte cell voltage
      break;
    case 8:
      setSensorID(0x0300); // Cells sensor ID
      setSensorValue (0, 1,  0x07); // Subadress 7 = Cell8 voltage
      setSensorValue (1, 3,  0x0FFF); // 3 Byte cell voltage Max value for 12 bit cell voltage = 8.19V
      break;
    default:
      break;
  }

}

bool check_am_I_addressed() {
      while (Serial_SP.available()<1) {
        delay(1); // Wait until data is available
      } 
      c = Serial_SP.read();
//      Serial_DBG.print(c,HEX);
      if (c == mySP_ID) { // Check if the first byte matches the sensor ID
//        Serial_DBG.println(" thats us!"); // Print a message if the sensor is addressed
        return true; // Return true if polled
      }
      else {
//        Serial_DBG.println(" not us!"); // Print a message if the sensor is not addressed
        return false; // Return false if not adressed
      }
}

void sendMyNextSmartPortData(bool validData) {
  // Send the next SmartPort data package
  if (validData) {
    setCRC(); // Calculate and set the CRC in the txPacket buffer
  sprintf(iFormatedString, "%02X : ", (uint8_t)mySP_ID);
  Serial_DBG.print(iFormatedString); // Print used Sensor ID 
  }
  for (uint8_t i = 0; i < sizeof txPacket; i++) {                 // Send the complete SmartPort data package
    if (validData) {
      outputSmartPortData(txPacket[i]); // Send the data from the txPackage buffer
      sprintf(iFormatedString, "%02X ", (uint8_t)txPacket[i]);
      Serial_DBG.print(iFormatedString); // Print sensor byte
    }
    else {
      outputSmartPortData(emptyPacket[i]); // Send empty data package
//      sprintf(iFormatedString, "%02X ", (uint8_t)emptyPacket[i]);
//      Serial_DBG.print(iFormatedString); // Print empty byte
    }
//    Serial1.print( txPacket[i], HEX );
//    Serial1.print(" ");
  }
  if (validData) {
    Serial_DBG.println();
  }
  Serial_SP.flush();              // wait until the TX buffer is empty
  Serial_SP.enableHalfDuplexRx(); // Reenable Serial_SP reception - without this USART reception is disabled by Serial_SP.write() in half-duplex mode
}


void setSensorID(uint16_t sensorID) {
  // Set the sensor ID in the txPacket buffer - low byte first
  txPacket[1] = (sensorID & 0xFF);  // Set the low byte of the sensor ID
  sensorID >>= 8; // Shift the sensor ID to the right by 8 bits
  txPacket[2] = (sensorID & 0xFF);  // Set the high byte of the sensor ID
}


void setSensorValue(uint8_t start, uint8_t length, uint32_t sensorValue) {
  // Set the sensor value in the txPacket buffer - low byte first
  for (uint8_t i = 3+start; i < 3+start+length; i++) {
    txPacket[ i] = (sensorValue & 0xFF);    // Set each byte of the sensor value
    sensorValue >>= 8;                      // Shift the sensor value to the right by 8 bits
  }
}

void setCRC(void) {
  // Calculate & Set the CRC in the txPacket buffer 
  uint16_t checkShort = 0x0; // Initialize checksum variable
  for (uint8_t i = 0; i < sizeof(txPacket)-1; i++) {
    checkShort = checkShort - txPacket[i];                        // Calculate the checksum
  }
  uint8_t checkByte= (checkShort + (checkShort / 0x100)) & 0xFF;  // finalize the Checkbyte - add the high byte to the low byte and mask to get the low byte
  txPacket[sizeof(txPacket)-1] = checkByte;                       // Set the last byte of the package to the checksum
}

void outputSmartPortData(uint8_t character) {
  // Output the SmartPort data to Serial_SP and insert Escape characters if needed
  if ((character == SP_START)||(character == SP_ESCAPE)) { // If transmit data == "start of frame character" or "Escape character"
    Serial_SP.write(SP_ESCAPE);                            // insert escape character
    Serial_SP.write(character & 0x5F);                     // + send modified character
//    Serial_DBG.println("Escape character sent: ");
  } 
  else {
    Serial_SP.write(character);                            // Send the character as is
  }
}
