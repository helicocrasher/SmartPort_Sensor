#include <Arduino.h>

#include <stdint.h> 
#include <Stream.h> 
#include "hardware.h"
#include "SmartPortSlave.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h" 
#include "NullSerial.h"
#include "telemetry_config.h"
//#include <Adafruit_INA3221.h>
//#include <Wire.h>



#define SENSOR_ID 0x1b // Sensor ID for this SmartPort slave device (can be changed to a different value if needed, but should be unique among all devices on the same SmartPort bus)

void printSmartPortData();
void telemetryCarousel();
void sendInavGnssPrecision(SFE_UBLOX_GNSS& gnss, int16_t sensorID);
bool autoBaudGNSS(int rxPin, int txPin);

SFE_UBLOX_GNSS myGNSS;
SmartPortSlave mySmartPortSlave; // Create an instance of the SmartPortSlave class

void setup() {
  
  pinMode(LED_BUILTIN, OUTPUT);
#ifdef ARDUINO_ARCH_STM32
  Serial_GNSS.setTx(Serial_GNSS_TX);
  Serial_GNSS.setRx(Serial_GNSS_RX);
  Serial_SP.setTx(Serial_SP_TX);
  Serial_SP.setRx(Serial_SP_RX);
#endif
  Serial_DBG.begin(115200); 
#ifdef HAS_USB_SERIAL
  int startTime = millis();
  while (!Serial_DBG && ((millis() - startTime) < 10000)){
    ; // Wait for the debug serial port to be ready
  }
  delay(3000);
#endif
  Serial_DBG.println("SmartPort Sensor Hub starting up");
  delay(10);
//  mySmartPortSlave.begin(&Serial_SP, 0x6a, &Serial_DBG); // Initialize the SmartPort slave with the SmartPort serial stream, sensor ID and debug serial stream
#ifdef ARDUINO_ARCH_ESP32
  mySmartPortSlave.begin(&Serial_SP, SENSOR_ID, nullptr, Serial_SP_RX, Serial_SP_TX, true); // Initialize SmartPort with explicit pins for ESP32
//  Serial_GNSS.begin(115200, SERIAL_8N1, Serial_GNSS_RX, Serial_GNSS_TX);
#else  
  mySmartPortSlave.begin(&Serial_SP, SENSOR_ID, nullptr); // Initialize the SmartPort slave with the SmartPort serial stream, sensor ID and debug serial stream
//  Serial_GNSS.begin(115200); // Initialize the GNSS serial stream for communication with the u-blox GNSS module
#endif  
  
  delay(100);
  
  if (autoBaudGNSS(Serial_GNSS_RX, Serial_GNSS_TX) == false) { // Initialize the u-blox GNSS module and check if it was successful
    Serial_DBG.println("u-blox GNSS initialization failed");
    while (1); // If initialization failed, enter an infinite loop to halt the program
  }
  Serial_DBG.println("u-blox GNSS initialization successful");
  myGNSS.saveConfiguration();         // Save the selected baudrate of 115200 permanently for faster power ups in future
  myGNSS.setMeasurementRate(250);     // Set the GNSS module to xxx ms
  myGNSS.setAutoPVT(true);            // Enable automatic PVT data messages
  myGNSS.setAutoDOP(true);            // Enable automatic DOP and Sat data messages
  myGNSS.setUART1Output(COM_TYPE_UBX);
#ifdef HAS_USB_SERIAL
  Serial_DBG.println("GNSS module configured and ready");  
//  myGNSS.enableDebugging();           // Uncomment this line to enable helpful debug messages on Serial
#endif

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
  } 
  i+=1;
  telemetryCarousel(); // Update the telemetry carousel to send different sensor values in a rotating manner
  myGNSS.getPVT();
  myGNSS.getDOP();
  mySmartPortSlave.update(); // Update the SmartPort slave to handle incoming data and send responses 
  if (mySmartPortSlave.sniffedDataAvailable()) { 
    mySmartPortSlave.getSniffedData(readBuffer); // Get the sniffed data and store it in the provided buffer
//    Serial_DBG.println("Sniffed data received:"); 
//    printSmartPortData(); // Print the sniffed data in a formatted way
  }
//  delay(1); // Add a small delay to avoid overwhelming the CPU
}


void telemetryCarousel() {
static uint8_t carouselIndex = 0;
  static uint32_t lastUpdateTime = 0, timeSinceLastUpdate;
  timeSinceLastUpdate = millis() - lastUpdateTime;
  if (timeSinceLastUpdate <50) return;
  if (!mySmartPortSlave.txBufferEmpty()) return; 
  
  uint8_t enabledCount = 0;

#ifdef ENABLE_SENSOR_V_A3
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendV_A3(i, 0x0901);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_CELL_VOLTAGE_0
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendCellVoltage(3700/2, 0, 0x00);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_CELL_VOLTAGE_1
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendCellVoltage(3800/2, 1, 0x00);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_CELL_VOLTAGE_2
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendCellVoltage(3900/2, 2, 0x00);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_CELL_VOLTAGE_3
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendCellVoltage(4000/2, 3, 0x00);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_CELL_VOLTAGE_4
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendCellVoltage(4100/2, 4, 0x00);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_CELL_VOLTAGE_5
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendCellVoltage(4200/2, 5, 0x00);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_LON
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendLON(((myGNSS.getLongitude()*3)/50), 0x00);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_LAT
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendLAT(((myGNSS.getLatitude()*3)/50), 0x00);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_DATE
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendDate(myGNSS.getYear()-208, myGNSS.getMonth(), myGNSS.getDay(), 0x00);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_TIME
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendTime(myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(), 0x00);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_INAV_GNSS_PRECISION
  if (carouselIndex == enabledCount) {
    sendInavGnssPrecision(myGNSS, 0x00);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_SATS_IN_VIEW
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendGnssSats(myGNSS.getSIV(), 0x01);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_GNSS_HEADING
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendHeading(myGNSS.getHeading(), 0x00);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_GNSS_ALTITUDE
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendGnssAltitude(myGNSS.getAltitudeMSL()/10, 0x00);
  }
  enabledCount++;
#endif
#ifdef ENABLE_SENSOR_GNSS_SPEED
  if (carouselIndex == enabledCount) {
    mySmartPortSlave.sendGnssSpeed(myGNSS.getGroundSpeed(), 0x00);
  }
  enabledCount++;
#endif

  if (enabledCount == 0) {
    lastUpdateTime = millis();
    return;
  }

  carouselIndex = (carouselIndex + 1) % enabledCount; // Move to the next enabled sensor slot
//  Serial_DBG.print(" ");
//  Serial_DBG.print(carouselIndex);
  lastUpdateTime = millis();
}

void sendInavGnssPrecision(SFE_UBLOX_GNSS& gnss, int16_t sensorID) {
  sensorID = 0x0480 |(sensorID & 0x000f); // GNSS Satellites in view sensor ID used by iNav for the number of satellites in view
#ifdef HAS_USB_SERIAL
  Serial_DBG.print(" Satellites in view: ");
  Serial_DBG.print(gnss.getSIV());
  Serial_DBG.print(" HDOP: ");
  Serial_DBG.print(((float)gnss.getHorizontalDOP())/100);
  Serial_DBG.print(" VDOP: ");
  Serial_DBG.print(((float)gnss.getVerticalDOP())/100);
#endif
  int32_t gnssLockPrecisionSats= gnss.getSIV();
  //int32_t iNavPPrecison =1000/((float)gnss.getHorizontalDOP());
  int32_t iNavPPrecison =1000/(gnss.getHorizontalDOP()); // no float to reduce flash usage.
  
  if (iNavPPrecison > 9) iNavPPrecison = 9; // Limit the iNav precision value to 9 for encoding in a single decimal digit
  gnssLockPrecisionSats = gnssLockPrecisionSats + 100*iNavPPrecison; // Combine the number of satellites in view with the iNav precision value for encoding in a single byte (0-3 satellites = poor lock, 4-6 = moderate lock, 7-9 = good lock, with the iNav precision value providing additional granularity within those categories) ;
  if (gnssLockPrecisionSats > 3) gnssLockPrecisionSats = gnssLockPrecisionSats + 3000; // Encode the number of satellites in view with an offset to distinguish it from the case of 0-3 satellites which can be used to indicate a poor GNSS lock
  mySmartPortSlave.sendSensorWord(gnssLockPrecisionSats, (uint16_t)sensorID); // Send HDOP as a floating point value in the sensor value field
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


bool autoBaudGNSS(int rxPin, int txPin) {
  // Attempt to connect to the GNSS module at different baud rates until successful
  const uint32_t possibleBauds[] = {115200, 9600, 19200, 38400, 57600}; // List of possible baud rates to try
  uint8_t BaudRateIndex = 0;

  Serial_DBG.print(("Attempting to connect to GNSS module with "));
  Serial_DBG.println(possibleBauds[BaudRateIndex]);
  #ifdef ARDUINO_ARCH_ESP32
    Serial_GNSS.begin(possibleBauds[BaudRateIndex], SERIAL_8N1, rxPin, txPin);
  #else
    Serial_GNSS.begin(possibleBauds[BaudRateIndex]);
  #endif

  while ((myGNSS.begin(Serial_GNSS) == false)) //Connect to the u-blox module using gnssSerial (defined above)
  {
    delay (5);
    BaudRateIndex++;
    if (BaudRateIndex >= sizeof(possibleBauds)) {
      Serial_DBG.println(("Failed to connect to GNSS module at all tested baud rates."));
      return false; // If we've tried all baud rates and failed, return false
    }  
    Serial_DBG.print(("Attempting to connect to GNSS module with "));
    Serial_DBG.println(possibleBauds[BaudRateIndex]);
    #ifdef ARDUINO_ARCH_ESP32
      Serial_GNSS.begin(possibleBauds[BaudRateIndex], SERIAL_8N1, rxPin, txPin);
    #else
      Serial_GNSS.begin(possibleBauds[BaudRateIndex]);
    #endif
  }
  Serial_DBG.print(("Connected to GNSS module at "));
  Serial_DBG.print(possibleBauds[BaudRateIndex]);
  Serial_DBG.println((" baud."));
  if (BaudRateIndex != 0){
    BaudRateIndex = 0; // We want to set back to 115200
    myGNSS.setSerialRate(possibleBauds[BaudRateIndex]); // Set u-blox module to 115200 baud for reconnect
    Serial_GNSS.begin(possibleBauds[BaudRateIndex]);
    Serial_DBG.print(("Reconnected to GNSS module at "));
    Serial_DBG.print(possibleBauds[BaudRateIndex]);
    Serial_DBG.println((" baud."));
  }
  return true;  
}
