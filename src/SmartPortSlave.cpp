#include "SmartPortSlave.h"

SmartPortSlave::SmartPortSlave() 
  // Constructor - can be used to initialize any member variables if needed
  : m_mySP_ID(0), m_readBufIndex(0), m_availableTX(0), m_TxMode(false), m_EscapeNext(false), m_sniffedDataReady(false) {}
  
SmartPortSlave::~SmartPortSlave() { 
  // Destructor - can be used to clean up resources if needed
  if (m_rxBuffer)     delete[] m_rxBuffer; // Free the memory allocated for the receive buffer
  if (m_rxBuf2)       delete[] m_rxBuf2;   // Free the memory allocated for the second receive buffer
  if (m_txPacket)     delete[] m_txPacket; // Free the memory allocated for the transmit buffer
  if (m_emptyTxPacket) delete[] m_emptyTxPacket; // Free the memory allocated for the empty transmit buffer
}

void SmartPortSlave::begin(HardwareSerial *smartPortSerial, uint8_t mySP_ID=0x6a, HardwareSerial *debugSerial=&Serial2) {
  m_smartPortSerial = smartPortSerial;
  m_debugSerial = debugSerial;
  m_mySP_ID = mySP_ID;
  m_rxBuffer = new char[SIZE_RX_BUFFER]; // Allocate memory for the receive buffer
  m_rxBuf2 = new char[SIZE_RX_BUFFER];   // Allocate memory for the second receive buffer
  m_txPacket = new char[SIZE_TX_PACKET]; // Allocate memory for the transmit buffer
  m_emptyTxPacket = new char[SIZE_TX_PACKET]; // Allocate memory for the empty transmit buffer
  memset(m_emptyTxPacket, 0, SIZE_TX_PACKET); // Initialize the empty transmit buffer with zeros
  m_smartPortSerial->setHalfDuplex();  // Set Serial_SP to half-duplex mode
  m_smartPortSerial->setTxInvert();    // Invert TX pin for Serial_SP
  m_smartPortSerial->setRxInvert();    // Invert RX pin for Serial_SP
  m_smartPortSerial->begin(SP_BAUD_RATE);     // Start the serial communication for SmartPort protocol at SmartPort baud rate
  m_smartPortSerial->enableHalfDuplexRx(); // Enable reception for half-duplex mode
  while(m_smartPortSerial->available(  ) > 0) { // Clear any remaining data in the UART RX buffer
    m_smartPortSerial->read();         // Clear any remaining data in the buffer
    }
  clearReadBuffer(); // Clear the read buffer to ensure it's empty before starting communication
  m_smartPortSerial->flush();              // Wait until the TX buffer is empty
  m_TxMaxAvail = m_smartPortSerial->availableForWrite(); // Clear any remaining data in the UART TX buffer
  m_smartPortSerial->enableHalfDuplexRx(); // Enable reception for half-duplex mode
}      

void SmartPortSlave::setMySP_ID(uint8_t id) {
  m_mySP_ID = id; // Set the sensor ID for this SmartPort slave device
}  

void SmartPortSlave::update(void) {
  // This function can be used to perform any periodic updates or checks if needed  
  if (m_TxMode) {
    if(m_smartPortSerial->availableForWrite() == m_TxMaxAvail) { // Check if the TX buffer is completely sent -> ready to listen again
      m_smartPortSerial->flush();              // Make sure the TX buffer is really empty
      m_smartPortSerial->enableHalfDuplexRx(); // Reenable Serial_SP reception - without this USART reception is disabled by Serial_SP.write() in half-duplex mode  
      m_TxMode = false; // Reset the transmit mode flag
      return; // Exit the update function after the TX packet has been sent and reception is reenabled
    }
    else  return;  // If still in transmit mode, we can perform any necessary actions while waiting for the TX packet to be sent
  }
  else { // receive mode - check for incoming data
    if (m_smartPortSerial->available() > 0) { // Check if there is any data available to read from the SmartPort serial
//      m_debugSerial->print("R");
      receiveSmartPortChar(); // Handle the reception of SmartPort data if needed
    }  
    else {
      m_smartPortSerial->enableHalfDuplexRx(); // Ensure reception is enabled in case it was disabled by a previous transmission
    }
  }
}

void SmartPortSlave::receiveSmartPortChar() {
  // This function can be used to handle the reception of SmartPort data if needed
  // It can be called from the update function when data is available to read

  char c = m_smartPortSerial->read(); // Read a byte from the SmartPort serial and handle escape characters
  if (c==SP_START) { // Start of frame character received
//    m_debugSerial->println("\n\rStart"); // Print a message for debugging
    clearReadBuffer(); // Clear the read buffer before starting a new frame
    m_EscapeNext = false; // Reset the escape flag
    return; // Exit the function after handling the start of frame character
  }
  else if (c==SP_ESCAPE) { // Escape character received
    m_EscapeNext = true; // Set the flag to indicate the next character is escaped
    return; // Exit the function after handling the escape character
  }
  else if (m_EscapeNext) { // If the previous character was an escape character
        // Apply the escape sequence
        if (c == 0x5e) { // 0x5e is the escaped version of 0x7e
          c = 0x7e; // Replace with 0x7e
        } 
        else if (c == 0x5d) { // 0x5d is the escaped version of 0x7d
          c = 0x7d; // Replace with 0x7d
        }
        m_EscapeNext = false; // Reset the escape flag
  }
  if (m_readBufIndex <  SIZE_RX_BUFFER) { // Ensure we don't overflow the buffer
    m_rxBuffer[m_readBufIndex] = c; // Store the character in the buffer
    snprintf(iFormatedString, 7, "%2d,%02X",  m_readBufIndex, (uint8_t)c); // Format the received character for debugging
//    m_debugSerial->print(iFormatedString); // Print the received character in hexadecimal format for debugging
    m_readBufIndex++;
  }
   if (m_readBufIndex ==  SIZE_RX_BUFFER) { // If the buffer is full, set the flag to indicate that sniffed data is ready
    memcpy(m_rxBuf2, m_rxBuffer,  SIZE_RX_BUFFER); // Copy the received data to the second buffer for later retrieval
    m_sniffedDataReady = true; // Set the flag to indicate that sniffed data is ready to be read
  }
  else if (m_readBufIndex >  SIZE_RX_BUFFER) { // If the buffer is overflowed, print a message and clear the buffer
    m_debugSerial->println("Buffer overflow"); // Print a message if the buffer is overflowed
    clearReadBuffer(); // Clear the buffer to prevent further overflow
  }
  else if ((m_readBufIndex == 1) && ( m_rxBuffer[0] == m_mySP_ID)) { // Check if the first byte matches the sensor ID
    answer_SmartPort(); // prepare & Send the next SmartPort data package;
   clearReadBuffer(); // Clear the buffer after responding to the poll
  }
}

void SmartPortSlave::answer_SmartPort(){
  if (millis() - m_LastTimeNow < VALID_DATA_INTERVAL_MS || !m_sensorTxPacketReady) { // If the time since the last valid data packet is less than the defined interval or if it's the first packet
    sendMyNextSmartPortData(false);    // Send empty SmartPort data packet
  }
  else{
    m_LastTimeNow = millis();
    m_sensorTxPacketReady = false;
    sendMyNextSmartPortData(true);     // Send the next SmartPort data packet
    m_sensorPacketNr++;
  }
}


void SmartPortSlave::sendMyNextSmartPortData(bool validData) {
  // Send the next SmartPort data package
  if (validData) {
    m_txPacket[0] = VALID_SENSOR_PACKAGE; // Set the first byte to indicate a valid sensor package
    setCRC();       // add the CRC byte in the txPacket buffer
  }
  else {
    memcpy(m_txPacket, m_emptyTxPacket,  SIZE_TX_PACKET); // Copy the empty packet to the transmit buffer
  }
  for (uint8_t i = 0; i <  SIZE_TX_PACKET; i++) {  // transfer the complete data package to the SmartPort UART with escaping "forbidden" caracters
    outputSmartPortChar(m_txPacket[i]); // Send the data from the txPackage buffer
    snprintf(iFormatedString, 7, "%2d,%02x",  i, (uint8_t)m_txPacket[i]); 
//    m_debugSerial->print(iFormatedString); // Print the transmitted character in hexadecimal format for debugging
  }
}


void SmartPortSlave::sendV_A3(int32_t v_a3_value, int16_t sensorID) {
  setSensorID(sensorID);
  setSensorValue(0, 4, v_a3_value);
  m_sensorTxPacketReady = true; // Set the flag to indicate that a valid sensor packet is ready to be sent
}


void SmartPortSlave::setSensorID(uint16_t sensorID) {
  // Set the sensor ID in the txPacket buffer - low byte first
  m_txPacket[1] = (sensorID & 0xFF);  // Set the low byte of the sensor ID
  sensorID >>= 8; // Shift the sensor ID to the right by 8 bits
  m_txPacket[2] = (sensorID & 0xFF);  // Set the high byte of the sensor ID
}


void SmartPortSlave::setSensorValue(uint8_t start, uint8_t length, uint32_t sensorValue) {
  // Set the sensor value in the txPacket buffer - low byte first
  for (uint8_t i = 3+start; i < 3+start+length; i++) {
    m_txPacket[ i] = (sensorValue & 0xFF);    // Set each byte of the sensor value
    sensorValue >>= 8;                      // Shift the sensor value to the right by 8 bits
  }
}

void SmartPortSlave::setCRC(void) {
  // Calculate & Set the CRC in the txPacket buffer 
  uint16_t checkShort = 0x0; // Initialize checksum variable
  for (uint8_t i = 0; i < 8-1; i++) {
    checkShort = checkShort - m_txPacket[i];                        // Calculate the checksum
  }
  uint8_t checkByte= (checkShort + (checkShort / 0x100)) & 0xFF;  // finalize the Checkbyte - add the high byte to the low byte and mask to get the low byte
  m_txPacket[SIZE_TX_PACKET-1] = checkByte;                       // Set the last byte of the package to the checksum
}

void SmartPortSlave::outputSmartPortChar(uint8_t character) {
  // Output the SmartPort data to m_smartPortSerial and insert Escape characters if needed
  if ((character == SP_START)||(character == SP_ESCAPE)) { // If transmit data == "start of frame character" or "Escape character"
    m_smartPortSerial->write(SP_ESCAPE);                            // insert escape character
    m_smartPortSerial->write(character & 0x5F);                     // + send modified character
//    m_debugSerial->println("Escape character sent: ");
  } 
  else {
    m_smartPortSerial->write(character);                            // Send the character as is
  }
}

char SmartPortSlave::getInputSmartPortChar() {
  // Read a byte from m_smartPortSerial and handle escape characters
  while (m_smartPortSerial->available() < 1) ;
  char c = m_smartPortSerial->read();
  if (c == SP_ESCAPE) { // If the character is an escape character
    while (m_smartPortSerial->available() < 1) ; // Wait for the next byte to be available
    c = m_smartPortSerial->read() ^ 0x20; // Read the next byte and unescape it
  }
  return c; // Return the read character
}

void SmartPortSlave::clearReadBuffer() {
  // Clear the read buffer
  for (uint8_t i = 0; i <  SIZE_RX_BUFFER; i++) {
    m_rxBuffer[i] = 0;
  }
//  m_debugSerial->println("Buffer cleared"); // Print a message for debugging
  m_readBufIndex = 0; // Reset the index
  m_sniffedDataReady = false; // Reset the flag to indicate that there is no sniffed data ready after clearing the buffer
} 

bool SmartPortSlave::sniffedDataAvailable() {
  return m_sniffedDataReady; // Check if there is any data available to read from the SmartPort serial
}

void SmartPortSlave::getSniffedData(char* rx_buffer) {
  // Read data from m_smartPortSerial and store it in the provided buffer
  memcpy(rx_buffer, m_rxBuf2,  SIZE_RX_BUFFER); // Copy the data from the internal buffer to the provided buffer
  m_sniffedDataReady = false; // Reset the flag after reading the data
}


bool SmartPortSlave::txBufferSent() {
 return !m_TxMode ; 
}

void SmartPortSlave::sendSensorWord(uint16_t sensorID, uint32_t sensorValue) {
}

void SmartPortSlave::sendLON(int32_t lon_min_X10k, int16_t sensorID) {

  lon_min_X10k = ((lon_min_X10k % (360*degree_in_minutes_div10k)) + (360 * degree_in_minutes_div10k)) % (360 * degree_in_minutes_div10k); // 
  if (lon_min_X10k >= 180*degree_in_minutes_div10k) { // Check if the longitude value exceeds 180.0° in minute x 10k
    lon_min_X10k = -lon_min_X10k + (360 * degree_in_minutes_div10k); // Make the longitude value positive for encoding
    lon_min_X10k |= 0x1 << 30; // Set the West bit for negative longitude
  }
  sensorID = 0x0800 |(sensorID & 0x000f); // GPS LAT or LON sensor ID
  setSensorID(sensorID); // GPS LAT or LON sensor ID
  setSensorValue (0, 4,  0x1<<31 | lon_min_X10k); // 0x<<31 = Lon bit 
  m_sensorTxPacketReady = true;
}

void SmartPortSlave::sendLAT(int32_t lat_min_X10k, int16_t sensorID) {

  // Prepare the SmartPort data packet with GPS LAT sensor value
  if (lat_min_X10k < -90*degree_in_minutes_div10k) { // Check if the latitude value exceeds 90.0° in minute x 10k
    lat_min_X10k = -90*degree_in_minutes_div10k; // Limit the latitude value to -90.0° for encoding
  }
  if (lat_min_X10k > 90*degree_in_minutes_div10k) { // Check if the latitude value exceeds 90.0° in minute x 10k
    lat_min_X10k = 90*degree_in_minutes_div10k; // Limit the latitude value to 90.0° for encoding
  }
  if (lat_min_X10k < 0) { // Check if the latitude value is negative
    lat_min_X10k = -lat_min_X10k; // Make the latitude value positive for encoding
    lat_min_X10k |= 0x1 << 30; // Set the South bit for negative latitude
  }
  sensorID = 0x0800 |(sensorID & 0x000f); // GPS LAT or LON sensor ID
  setSensorID(sensorID); // GPS LAT or LON sensor ID
  setSensorValue (0, 4,  lat_min_X10k); 
  m_sensorTxPacketReady = true;
}

void SmartPortSlave::sendDate(uint8_t year, uint8_t month, uint8_t day, int16_t sensorID) {
  uint32_t dateValue = 0;
  if (month > 12) month = 12; // Limit month to 12 (4 bits)
  if (day > 31) day = 31; // Limit day to 31 (5 bits)
  dateValue |= (year) << 16;   // Year in bits 16-22
  dateValue |= (month) << 8;   // Month in bits 8-11
  dateValue |= (day );     // Day in bits 0-4
  sensorID = 0x0850 |(sensorID & 0x000f); // Date sensor ID
  setSensorID(sensorID); // Date sensor ID
  setSensorValue(0, 1, 0x01); // Subadress 1 = Date value
  setSensorValue(1,3, dateValue); // Set the encoded date value in the sensor packet
  m_sensorTxPacketReady = true;
}

void SmartPortSlave::sendTime(uint8_t hour, uint8_t minute, uint8_t second, int16_t sensorID) {
  uint32_t timeValue = 0;
  if (hour > 23) hour = 23;       // Limit hour to 23
  if (minute > 59) minute = 59;   // Limit minute to 59
  if (second > 59) second = 59;   // Limit second to 59
  // Encode the time values into a single 32-bit integer
  timeValue |= (hour ) << 16;    // Hour in bits 16-20
  timeValue |= (minute ) << 8;   // Minute in bits 8-13
  timeValue |= (second );        // Second in bits 0-5
  sensorID = 0x0850 |(sensorID & 0x000f); // Time sensor ID
  setSensorID(sensorID); // Time sensor ID
  setSensorValue(0, 1, 0x00); // Subadress 0 = Time value
  setSensorValue(1,3, timeValue); // Set the encoded time value in the sensor packet
  m_sensorTxPacketReady = true;
}

void SmartPortSlave::sendCellVoltage(uint32_t cellVoltage_mV, uint8_t cellNr, int16_t sensorID) {
  sensorID = 0x0300 |(sensorID & 0x000f); // Cells sensor ID
  setSensorID(sensorID); // Cells sensor ID
  setSensorValue (0, 01,  cellNr); // Subadress 0 = Cell1 voltage
  setSensorValue (1, 3,  cellVoltage_mV & 0x0ffffff); // 3 Byte cell voltage
  m_sensorTxPacketReady = true;
}

void SmartPortSlave::sendAltitude(int32_t altitude_cm, int16_t sensorID) {
  sensorID = 0x0100 |(sensorID & 0x000f); // Altitude sensor ID
  setSensorID(sensorID); // Altitude sensor ID
  setSensorValue (0, 4,  altitude_cm + 1230); // Altitude value in centimeters plus a silly offset to get Zero for Zero
  m_sensorTxPacketReady = true;
}

void SmartPortSlave::sendHeading(uint16_t heading_centiDeg, int16_t sensorID) {
  sensorID = 0x0840 |(sensorID & 0x000f); // Heading sensor ID
  setSensorID(sensorID); // Heading sensor ID
  heading_centiDeg = heading_centiDeg % 36000; 
  setSensorValue (0, 2,  heading_centiDeg); // Heading value in centi-degrees
  m_sensorTxPacketReady = true;
}

void SmartPortSlave::sendGnssAltitude(int32_t altitude_mm, int16_t sensorID) {
  sensorID = 0x0820 |(sensorID & 0x000f); // GNSS Altitude sensor ID
  setSensorID(sensorID); // GNSS Altitude sensor ID
  setSensorValue (0, 4,  altitude_mm); // Altitude value in millimeters
  m_sensorTxPacketReady = true;
}

 void SmartPortSlave::sendGnssSpeed(int32_t speed_milli_kts, int16_t sensorID) {
  sensorID = 0x0830 |(sensorID & 0x000f); // Ground Speed sensor ID
  setSensorID(sensorID); // Ground Speed sensor ID
  setSensorValue (0, 4,  speed_milli_kts); // Ground Speed value in millimeters per second
  m_sensorTxPacketReady = true;
 }