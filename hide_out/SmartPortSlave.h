#pragma once

#include <Arduino.h>

#define SP_START 0x07e // Start character for SmartPort protocol
#define SP_ESCAPE 0x07d // Escape character for SmartPort protocol 
#define VALID_SENSOR_PACKAGE 0x10 // Valid sensor data indicator for SmartPort protocol
#define SP_BAUD_RATE 57600 // Baud rate for SmartPort protocol
#define VALID_DATA_INTERVAL_MS 1 // Interval for valid data updates
#define SIZE_RX_BUFFER 9 // Size of the receive buffer for SmartPort data packets
#define SIZE_TX_PACKET 8 // Size of the transmit buffer for SmartPort data packets
#define degree_in_minutes_div10k 600000 // Factor to convert degree to minute x 10k for GPS data encoding in SmartPort protocol

class SmartPortSlave {
  public:
    SmartPortSlave(); // Constructor
    ~SmartPortSlave(); // Destructor

    void begin(HardwareSerial *smartPortSerial, uint8_t mySP_ID, HardwareSerial *debugSerial); // Initialize the SmartPort slave with the provided serial streams; 
    void setMySP_ID(uint8_t id) ;
    void update(void) ;
    bool sniffedDataAvailable() ;
    void getSniffedData(char* rx_buffer) ; // Get the sniffed data and store it in the provided buffer 9 bytes long
    bool txBufferSent() ;
    void sendSensorWord(uint16_t sensorID, uint32_t sensorValue) ;
    void sendV_A3(int32_t v_a3_value, int16_t sensorID) ;
    void sendLON(int32_t lon_min_X10k, int16_t sensorID) ;
    void sendLAT(int32_t lat_min_X10k, int16_t sensorID) ;
    void sendDate(uint8_t year, uint8_t month, uint8_t day, int16_t sensorID) ;
    void sendTime(uint8_t hour, uint8_t minute, uint8_t second, int16_t sensorID) ;
    void sendCellVoltage(uint32_t voltage_mV, uint8_t cellNr, int16_t sensorID) ;
    void sendAltitude(int32_t altitude_m, int16_t sensorID) ;
    void sendHeading(uint16_t heading_centiDeg, int16_t sensorID) ;
    void sendGnssAltitude(int32_t altitude_cm, int16_t sensorID) ;
    void sendGnssSpeed(int32_t speed_mm_s, int16_t sensorID) ;  
    
    private:
    bool m_sensorTxPacketReady;
    void clearReadBuffer();
    void outputSmartPortChar(uint8_t character);
    char getInputSmartPortChar();
    void setCRC(void);
    void receiveSmartPortChar(void);
    void answer_SmartPort(void);
    void sendMyNextSmartPortData(bool validData);
    void setSensorValue(uint8_t start, uint8_t length, uint32_t sensorValue);
    void setSensorID(uint16_t sensorID) ;


    HardwareSerial *m_debugSerial; // Pointer to the debug serial stream
    HardwareSerial *m_smartPortSerial; // Pointer to the SmartPort serial stream
    uint32_t m_LastTimeNow; 
    uint32_t m_sensorPacketNr;
    char *m_rxBuffer; // Buffer for received SmartPort data  m_rxBuffer[0] = Sensor_ID polled by the SP master
    char *m_rxBuf2; // Buffer for received SmartPort data  m_rxBuffer[0] = Sensor_ID polled by the SP master
    char *m_txPacket;   // Buffer for transmitting SmartPort data
    char *m_emptyTxPacket; // Buffer for empty TX packet
    uint8_t m_mySP_ID; // Sensor ID for this SmartPort slave device
    uint8_t m_txPacketIndex; // Index for the transmit buffer
    uint8_t m_readBufIndex; // Index for the read buffer
    uint32_t guard;
    uint8_t m_TxMaxAvail; // Result of empty UART TX buffer used for non-blocking "HardwareSerial.flush" check
    uint8_t m_availableTX; // Variable to store the available space in the TX buffer
    bool m_TxMode; // Flag to indicate if I am in transmit mode to be set moving bytes to the m_SmartPortSerial to be reset when Sm_TX is idle again
    bool m_EscapeNext; // Flag to indicate if the next character is escaped
    bool m_sniffedDataReady; // Flag to indicate if sniffed data is ready to be read
    char iFormatedString[100]; // Buffer for formatted strings for debugging output
};