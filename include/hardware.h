#include <HardwareSerial.h>

#ifdef TARGET_G031F8
  #define LED_BUILTIN PA4
    HardwareSerial Serial1(USART1); // PB6, PB7, TX, RX
  #ifndef Serial2
    HardwareSerial Serial2(USART2); // PA2, PA3, TX, RX 
  #endif 
  #define Serial_SP Serial1
  #define Serial_GNSS Serial2
  #define Serial_DBG NullSerial
#endif

#ifdef TARGET_G431CB
  #define LED_BUILTIN PC6
  #ifndef Serial1
    HardwareSerial Serial1(USART1); // PA9, PA10, TX, RX
  #endif
  #ifndef Serial2
    HardwareSerial Serial2(USART2); // PA2, PA3, TX, RX
  #endif
  #ifndef Serial3
    HardwareSerial Serial3(USART3); // PC10, PC11, TX, RX
  #endif
  #define Serial_SP Serial1
  #define Serial_DBG Serial
  #define Serial_GNSS Serial3
  #define HAS_USB_SERIAL 
#endif

#ifdef TARGET_WS_ESP32S3_ZERO
  #ifndef LED_BUILTIN
    #define LED_BUILTIN d3
  #endif
  #define Serial_SP_TX 6 // Pin49 = D6
  #define Serial_SP_RX 7 // Pin50 = D7
  #define Serial_GNSS_TX 2 // Pin8 = D2
  #define Serial_GNSS_RX 3 // Pin9 = D3
  #define Serial_SP   Serial0
  #define Serial_GNSS Serial1
  #define Serial_DBG  Serial 
  #define HAS_USB_SERIAL 
#endif

#ifdef TARGET_ESP32C3_SUPERMINI
  #define LED_BUILTIN 8
  #define Serial_SP_TX 21
  #define Serial_SP_RX 20 
  #define Serial_GNSS_TX 6
  #define Serial_GNSS_RX 5 
  #define Serial_SP   Serial0 // ESP32C3 has only 2 hardware serials
  #define Serial_GNSS Serial1 // ESP32C3 has only 2 hardware serials
  #define Serial_DBG  Serial 
  #define HAS_USB_SERIAL 
#endif