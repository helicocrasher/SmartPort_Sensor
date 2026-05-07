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

#endif