#include <HardwareSerial.h>

#ifdef TARGET_BLUEPILL
  #define LED_BUILTIN PB2
  HardwareSerial Serial1(USART1); // PA2, PA10, TX, RX
  #ifndef Serial2
  //    HardwareSerial Serial2(USART2); // Smartport input TX, RX
  #endif
  //  HardwareSerial Serial3(USART3); // PB10, PB11 TX, RX
#endif

#ifdef TARGET_G031F8
  #define LED_BUILTIN PA4
  HardwareSerial Serial1(USART1); // Pb3, PB7, TX, RX
  #ifndef Serial2
    HardwareSerial Serial2(USART2); // Smartport input PA2, PA3 TX, RX 
  #endif 
  #define Serial_SP Serial2
#endif

#ifdef TARGET_F303K8
  #define LED_BUILTIN PA4
  HardwareSerial Serial1(USART1); // Pb3, PB7, TX, RX
  #ifndef Serial2
 //   HardwareSerial Serial2(USART2); // Smartport input PA2, PA3 TX, RX 
  #endif 
 // HardwareSerial Serial2(USART2);
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
  #define Serial_DBG Serial3

#endif