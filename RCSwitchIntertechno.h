#ifndef _RCSwitchIntertechno_h
#define _RCSwitchIntertechno_h

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#else
    #include <wiringPi.h>
    #include <stdint.h>
    #include <stddef.h>
    #define CHANGE 1
#ifdef __cplusplus
extern "C"{
#endif
typedef uint8_t boolean;
typedef uint8_t byte;

#if !defined(NULL)
#endif
#ifdef __cplusplus
}
#endif
#endif


// Number of maximum High/Low changes per packet.
// We can handle up to (unsigned long) => long low pulse + 2 (preamble) + 4 * (2 bit intro + 24 bit learncode + 1 bit low + on/off bit + 4 device index bits) + 1
#define RCSwitchIntertechno_MAX_CHANGES 1+2+4*(2+24+1+1+4)+1

class RCSwitchIntertechno {

  public:
    RCSwitchIntertechno();
    
    void enableReceive(int interrupt);
    void enableReceive();
    void disableReceive();
    bool available();
    void resetAvailable();
  
    unsigned long getReceivedValue();
    unsigned int getReceivedBitlength();
    unsigned int getReceivedDelay();
    unsigned int getReceivedProtocol();
    unsigned int* getReceivedRawdata();
  
    void setReceiveTolerance(int nPercent);

  private:
    static void handleInterrupt();
    static bool receiveIntertechnoProtocol3(unsigned int changeCount);
    int nReceiverInterrupt;

    static int nReceiveTolerance;
    volatile static unsigned long nReceivedValue;
    volatile static unsigned int nReceivedBitlength;
    volatile static unsigned int nReceivedDelay;
    volatile static unsigned int nReceivedProtocol;
    static unsigned int timings[RCSwitchIntertechno_MAX_CHANGES];
};

#endif
