#ifndef ModbusRTUSlave_h
#define ModbusRTUSlave_h

#define NO_DE_PIN 255

#define FC_01_R_COILS 1
#define FC_02_R_DISCRETEINPUTS 2
#define FC_03_R_HOLDINGREGISTERS 3
#define FC_04_R_INPUTREGISTER 4
#define FC_05_W_SINGLECOIL 5
#define FC_06_W_SINGLEREGISTER 6
#define FC_15_W_MULTIPLECOILS 15
#define FC_16_W_MULTIPLEREGISTERS 16

#include "Arduino.h"

class ModbusRTUSlave {
  public:
    ModbusRTUSlave(Stream& serial,
                    uint8_t *buf, uint16_t bufSize,
                    uint8_t dePin = NO_DE_PIN,
                    uint32_t responseDelay = 0,
                    void (*txirq_enable_disable)(bool) = 0,
                    bool blocking=true,
                    uint8_t multiChannel = 0);

    typedef int8_t (*BoolRead_mc)(uint16_t, uint8_t*, uint8_t);
    typedef bool (*BoolWrite_mc)(uint16_t, bool, uint8_t);
    typedef int32_t (*WordRead_mc)(uint16_t, uint8_t*, uint8_t);
    typedef bool (*WordWrite_mc)(uint16_t, uint16_t, uint8_t);
    
    typedef int8_t (*BoolRead)(uint16_t, uint8_t*);
    typedef bool (*BoolWrite)(uint16_t, bool);
    typedef int32_t (*WordRead)(uint16_t, uint8_t*);
    typedef bool (*WordWrite)(uint16_t, uint16_t);
    
    void configureCoils(uint16_t numCoils, BoolRead coilRead, BoolWrite coilWrite);
    void configureDiscreteInputs(uint16_t numDiscreteInputs, BoolRead discreteInputRead);
    void configureHoldingRegisters(uint16_t numHoldingRegisters, WordRead holdingRegisterRead, WordWrite holdingRegisterWrite);
    void configureInputRegisters(uint16_t numInputRegisters, WordRead inputRegisterRead);

    void configureCoils(uint16_t numCoils, BoolRead_mc coilRead, BoolWrite_mc coilWrite);
    void configureDiscreteInputs(uint16_t numDiscreteInputs, BoolRead_mc discreteInputRead);
    void configureHoldingRegisters(uint16_t numHoldingRegisters, WordRead_mc holdingRegisterRead, WordWrite_mc holdingRegisterWrite);
    void configureInputRegisters(uint16_t numInputRegisters, WordRead_mc inputRegisterRead);

    /* NOTE: Be sure to always check getTransmitting() before changing settings and calling begin()!!
     *       For any nonblocking users without IRQ operation, the user must manually check the XDIR pin or otherwise
     *       manually check the serial port is done transmitting before doing something which changes the serial port's state. */
    void begin(uint8_t id, uint32_t baud, uint8_t config = 0x06);
    void poll();
    void txDone_irq(void);
    bool getTransmitting(void);
    uint8_t GetOffset() const {
      return _offset;
    }
    void setMultiChLimit(uint8_t in_lim) { 
        if (in_lim > 227) {
            in_lim = 227; // Protect reserved addresses
        }
        _limit = in_lim; 
    }
    
  private:
    const uint8_t _multiCh; // Protect reserved addresses
    uint8_t _offset;
    uint8_t _limit;
    Stream *_serial;
    uint8_t *_buf;
    uint16_t _bufSize;
    uint8_t _dePin;
    uint16_t _numCoils = 0;
    uint16_t _numDiscreteInputs = 0;
    uint16_t _numHoldingRegisters = 0;
    uint16_t _numInputRegisters = 0;
    uint16_t _bufpos = 0;
    uint16_t _writesize = 0;
    bool _blocking;
    bool _transmitting;
    BoolRead _coilRead;
    BoolRead _discreteInputRead;
    WordRead _holdingRegisterRead;
    WordRead _inputRegisterRead;
    BoolWrite _coilWrite;
    WordWrite _holdingRegisterWrite;
    BoolRead_mc _coilRead_mc;
    BoolRead_mc _discreteInputRead_mc;
    WordRead_mc _holdingRegisterRead_mc;
    WordRead_mc _inputRegisterRead_mc;
    BoolWrite_mc _coilWrite_mc;
    WordWrite_mc _holdingRegisterWrite_mc;
    uint8_t _id;
    uint32_t _charTimeout;
    uint32_t _frameTimeout;
    uint32_t _responseDelay;
    uint32_t _rxStartTime;
    void (*_txirq_enable_disable)(bool);
    void _processBoolRead(uint16_t numBools, BoolRead boolRead);
    void _processWordRead(uint16_t numWords, WordRead wordRead);
    void _processBoolRead(uint16_t numBools, BoolRead_mc boolRead);
    void _processWordRead(uint16_t numWords, WordRead_mc wordRead);
    void _exceptionResponse(uint8_t code);
    void _write(uint8_t len);
    uint16_t _crc(uint8_t len);
    uint16_t _div8RndUp(uint16_t value);
    uint16_t _bytesToWord(uint8_t high, uint8_t low);
};

#endif
