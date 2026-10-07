#include "ModbusRTUSlave.h"

ModbusRTUSlave::ModbusRTUSlave(Stream& serial,
								uint8_t *buf,
								uint16_t bufSize,
								uint8_t dePin,
								uint32_t responseDelay,
								void (*txirq_enable_disable)(bool),
								bool blocking,
								uint8_t multiCh) : _MULTI_CHANNEL(multiCh) {
	_serial = &serial;
	_offset = 0;
	_mcLimit = 50;
	_buf = buf;
	_bufSize = bufSize;
	_dePin = dePin;
	_responseDelay = responseDelay;
	_txirq_enable_disable = txirq_enable_disable;
	_blocking = blocking;
	_transmitting = false;
}


void ModbusRTUSlave::configureCoils(uint16_t numCoils, BoolRead coilRead, BoolWrite coilWrite) {
	if(_MULTI_CHANNEL > 0) {
		Serial2.println(F("ERROR: Multi-channel does not support this function, use configureCoils_mc() instead."));
		return; // multi-channel not supported
	}
	
	_numCoils = numCoils;
	_coilRead = coilRead;
	_coilWrite = coilWrite;
}

void ModbusRTUSlave::configureDiscreteInputs(uint16_t numDiscreteInputs, BoolRead discreteInputRead) {
	if(_MULTI_CHANNEL > 0) {
		Serial2.println(F("ERROR: Multi-channel does not support this function, use configureDiscreteInputs_mc() instead."));
		return; // multi-channel not supported
	}

	_numDiscreteInputs = numDiscreteInputs;
	_discreteInputRead = discreteInputRead;
}

void ModbusRTUSlave::configureHoldingRegisters(uint16_t numHoldingRegisters, WordRead holdingRegisterRead, WordWrite holdingRegisterWrite) {
	if(_MULTI_CHANNEL > 0) {
		Serial2.println(F("ERROR: Multi-channel does not support this function, configureHoldingRegisters_mc() instead."));
		return; // multi-channel not supported
	}

	_numHoldingRegisters = numHoldingRegisters;
	_holdingRegisterRead = holdingRegisterRead;
	_holdingRegisterWrite = holdingRegisterWrite;
}

void ModbusRTUSlave::configureInputRegisters(uint16_t numInputRegisters, WordRead inputRegisterRead){
	if(_MULTI_CHANNEL > 0) {
		Serial2.println(F("ERROR: Multi-channel does not support this function, configureInputRegisters_mc() instead."));
		return; // multi-channel not supported
	}

	_numInputRegisters = numInputRegisters;
	_inputRegisterRead = inputRegisterRead;
}

void ModbusRTUSlave::configureCoils_mc(uint16_t numCoils, BoolRead_mc coilRead, BoolWrite_mc coilWrite) {
	if(_MULTI_CHANNEL == 0) {
		Serial2.println(F("ERROR: Do not use this function without multi-channel!"));
		return; // multi-channel not supported
	}

	_numCoils = numCoils;
	_coilRead_mc = coilRead;
	_coilWrite_mc = coilWrite;
	
}

void ModbusRTUSlave::configureDiscreteInputs_mc(uint16_t numDiscreteInputs, BoolRead_mc discreteInputRead) {
	if(_MULTI_CHANNEL == 0) {
		Serial2.println(F("ERROR: Do not use this function without multi-channel!"));
		return; // multi-channel not supported
	}

	_numDiscreteInputs = numDiscreteInputs;
	_discreteInputRead_mc = discreteInputRead;
}

void ModbusRTUSlave::configureHoldingRegisters_mc(uint16_t numHoldingRegisters, WordRead_mc holdingRegisterRead, WordWrite_mc holdingRegisterWrite) {
	if(_MULTI_CHANNEL == 0) {
		Serial2.println(F("ERROR: Do not use this function without multi-channel!"));
		return; // multi-channel not supported
	}

	_numHoldingRegisters = numHoldingRegisters;
	_holdingRegisterRead_mc = holdingRegisterRead;
	_holdingRegisterWrite_mc = holdingRegisterWrite;
}

void ModbusRTUSlave::configureInputRegisters_mc(uint16_t numInputRegisters, WordRead_mc inputRegisterRead) {
	if(_MULTI_CHANNEL == 0) {
		Serial2.println(F("ERROR: Do not use this function without multi-channel!"));
		return; // multi-channel not supported
	}

	_numInputRegisters = numInputRegisters;
	_inputRegisterRead_mc = inputRegisterRead;
}


void ModbusRTUSlave::begin(uint8_t id, uint32_t baud, uint8_t config) {
	_id = id;
	uint32_t bitsPerChar;
	_rxStartTime = micros();

	if (config == SERIAL_8E2 || config == SERIAL_8O2) bitsPerChar = 12;
	else if (config == SERIAL_8N2 || config == SERIAL_8E1 || config == SERIAL_8O1) bitsPerChar = 11;
	else bitsPerChar = 10;
	if (baud <= 19200) {
		_charTimeout = (bitsPerChar * 1500300) / baud;
		_frameTimeout = (bitsPerChar * 3500000) / baud;
	} else {
		_charTimeout = (bitsPerChar * 1000000) / baud + 750;
		_frameTimeout = (bitsPerChar * 1000000) / baud + 1750;
	}

	if (_dePin != 255) {
		digitalWrite(_dePin, LOW);
		pinMode(_dePin, OUTPUT);
	}
	do {
		if (_serial->available() > 0) {
			_rxStartTime = micros();
			_serial->read();
		}
	} while (micros() - _rxStartTime < _frameTimeout);
}

void ModbusRTUSlave::poll() {
	static uint8_t i = 0;
	static enum pollstateenum {
		STATE_IDLE,
		STATE_FRAME_RX,
		STATE_PARSE,
		STATE_BUILD_REPLY,
		STATE_TRANSMIT,
	} pollstate = STATE_IDLE;

	switch (pollstate) {
		case STATE_IDLE:
			if (_serial->available() > 0) {
				_bufpos = 0;
				_writesize = 0;
				_rxStartTime = micros();
				i = 0;

				pollstate = STATE_FRAME_RX;
			}
			break;

		case STATE_FRAME_RX:

			Serial2.println("Recieved transmission"); // TEMP

			while (_serial->available() > 0 && i < _bufSize) {
				_rxStartTime = micros();
				_buf[i] = _serial->read();
				i++;
			}
			
			if (micros() - _rxStartTime > _charTimeout) {
				pollstate = STATE_PARSE;
			}

			break;

		case STATE_PARSE:
			// this represents an error state, return to idle
			// TODO: should register an error from this!
			// TODO: should this check that time is less than frame timeout?
			if(_serial->available() > 0) {
				pollstate = STATE_IDLE;
				break;
			}

			// minimum size & crc pass
			if((i >= 8)  &&  (_crc(i - 2) == _bytesToWord(_buf[i - 1], _buf[i - 2]))) {
				if((_buf[0] == _id || _buf[0] == 0)) {
					pollstate = STATE_BUILD_REPLY;
					_offset = 0;
				} else if((uint8_t(_buf[0] - _id) < _MULTI_CHANNEL)  &&  (_buf[0] <= _mcLimit)) {
					pollstate = STATE_BUILD_REPLY;
					_offset = _buf[0] - _id;
					Serial2.println("Offset: " + String(_offset)); // TEMP
				} else {
					pollstate = STATE_IDLE;
				}
			} else {
				pollstate = STATE_IDLE;
			}

			break;
		
		case STATE_BUILD_REPLY:
			// this represents an error state, return to idle
			// TODO: should register an error from this!
			if (_serial->available() > 0) {
				pollstate = STATE_IDLE;
				break;
			}

			// must wait at least _frameTimeout before responding
			if (micros() - _rxStartTime < _frameTimeout) {
				break; // go-around
			}

			switch(_buf[1]) {
				case FC_01_R_COILS: /* Read Coils */
					if(_MULTI_CHANNEL == 0 && _coilRead != nullptr) {
						_processBoolRead(_numHoldingRegisters, _coilRead);
					} else if(_coilRead_mc != nullptr) {
						_processBoolRead(_numHoldingRegisters, _coilRead_mc);
					} else {
						Serial2.println(F("ERROR: No _coilRead handler exists to call."));
						pollstate = STATE_IDLE;
					}	

					break;

				case FC_02_R_DISCRETEINPUTS: /* Read Discrete Inputs */
					if(_MULTI_CHANNEL == 0 && _discreteInputRead != nullptr) {
						_processBoolRead(_numHoldingRegisters, _discreteInputRead);
					} else if(_discreteInputRead_mc != nullptr) {
						_processBoolRead(_numHoldingRegisters, _discreteInputRead_mc);
					} else {
						Serial2.println(F("ERROR: No _discreteInputRead handler exists to call."));
						pollstate = STATE_IDLE;
					}					
					
					break;

				case FC_03_R_HOLDINGREGISTERS: /* Read Holding Registers */
					if(_MULTI_CHANNEL == 0 && _holdingRegisterRead != nullptr) {
						_processWordRead(_numHoldingRegisters, _holdingRegisterRead);
					} else if(_holdingRegisterRead_mc != nullptr) {
						_processWordRead(_numHoldingRegisters, _holdingRegisterRead_mc);
					} else {
						Serial2.println(F("ERROR: No _holdingRegisterRead handler exists to call."));
						pollstate = STATE_IDLE;
					}
					
					break;

				case FC_04_R_INPUTREGISTER: /* Read Input Registers */
					if(_MULTI_CHANNEL == 0 && _inputRegisterRead != nullptr) {
						_processWordRead(_numHoldingRegisters, _inputRegisterRead);
					} else if(_inputRegisterRead_mc != nullptr) {
						_processWordRead(_numHoldingRegisters, _inputRegisterRead_mc);
					} else {
						Serial2.println(F("ERROR: No _inputRegisterRead handler exists to call."));
						pollstate = STATE_IDLE;
					}					
					
					break;

				case FC_05_W_SINGLECOIL: /* Write Single Coil */
					{
						uint16_t address = _bytesToWord(_buf[2], _buf[3]);
						uint16_t value = _bytesToWord(_buf[4], _buf[5]);
						if (value != 0 && value != 0xFF00) {
							_exceptionResponse(3);
							break;
						} else if (address >= _numCoils) {
							_exceptionResponse(2);
							break;
						} 
						
						bool b = false;
						if(_MULTI_CHANNEL == 0) { // No multichannel
							if(_coilWrite == nullptr) {
								Serial2.println(F("ERROR: Standard coilWrite handler not provided, aborting."));
							}
							b = _coilWrite(address, value);
						} else { // Multichannel
							if(_coilWrite_mc == nullptr) {
								Serial2.println(F("ERROR: Multichannel coilWrite handler not provided, aborting."));
							}
							b = _coilWrite_mc(address, value, _offset);
						}

						if(b == false) {
							_exceptionResponse(4);
						} else {
							_write(6);
						}
					}
					break;

				case FC_06_W_SINGLEREGISTER: /* Write Single Holding Register */
					{
						uint16_t address = _bytesToWord(_buf[2], _buf[3]);
						uint16_t value = _bytesToWord(_buf[4], _buf[5]);
						if(address >= _numHoldingRegisters) {
							_exceptionResponse(2);
							break;
						}

						bool b = false;
						if(_MULTI_CHANNEL == 0) { // No multichannel
							if(_holdingRegisterWrite == nullptr) {
								Serial2.println(F("ERROR: Standard _holdingRegisterWrite handler not provided, aborting."));
							}
							b = _holdingRegisterWrite(address, value);
						} else { // Multichannel
							if(_holdingRegisterWrite_mc == nullptr) {
								Serial2.println(F("ERROR: Multichannel _holdingRegisterWrite handler not provided, aborting."));
							}
							b = _holdingRegisterWrite_mc(address, value, _offset);
						}

						if(b == false) {
							_exceptionResponse(4);
						} else {
							_write(6);
						}
					}
					break;

				case FC_15_W_MULTIPLECOILS: /* Write Multiple Coils */
					{
						uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
						uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
						if(quantity == 0 || quantity > ((_bufSize - 10) << 3) || _buf[6] != _div8RndUp(quantity)) _exceptionResponse(3);
						else if((startAddress + quantity) > _numCoils) _exceptionResponse(2);
						else {
							for(uint8_t j = 0; j < quantity; j++) {
								bool b = false;
								if(_MULTI_CHANNEL == 0) { // No multichannel
									if(_coilWrite == nullptr) {
										Serial2.println(F("ERROR: Standard _coilWrite handler not provided, aborting."));
									}
									b = _coilWrite(startAddress + j, bitRead(_buf[7 + (j >> 3)], j & 7));
								} else { // Multichannel
									if(_coilWrite_mc == nullptr) {
										Serial2.println(F("ERROR: Multichannel _coilWrite handler not provided, aborting."));
									}
									b = _coilWrite_mc(startAddress + j, bitRead(_buf[7 + (j >> 3)], j & 7), _offset);
								}

								if(b == false) {
									_exceptionResponse(4);
									pollstate = STATE_TRANSMIT;
									return;
								}
							}
							_write(6);
						}
					}
					break;

				case FC_16_W_MULTIPLEREGISTERS: /* Write Multiple Holding Registers */
					{
						uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
						uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
						if(quantity == 0 || quantity > ((_bufSize - 10) >> 1) || _buf[6] != (quantity * 2)) _exceptionResponse(3);
						else if(startAddress + quantity > _numHoldingRegisters) _exceptionResponse(2);
						else {
							for (uint8_t j = 0; j < quantity; j++) {
								bool b = false;
								if(_MULTI_CHANNEL == 0) { // No multichannel
									if(_holdingRegisterWrite == nullptr) {
										Serial2.println(F("ERROR: Standard _holdingRegisterWrite handler not provided, aborting."));
									}
									b = _holdingRegisterWrite(startAddress + j, _bytesToWord(_buf[j * 2 + 7], _buf[j * 2 + 8]));
								} else { // Multichannel
									if(_holdingRegisterWrite_mc == nullptr) {
										Serial2.println(F("ERROR: Multichannel _holdingRegisterWrite handler not provided, aborting."));
									}
									b = _holdingRegisterWrite_mc(startAddress + j, _bytesToWord(_buf[j * 2 + 7], _buf[j * 2 + 8]), _offset);
								}

								if(b == false) {
									_exceptionResponse(4);
									pollstate = STATE_TRANSMIT;
									return;
								}
							}
							_write(6);
						}
					}
					break;

				default:
					_exceptionResponse(1);
					break;
			}
			pollstate = STATE_TRANSMIT;
			/* FALLTHROUGH */

		case STATE_TRANSMIT:
			// Non-ISR non-blocking operation:
			// On DxCore, the TXC interrupt is already used, so we can't use it for this.
			// Apparently writing to serial from within an ISR is bad practice anyway
			// (though at the low baud rates we operate at it seems to be fine).
			//
			// Check if we can add any more data to buffer, even one byte.
			// The DxCore TX buffer is 64 bytes deep - at 19200 baud and 11 bits per byte,
			// this _cannot_ be blocked for more than 36ms!!
			if(!_blocking && _bufpos != _writesize && _serial->availableForWrite()) {
				int writesize = _serial->availableForWrite();

				// write as much as we can
				if(_writesize - _bufpos > (uint16_t)writesize){
					_serial->write(_buf + _bufpos, writesize);
					_bufpos = _bufpos + (uint16_t)writesize;

				// write the remainder of the message and move the marker
				// do not write if we have already written everything!
				} else if (_bufpos != _writesize) {
					_serial->write(_buf + _bufpos, _writesize - _bufpos);
					_bufpos = _writesize;
				}
			}

			// if we are not blocking but do not have a transmit irq defined,
			// we have no way of getting feedback. If the buffer position matches writesize,
			// all we can do is assume it's done and go back to IDLE!
			// The consequences are low - in this condition, transmit should complete on its own,
			// and we by definition cannot receive data while transmitting anyway.
			//
			// The worst consequences are if the next operation is something which changes the serial
			// port's settings - data may be lost in this case, so the user will be responsible for checking
			// the transmission is done somehow.
			//
			// If we are not blocking but _do_ have a transmit irq defined,
			// _transmitting is toggled to false in the IRQ, so this is not needed.
			if (!_blocking && *_txirq_enable_disable == 0 && _bufpos == _writesize) {
				_transmitting = false;
			}

			// wait for transmission to finish
			// blocking operation should still toggle _transmitting correctly
			// TODO: some sort of safety timeout?
			if (!_transmitting) {
				pollstate = STATE_IDLE;
			}

			break;
		
		default:
			pollstate = STATE_IDLE;

			break;
	}
}

void ModbusRTUSlave::_processBoolRead(uint16_t numBools, BoolRead boolRead) {
	if(boolRead == nullptr) {
		Serial2.println(F("ERROR: Standard boolRead has no handler assigned to it, aborting..."));
		return;
	}

	uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
	uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
	if (quantity == 0 || quantity > ((_bufSize - 6) * 8)) _exceptionResponse(3);
	else if ((startAddress + quantity) > numBools) _exceptionResponse(2);
	else {
		for (uint8_t j = 0; j < quantity; j++) {
			uint8_t err = 0;
			int8_t value = boolRead(startAddress + j, &err);
			if (err > 0) {
				_exceptionResponse(err);
				return;
			}
			bitWrite(_buf[3 + (j >> 3)], j & 7, value);
		}
		_buf[2] = _div8RndUp(quantity);
		_write(3 + _buf[2]);
	}
}

void ModbusRTUSlave::_processWordRead(uint16_t numWords, WordRead wordRead) {
	if(wordRead == nullptr) {
		Serial2.println(F("ERROR: Standard wordRead has no handler assigned to it, aborting..."));
		return;
	}
	
	uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
	uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
	if (quantity == 0 || quantity > ((_bufSize - 6) >> 1)) _exceptionResponse(3);
	else if ((startAddress + quantity) > numWords) _exceptionResponse(2);
	else {
		for (uint8_t j = 0; j < quantity; j++) {
			uint8_t err = 0;
			int32_t value = wordRead(startAddress + j, &err);
			if (err > 0) {
				_exceptionResponse(err);
				return;
			}
			_buf[3 + (j * 2)] = highByte(value);
			_buf[4 + (j * 2)] = lowByte(value);
		}
		_buf[2] = quantity * 2;
		_write(3 + _buf[2]);
	}
}

void ModbusRTUSlave::_processBoolRead(uint16_t numBools, BoolRead_mc boolRead) {
	if(boolRead == nullptr) {
		Serial2.println(F("ERROR: Multi-channel boolRead has no handler assigned to it, aborting..."));
		return;
	}

	uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
	uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
	if (quantity == 0 || quantity > ((_bufSize - 6) * 8)) _exceptionResponse(3);
	else if ((startAddress + quantity) > numBools) _exceptionResponse(2);
	else {
		for (uint8_t j = 0; j < quantity; j++) {
			uint8_t err = 0;
			int8_t value = boolRead(startAddress + j, &err, _offset);
			if (err > 0) {
				_exceptionResponse(err);
				return;
			}
			bitWrite(_buf[3 + (j >> 3)], j & 7, value);
		}
		_buf[2] = _div8RndUp(quantity);
		_write(3 + _buf[2]);
	}
}

void ModbusRTUSlave::_processWordRead(uint16_t numWords, WordRead_mc wordRead) {
	if(wordRead == nullptr) {
		Serial2.println(F("ERROR: Multi-channel wordRead has no handler assigned to it, aborting..."));
		return;
	}
	
	uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
	uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
	if (quantity == 0 || quantity > ((_bufSize - 6) >> 1)) _exceptionResponse(3);
	else if ((startAddress + quantity) > numWords) _exceptionResponse(2);
	else {
		for (uint8_t j = 0; j < quantity; j++) {
			uint8_t err = 0;
			int32_t value = wordRead(startAddress + j, &err, _offset);
			if (err > 0) {
				_exceptionResponse(err);
				return;
			}
			_buf[3 + (j * 2)] = highByte(value);
			_buf[4 + (j * 2)] = lowByte(value);
		}
		_buf[2] = quantity * 2;
		_write(3 + _buf[2]);
	}
}

void ModbusRTUSlave::_exceptionResponse(uint8_t code) {
	_buf[1] |= 0x80;
	_buf[2] = code;
	_write(3);
}

void ModbusRTUSlave::_write(uint8_t len) {
	delay(_responseDelay);
	if (_buf[0] != 0) {
		_transmitting = true;
		uint16_t crc = _crc(len);
		_buf[len] = lowByte(crc);
		_buf[len + 1] = highByte(crc);
		if (_dePin != 255) digitalWrite(_dePin, HIGH);
		_writesize = len + 2;

		// chunk writes under the following conditions:
		// 1. trying to write more than available buffer space
		// 2. we're not blocking
		//
		// if not, write entire buffer
		if (_writesize > (uint16_t)_serial->availableForWrite() && !_blocking){
			_bufpos = (uint16_t)_serial->availableForWrite();
			_serial->write(_buf, _bufpos);
		} else {
			_serial->write(_buf, _writesize);
			_bufpos = _writesize;
		}

		// enable interrupt under the following conditions:
		// 1. we're not blocking
		// 2. we have an interrupt enable/disable function defined
		//
		// if blocking, flush and write dePin low
		//
		// otherwise, we will check back in on this once we come around to poll()
		if (!_blocking && _txirq_enable_disable){
			_txirq_enable_disable(true); // provided by project
		} else if (_blocking) {
			_serial->flush();
			if (_dePin != 255) digitalWrite(_dePin, LOW);
			_transmitting = false;
		}
		
	
	}
}

// we can't trust _transmitting for anything outside this library, IF NOT USING TXIRQ!
// Specifically, if we are on a board with no TxIRQ but doing non-blocking operation
// _transmitting will be set to false when we load the last set of data into buffer,
// not when we are actually done.
//
// TxIRQ users can trust this.
bool ModbusRTUSlave::getTransmitting(void) {
	return _transmitting;
}

void ModbusRTUSlave::setMultiChannelLimit(uint8_t in_limit) {
	if(in_limit > 227) {
		in_limit = 227;
	}
	_mcLimit = in_limit;
}

uint16_t ModbusRTUSlave::_crc(uint8_t len) {
	uint16_t crc = 0xFFFF;
	for (uint8_t i = 0; i < len; i++) {
		crc ^= (uint16_t)_buf[i];
		for (uint8_t j = 0; j < 8; j++) {
			bool lsb = crc & 1;
			crc >>= 1;
			if (lsb == true) {
				crc ^= 0xA001;
			}
		}
	}
	return crc;
}

uint16_t ModbusRTUSlave::_div8RndUp(uint16_t value) {
	return (value + 7) >> 3;
}

uint16_t ModbusRTUSlave::_bytesToWord(uint8_t high, uint8_t low) {
	return (high << 8) | low;
}

void ModbusRTUSlave::txDone_irq(void){
	_txirq_enable_disable(false); // provided by project
	if (_dePin != 255) digitalWrite(_dePin, LOW);
	_transmitting = false;
}
