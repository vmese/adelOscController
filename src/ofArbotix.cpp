/*
 * 1/9/13:
 *   - Fixed issue where digitalPinchange
 *
 * 9/28/11:
 *   - updated to be Firmata 2.3/Arduino 1.0 compatible
 *   - fixed ability to use analog pins as digital inputs
 *
 * 3/5/11:
 *   - added servo support for firmata 2.2 and greater (should be
 *     backwards compatible with Erik Sjodin's older firmata servo
 *     implementation)
 *
 *
 * Copyright 2007-2008 (c) Erik Sjodin, eriksjodin.net
 *
 * Devloped at: The Interactive Institutet / Art and Technology,
 * OF Lab / Ars Electronica
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial _portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

//#include "StdAfx.h"
#include "ofArbotix.h"
////#include "ofUtils.h"

// TODO thread it?
// TODO throw event or exception if the serial port goes down...
//---------------------------------------------------------------------------
ofArbotix::ofArbotix(){
	_portStatus=-1;
	_waitForData=0;
	_analogHistoryLength = 2;
	_digitalHistoryLength = 2;
	_stringHistoryLength = 1;
	_sysExHistoryLength = 1;

	_majorProtocolVersion = 0;
	_minorProtocolVersion = 0;
	_majorFirmwareVersion = 0;
	_minorFirmwareVersion = 0;
	_firmwareName = "Unknown";

	bUseDelay = true;
	_dynamixelMoveAdds = 0;

	_waitingForSysExMessage = -1;
	_sysExMessageFound = false;
	_firmwareReceived = false;
}

ofArbotix::~ofArbotix() {
	_port.close();
}

// initialize pins once we get the Firmata version back from the Arduino board
// the version is sent automatically by the Arduino board on startup
void ofArbotix::initPins() {
    int firstAnalogPin;

    if (_initialized) return;   // already initialized

    // support Firmata 2.3/Arduino 1.0 with backwards compatibility
    // to previous protocol versions
    if (_firmwareVersionSum >= FIRMWARE2_3) {
        _totalDigitalPins = 20;
        firstAnalogPin = 14;
    } else {
        _totalDigitalPins = ARD_TOTAL_DIGITAL_PINS;
        firstAnalogPin = 16;
    }

    // ports
	for(int i=0; i<ARD_TOTAL_PORTS; ++i) {
		_digitalPortValue[i]=0;
		_digitalPortReporting[i] = ARD_OFF;
	}

    // digital pins
	for(int i=0; i<firstAnalogPin; ++i) {
		_digitalPinValue[i] = -1;
		_digitalPinMode[i] = ARD_OUTPUT;
		_digitalPinReporting[i] = ARD_OFF;
	}

	// analog in pins
    for (int i=firstAnalogPin; i<_totalDigitalPins; ++i) {
		_analogPinReporting[i-firstAnalogPin] = ARD_OFF;
		// analog pins used as digital
		_digitalPinMode[i]=ARD_ANALOG;
		_digitalPinValue[i] = -1;
	}

	for (int i=0; i<_totalDigitalPins; ++i) {
		_servoValue[i] = -1;
	}

    _initialized = true;
}

bool ofArbotix::connect(std::string device, int baud){
	try
	{
		_Timer.restart();
		_initialized = false;
        _port.listDevices();
		connected = _port.setup(device.c_str(), baud);
		return connected;
	}
	catch(...)
	{
		return false;
	}

}

// this method is not recommended
// the preferred method is to listen for the EInitialized event in your application
bool ofArbotix::isArduinoReady(){
	if(bUseDelay) {
		if (_initialized || (_Timer.elapsed() > OF_ARDUINO_DELAY_LENGTH)) {
			initPins();
			connected = true;
		}
	}
	return connected;
}

void  ofArbotix::setUseDelay(bool bDelay){
	bUseDelay = bDelay;
}

void ofArbotix::setDigitalHistoryLength(int length){
	if(length>=2)
		_digitalHistoryLength=length;
}

void ofArbotix::setAnalogHistoryLength(int length){
	if(length>=2)
		_analogHistoryLength=length;
}

void ofArbotix::setSysExHistoryLength(int length){
	if(length>=1)
		_sysExHistoryLength=length;
}

void ofArbotix::setStringHistoryLength(int length){
	if(length>=1)
		_stringHistoryLength=length;
}

void ofArbotix::disconnect(){
	_port.close();
}

void ofArbotix::update(){
	static std::vector<unsigned char> bytesToProcess;
	int bytesToRead = _port.available();
	if (bytesToRead>0) {
		bytesToProcess.resize(bytesToRead);
		_port.readBytes(&bytesToProcess[0], bytesToRead);
		for (int i = 0; i < bytesToRead; i++) {
			processData((char)(bytesToProcess[i]));
		}
	}
}

int ofArbotix::getAnalog(int pin){
	if(_analogHistory[pin].size()>0)
		return _analogHistory[pin].front();
	else
		return -1;
}

int ofArbotix::getDigital(int pin){
	if((_digitalPinMode[pin]==ARD_INPUT || _digitalPinMode[pin]==ARD_INPUT_PULLUP) && _digitalHistory[pin].size()>0)
		return _digitalHistory[pin].front();
	else if (_digitalPinMode[pin]==ARD_OUTPUT)
		return _digitalPinValue[pin];
	else
		return -1;
}

int ofArbotix::getPwm(int pin){
	if(_digitalPinMode[pin]==ARD_PWM)
		return _digitalPinValue[pin];
	else
		return -1;
}

std::vector<unsigned char> ofArbotix::getSysEx(){
	return _sysExHistory.front();
}

std::string ofArbotix::getString(){
	return _stringHistory.front();
}

int ofArbotix::getDigitalPinMode(int pin){
	return _digitalPinMode[pin];
}

void ofArbotix::sendDigital(int pin, int value, bool force){
	if((_digitalPinMode[pin]==ARD_INPUT || _digitalPinMode[pin]==ARD_INPUT_PULLUP || _digitalPinMode[pin]==ARD_OUTPUT) && (_digitalPinValue[pin]!=value || force)){

		_digitalPinValue[pin] = value;

		int port=0;
		int bit=0;
        int port1Offset;
        int port2Offset;

        // support Firmata 2.3/Arduino 1.0 with backwards compatibility
        // to previous protocol versions
        if (_firmwareVersionSum >= FIRMWARE2_3) {
            port1Offset = 16;
            port2Offset = 20;
        } else {
            port1Offset = 14;
            port2Offset = 22;
        }

		if(pin < 8 && pin >1){
			port=0;
			bit = pin;
		}
		else if(pin>7 && pin <port1Offset){
			port = 1;
			bit = pin-8;
		}
		else if(pin>15 && pin <port2Offset){
			port = 2;
			bit = pin-16;
		}

		// set the bit
		if(value==1)
			_digitalPortValue[port] |= (1 << bit);

		// clear the bit
		if(value==0)
			_digitalPortValue[port] &= ~(1 << bit);

		sendByte(FIRMATA_DIGITAL_MESSAGE+port);
		sendValueAsTwo7bitBytes(_digitalPortValue[port]);

	}
}

void ofArbotix::sendPwm(int pin, int value, bool force){
	if(_digitalPinMode[pin]==ARD_PWM && (_digitalPinValue[pin]!=value || force)){
		sendByte(FIRMATA_ANALOG_MESSAGE+pin);
		sendValueAsTwo7bitBytes(value);
		_digitalPinValue[pin] = value;
	}
}

void ofArbotix::sendSysEx(int command, std::vector<unsigned char> data){
	sendByte(FIRMATA_START_SYSEX);
	sendByte(command);
	std::vector<unsigned char>::iterator it = data.begin();
	while( it != data.end() ) {
		//sendByte(*it);	// need to split data into 2 bytes before sending
		sendValueAsTwo7bitBytes(*it);
		it++;
	}
	sendByte(FIRMATA_END_SYSEX);
}

void ofArbotix::sendSysExBegin(){
	sendByte(FIRMATA_START_SYSEX);
}

void ofArbotix::sendSysExEnd(){
	sendByte(FIRMATA_END_SYSEX);
}

void ofArbotix::sendString(std::string str){
	sendByte(FIRMATA_START_SYSEX);
	sendByte(FIRMATA_SYSEX_FIRMATA_STRING);
	std::string::iterator it = str.begin();
	while( it != str.end() ) {
		sendValueAsTwo7bitBytes(*it);
		it++;
	}
	sendByte(FIRMATA_END_SYSEX);
}

void ofArbotix::sendProtocolVersionRequest(){
	sendByte(FIRMATA_REPORT_VERSION);
}

void ofArbotix::sendFirmwareVersionRequest(){
	_firmwareReceived = false;
	sendByte(FIRMATA_START_SYSEX);
	sendByte(FIRMATA_SYSEX_REPORT_FIRMWARE);
	sendByte(FIRMATA_END_SYSEX);
}

void ofArbotix::sendReset(){
	_port.flush();
	sendByte(FIRMATA_SYSTEM_RESET);
}

void ofArbotix::sendAnalogPinReporting(int pin, int mode){

    int firstAnalogPin;
    // support Firmata 2.3/Arduino 1.0 with backwards compatibility
    // to previous protocol versions
    if (_firmwareVersionSum >= FIRMWARE2_3) {
        firstAnalogPin = 14;
    } else {
        firstAnalogPin = 16;
    }

    // if this analog pin is set as a digital input, disable digital pin reporting
    if (_digitalPinReporting[pin + firstAnalogPin] == ARD_ON) {
        sendDigitalPinReporting(pin + firstAnalogPin, ARD_OFF);
    }

	_digitalPinMode[firstAnalogPin+pin]=ARD_ANALOG;

	sendByte(FIRMATA_REPORT_ANALOG+pin);
	sendByte(mode);
	_analogPinReporting[pin] = mode;
}

void ofArbotix::sendDigitalPinMode(int pin, int mode){
	sendByte(FIRMATA_SET_PIN_MODE);
	sendByte(pin);
	sendByte(mode);
	_digitalPinMode[pin]=mode;

	// turn on or off reporting on the port
	if(mode==ARD_INPUT || mode==ARD_INPUT_PULLUP){
		sendDigitalPinReporting(pin, ARD_ON);
	}
	else {
		sendDigitalPinReporting(pin, ARD_OFF);
	}
}

int ofArbotix::getAnalogPinReporting(int pin){
	return _analogPinReporting[pin];
}

std::list<int>* ofArbotix::getAnalogHistory(int pin){
	return &_analogHistory[pin];
}

std::list<int>* ofArbotix::getDigitalHistory(int pin){
	return &_digitalHistory[pin];
}

std::list<std::vector<unsigned char> >* ofArbotix::getSysExHistory(){
	return &_sysExHistory;
}

std::list<std::string>* ofArbotix::getStringHistory(){
	return &_stringHistory;
}

int ofArbotix::getMajorProtocolVersion(){
	return _majorFirmwareVersion;
}

int ofArbotix::getMinorProtocolVersion(){
	return _minorFirmwareVersion;
}

int ofArbotix::getMajorFirmwareVersion(){
	return _majorFirmwareVersion;
}

int ofArbotix::getMinorFirmwareVersion(){
	return _minorFirmwareVersion;
}

std::string ofArbotix::getFirmwareName(){
	return _firmwareName;
}

int ofArbotix::makeWord(unsigned char  low, unsigned char  high)
{
	return (int) (low + (high<<8));
}

unsigned char  ofArbotix::getLowByte(int val)
{
	return (unsigned char ) (val & 0xff);
}

unsigned char  ofArbotix::getHighByte(int val)
{
	return (unsigned char ) ((val & 0xff00)>> 8);
}

bool ofArbotix::isInitialized(){
	return _initialized;
}

// ------------------------------ private functions

void ofArbotix::processData(unsigned char inputData){

	char msg[100];
    //sprintf(msg, "Received Byte: %i", inputData);
    //printf("Received Byte: %i", inputData);
	//Logger::get("Application").information(msg);

	// we have command data
	if(_waitForData>0 && inputData<128) {
		_waitForData--;

		// collect the data
		_storedInputData[_waitForData] = inputData;

		// we have all data executeMultiByteCommand
		if(_waitForData==0) {
			switch (_executeMultiByteCommand) {
				case FIRMATA_DIGITAL_MESSAGE:
					processDigitalPort(_multiByteChannel, (_storedInputData[0] << 7) | _storedInputData[1]);
				break;
				case FIRMATA_REPORT_VERSION: // report version
					_majorProtocolVersion = _storedInputData[1];
					_minorProtocolVersion = _storedInputData[0];
					EProtocolVersionReceived(_majorProtocolVersion);
				break;
				case FIRMATA_ANALOG_MESSAGE:
					if(_analogHistory[_multiByteChannel].size()>0){
						int previous = _analogHistory[_multiByteChannel].front();

						_analogHistory[_multiByteChannel].push_front((_storedInputData[0] << 7) | _storedInputData[1]);
						if((int)_analogHistory[_multiByteChannel].size()>_analogHistoryLength)
							_analogHistory[_multiByteChannel].pop_back();

						// trigger an event if the pin has changed value
						if(_analogHistory[_multiByteChannel].front()!=previous)
							EAnalogPinChanged(_multiByteChannel);
					}else{
						_analogHistory[_multiByteChannel].push_front((_storedInputData[0] << 7) | _storedInputData[1]);
						if((int)_analogHistory[_multiByteChannel].size()>_analogHistoryLength)
							_analogHistory[_multiByteChannel].pop_back();
					}
				break;
			}

		}
	}
	// we have SysEx command data
	else if(_waitForData<0){

		// we have all sysex data
		if(inputData==FIRMATA_END_SYSEX){
			_waitForData=0;
            //if(_sysExData.size() > 0)
				processSysExData(_sysExData);
			_sysExData.clear();
		}
		// still have data, collect it
		else {
			_sysExData.push_back((unsigned char)inputData);
		}
	}
	// we have a command
	else{

		int command;

		// extract the command and channel info from a byte if it is less than 0xF0
		if(inputData < 0xF0) {
		  command = inputData & 0xF0;
		  _multiByteChannel = inputData & 0x0F;
		}
		else {
		  // commands in the 0xF* range don't use channel data
		  command = inputData;
		}

		switch (command) {
			case FIRMATA_REPORT_VERSION:
			case FIRMATA_DIGITAL_MESSAGE:
			case FIRMATA_ANALOG_MESSAGE:
				_waitForData = 2;  // 2 bytes needed
				_executeMultiByteCommand = command;
			break;
			case FIRMATA_START_SYSEX:
				_sysExData.clear();
				_waitForData = -1;  // n bytes needed, -1 is used to indicate sysex message
				_executeMultiByteCommand = command;
			break;
		}

	}
}

// sysex data is assumed to be 8-bit bytes split into two 7-bit bytes.
void ofArbotix::processSysExData(std::vector<unsigned char> data){

	std::string str;

	std::vector<unsigned char>::iterator it;
	std::vector<unsigned char>::iterator vend;
	unsigned char buffer;
	unsigned int iID = 0;
	unsigned char iCmd = 0;
	int iChecksum = 0, iRecChecksum = 0;
	int iSize=0;
	//int i = 1;

	// act on reserved sysEx messages (extended commands) or trigger SysEx event...
	switch(data.front()) { //first byte in buffer is command
		case FIRMATA_SYSEX_REPORT_FIRMWARE:
			it = data.begin();
			it++; // skip the first byte, which is the firmware version command
			_majorFirmwareVersion = *it;
			it++;
			_minorFirmwareVersion = *it;
			it++;

			while( it != data.end() ) {
					buffer = *it;
					it++;
					buffer += *it << 7;
					it++;
					str+=buffer;
			}
			_firmwareName = str;
			_firmwareVersionSum = _majorFirmwareVersion * 10 + _minorFirmwareVersion;

			_firmwareReceived = true;
			std::cout << this->getFirmwareName(); 
			std::cout << "firmata v" << this->getMajorFirmwareVersion() << "." << this->getMinorFirmwareVersion() << "\r\n";
			EFirmwareVersionReceived(_majorFirmwareVersion);

			// trigger the initialization event
            if (!_initialized) {
                initPins();
                EInitialized(_majorFirmwareVersion);

            }

			checkIncomingSysExMessage(FIRMATA_SYSEX_REPORT_FIRMWARE);

		break;
		case FIRMATA_SYSEX_FIRMATA_STRING:
			it = data.begin();
			it++; // skip the first byte, which is the string command
			while( it != data.end() ) {
					buffer = *it;
					it++;
					buffer += *it << 7;
					it++;
					str+=buffer;
			}

			_stringHistory.push_front(str);
			if((int)_stringHistory.size()>_stringHistoryLength)
					_stringHistory.pop_back();

			EStringReceived(str);

			checkIncomingSysExMessage(FIRMATA_SYSEX_FIRMATA_STRING);
		break;
		case SYSEX_DYNAMIXEL_KEY_SERVO_DATA:
			it = data.begin();
			vend = data.end();
			it++; // skip the first byte, which is the Dynamixel servo command

			//Get the servo ID number
			iID = getByteFromDataIterator(it, vend);

			if(iID < MAX_DYNAMIXEL_SERVOS && data.size() == DYNAMIXEL_KEY_DATA_LENGTH)
			{
				//Now get current position.
				unsigned int iPos = GetWordFromDataIterator(it, vend);

				//Now get the speed.
				unsigned int iSpeed = GetWordFromDataIterator(it, vend);

				//Get the checksum
				unsigned char iRecChecksum = getByteFromDataIterator(it, vend);
				unsigned int iCheckSum = (~(iID + iPos + iSpeed)) & 0xFF;

				if(iCheckSum == iRecChecksum)
				{
					_dynamixelServos[iID]._keyChanged = true;
					_dynamixelServos[iID]._actualPosition = iPos;
					_dynamixelServos[iID]._actualSpeed = iSpeed;

					EDynamixelKeyReceived(iID);
					checkIncomingSysExMessage(SYSEX_DYNAMIXEL_KEY_SERVO_DATA);
				}
			}
		break;
		case SYSEX_DYNAMIXEL_ALL_SERVO_DATA:
			it = data.begin();
			vend = data.end();
			it++; // skip the first byte, which is the Dynamixel servo command

			//Get the servo ID number
			iID = getByteFromDataIterator(it, vend);

			if(iID < MAX_DYNAMIXEL_SERVOS && data.size() == DYNAMIXEL_ALL_DATA_LENGTH)
			{
				//Now get current position.
				unsigned int iPos = GetWordFromDataIterator(it, vend);

				//Now get the speed.
				unsigned int iSpeed = GetWordFromDataIterator(it, vend);

				//Now get the load.
				unsigned int iLoad = GetWordFromDataIterator(it, vend);

				//Now get the voltage.
				unsigned char cVoltage = getByteFromDataIterator(it, vend);

				//Now get the voltage.
				unsigned char cTemp = getByteFromDataIterator(it, vend);

				//Get the checksum
				unsigned char iRecChecksum = getByteFromDataIterator(it, vend);
				unsigned int iCheckSum = (~(iID + iPos + iSpeed + iLoad + cVoltage + cTemp)) & 0xFF;

				if(iCheckSum == iRecChecksum)
				{
                    printf("update temp\n");
					_dynamixelServos[iID]._keyChanged = true;
					_dynamixelServos[iID]._actualPosition = iPos;
					_dynamixelServos[iID]._actualSpeed = iSpeed;
					_dynamixelServos[iID]._load = iLoad;
					_dynamixelServos[iID]._voltage = cVoltage;
					_dynamixelServos[iID]._temperature = cTemp;

					EDynamixelAllReceived(iID);
					checkIncomingSysExMessage(SYSEX_DYNAMIXEL_ALL_SERVO_DATA);
				}
			}
		break;
		case SYSEX_DYNAMIXEL_TRANSMIT_ERROR:
			it = data.begin();
			vend = data.end();
			it++; // skip the first byte, which is the Dynamixel servo command

			//Get the command value
			iCmd = getByteFromDataIterator(it, vend);

			//Get the servo ID number
			iID = getByteFromDataIterator(it, vend);

			//Get the servo ID number
			iRecChecksum = getByteFromDataIterator(it, vend);

			//Get the servo ID number
			iChecksum = (~(iCmd + iID)) & 0xFF;

			if(iChecksum == iRecChecksum) {
				EDynamixelTransmitError(iCmd, iID);
				checkIncomingSysExMessage(SYSEX_DYNAMIXEL_TRANSMIT_ERROR);
			}
		break;
		case SYSEX_DYNAMIXEL_GET_REGISTER:
			if(data.size() == DYNAMIXEL_GET_REGISTER_LENGTH) {
				it = data.begin();
                vend = data.end();
				it++; // skip the first byte, which is the Dynamixel servo command

				//Get the servo value
				iID = getByteFromDataIterator(it, vend);

				//Get the register address
				unsigned int iReg = getByteFromDataIterator(it, vend);

				//Get the register value
				unsigned int iValue = GetWordFromDataIterator(it, vend);

				//Get the checksum
				iRecChecksum = getByteFromDataIterator(it, vend);

				//Get the servo ID number
				iChecksum = (~(iID + iReg + iValue)) & 0xFF;

				if(iChecksum == iRecChecksum) {
					EDynamixelGetRegister(iID, iReg, iValue);
					checkIncomingSysExMessage(SYSEX_DYNAMIXEL_GET_REGISTER);
				}
			}
		break;
		case SYSEX_COMMANDER_DATA:
			if(data.size() == COMMANDER_DATA_LENGTH)
			{
				it = data.begin();
                vend = data.end();
				it++; // skip the first byte, which is the Commander command

				signed char iWalkV = (signed char) getByteFromDataIterator(it, vend);
				signed char iWalkH = (signed char) getByteFromDataIterator(it, vend);
				signed char iLookV = (signed char) getByteFromDataIterator(it, vend);
				signed char iLookH = (signed char) getByteFromDataIterator(it, vend);
				signed char iButtons = (signed char) getByteFromDataIterator(it, vend);
				iRecChecksum = getByteFromDataIterator(it, vend);
				iChecksum = (~(iWalkV + iWalkH + iLookV + iLookH + iButtons)) & 0xFF;

				if(iChecksum == iRecChecksum)
				{
					_commanderData._changed = true;
					_commanderData._walkV = iWalkV;
					_commanderData._walkH = iWalkH;
					_commanderData._lookV = iLookV;
					_commanderData._lookH = iLookH;
					_commanderData._buttons = iButtons;
					//_commanderData._ext = (signed char) getByteFromDataIterator(it, data.end());

					ECommanderDataReceived(iID);
					checkIncomingSysExMessage(SYSEX_COMMANDER_DATA);
				}
			}
		break;
		case SYSEX_DYNAMIXEL_STOPPED:
			it = data.begin();
			vend = data.end();
			it++; // skip the first byte, which is the Dynamixel servo command

			//Get the servo ID number
			iID = getByteFromDataIterator(it, vend);

			//Get the servo ID number
			iRecChecksum = getByteFromDataIterator(it, vend);

			//Get the servo ID number
			iChecksum = (~(iID)) & 0xFF;

			if(iChecksum == iRecChecksum) {
				EDynamixelStopped(iID);
				checkIncomingSysExMessage(SYSEX_DYNAMIXEL_STOPPED);
			}
		break;
		default: // the message isn't in Firmatas extended command set
			_sysExHistory.push_front(data);
			if((int)_sysExHistory.size()>_sysExHistoryLength)
					_sysExHistory.pop_back();
			ESysExReceived(data);
		break;

	}
}

void ofArbotix::processDigitalPort(int port, unsigned char value){

	unsigned char mask;
	int previous;
	int i;
	int pin;
    int port1Pins;
    int port2Pins;

    // support Firmata 2.3/Arduino 1.0 with backwards compatibility to previous protocol versions
    if (_firmwareVersionSum >= FIRMWARE2_3) {
        port1Pins = 8;
        port2Pins = 4;
    } else {
        port1Pins = 6;
        port2Pins = 6;
    }

	switch(port) {
    case 0: // pins 2-7  (0,1 are ignored as serial RX/TX)
        for(i=2; i<8; ++i) {
            pin = i;
            previous = -1;
            if(_digitalPinMode[pin]==ARD_INPUT || _digitalPinMode[pin]==ARD_INPUT_PULLUP){
              if (_digitalHistory[pin].size() > 0)
                previous = _digitalHistory[pin].front();

                mask = 1 << i;
                _digitalHistory[pin].push_front((value & mask)>>i);

                if((int)_digitalHistory[pin].size()>_digitalHistoryLength)
                        _digitalHistory[pin].pop_back();

                // trigger an event if the pin has changed value
                if(_digitalHistory[pin].front()!=previous){
					EDigitalPinChanged(pin);
                }
            }
        }
        break;
    case 1: // pins 8-13 (in Firmata 2.3/Arduino 1.0, pins 14 and 15 are analog 0 and 1)
        for(i=0; i<port1Pins; ++i) {
            pin = i+8;
            previous = -1;
            if(_digitalPinMode[pin]==ARD_INPUT || _digitalPinMode[pin]==ARD_INPUT_PULLUP){
              if (_digitalHistory[pin].size() > 0)
                previous = _digitalHistory[pin].front();

                mask = 1 << i;
                _digitalHistory[pin].push_front((value & mask)>>i);

                if((int)_digitalHistory[pin].size()>_digitalHistoryLength)
                        _digitalHistory[pin].pop_back();

                // trigger an event if the pin has changed value
                if(_digitalHistory[pin].front()!=previous){
                ////    ofNotifyEvent(EDigitalPinChanged, pin, this);
                }
            }
        }
        break;
	case 2: // analog pins used as digital pins 16-21 (in Firmata 2.3/Arduino 1.0, digital pins 14 - 19)
		for(i=0; i<port2Pins; ++i) {
			//pin = i+analogOffset;
            pin = i+16;
			      previous = -1;
            if(_digitalPinMode[pin]==ARD_INPUT || _digitalPinMode[pin]==ARD_INPUT_PULLUP){
              if (_digitalHistory[pin].size() > 0)
                previous = _digitalHistory[pin].front();

                mask = 1 << i;
                _digitalHistory[pin].push_front((value & mask)>>i);

                if((int)_digitalHistory[pin].size()>_digitalHistoryLength)
                        _digitalHistory[pin].pop_back();

                // trigger an event if the pin has changed value
                if(_digitalHistory[pin].front()!=previous){
                ////    ofNotifyEvent(EDigitalPinChanged, pin, this);
                }
            }
		}
        break;
	}
}

// port 0: pins 2-7  (0,1 are serial RX/TX, don't change their values)
// port 1: pins 8-13 (in Firmata 2.3/Arduino 1.0, pins 14 and 15 are analog pins 0 and 1 used as digital pins)
// port 2: pins 16-21 analog pins used as digital (in Firmata 2.3/Arduino 1.0, pins 14 - 19),
//         all analog reporting will be turned off if this is set to ARD_ON

void ofArbotix::sendDigitalPortReporting(int port, int mode){
	sendByte(FIRMATA_REPORT_DIGITAL+port);
	sendByte(mode);
	_digitalPortReporting[port] = mode;
    int offset;

    if (_firmwareVersionSum >= FIRMWARE2_3) {
        offset = 2;
    } else {
        offset = 0;
    }

    // for Firmata 2.3 and higher:
    if(port==1 && mode==ARD_ON) {
        for (int i=0; i<2; i++) {
            _analogPinReporting[i] = ARD_OFF;
		}
    }

    // for Firmata 2.3 and all prior Firmata protocol versions:
    if(port==2 && mode==ARD_ON){ // if reporting is turned on on port 2 then ofArbotix on the Arduino disables all analog reporting

		for (int i=offset; i<ARD_TOTAL_ANALOG_PINS; i++) {
				_analogPinReporting[i] = ARD_OFF;
		}
	}
}

void ofArbotix::sendDigitalPinReporting(int pin, int mode){
	_digitalPinReporting[pin] = mode;
    int port1Offset;
    int port2Offset;

    // Firmata backwards compatibility mess
    if (_firmwareVersionSum >= FIRMWARE2_3) {
        port1Offset = 15;
        port2Offset = 19;
    } else {
        port1Offset = 13;
        port2Offset = 21;
    }

	if(mode==ARD_ON){	// enable reporting for the port
		if(pin<=7 && pin>=2)
			sendDigitalPortReporting(0, ARD_ON);
        // Firmata backwards compatibility mess
        if(pin<=port1Offset && pin>=8)
            sendDigitalPortReporting(1, ARD_ON);
        if(pin<=port2Offset && pin>=16)
            sendDigitalPortReporting(2, ARD_ON);
	}
	else if(mode==ARD_OFF){
		int i;
		bool send=true;
		if(pin<=7 && pin>=2){    // check if all pins on the port are off, if so set port reporting to off..
			for(i=2; i<8; ++i) {
				if(_digitalPinReporting[i]==ARD_ON)
						send=false;
			}
			if(send)
				sendDigitalPortReporting(0, ARD_OFF);
		}
        // Firmata backwards compatibility mess
        if(pin<=port1Offset && pin>=8){
            for(i=8; i<=port1Offset; ++i) {
                if(_digitalPinReporting[i]==ARD_ON)
                        send=false;
            }
            if(send)
                sendDigitalPortReporting(1, ARD_OFF);
        }
        if(pin<=port2Offset && pin>=16){
            for(i=16; i<=port2Offset; ++i) {
                if(_digitalPinReporting[i]==ARD_ON)
                        send=false;
            }
            if(send)
                sendDigitalPortReporting(2, ARD_OFF);
        }
	}
}

void ofArbotix::sendByte(unsigned char value){
	//char msg[100];
	//sprintf(msg, "Sending Byte: %i", byte);
	//Logger::get("Application").information(msg);
	_port.writeByte(value);
}

// in Firmata (and MIDI) data bytes are 7-bits. The 8th bit serves as a flag to mark a byte as either command or data.
// therefore you need two data bytes to send 8-bits (a char).
void ofArbotix::sendValueAsTwo7bitBytes(int value)
{
	sendByte(value & 127); // LSB
	sendByte(value >> 7 & 127); // MSB
}

// SysEx data is sent as 8-bit bytes split into two 7-bit bytes, this function merges two 7-bit bytes back into one 8-bit byte.
int ofArbotix::getValueFromTwo7bitBytes(unsigned char lsb, unsigned char msb){
   return (msb << 7) | lsb;
}

// SysEx data is sent as 8-bit bytes split into two 7-bit bytes, this function merges two 7-bit bytes back into one 8-bit byte.
unsigned int ofArbotix::getByteFromDataIterator(std::vector<unsigned char>::iterator &it, std::vector<unsigned char>::iterator &end){
	if(it == end)
		return 0;
	unsigned char lsb = (*it++);

	if(it == end)
		return 0;
	unsigned char msb = (*it++);

   return (unsigned int) ((msb << 7) | lsb);
}

unsigned int ofArbotix::GetWordFromDataIterator(std::vector<unsigned char>::iterator &it, std::vector<unsigned char>::iterator &end)
{
	int lsb = getByteFromDataIterator(it, end);
	int msb = getByteFromDataIterator(it, end);

	int iRet = (msb << 8) | lsb;
	return iRet;
}

void ofArbotix::sendServo(int pin, int value, bool force){
	// for firmata v2.2 and greater
	if (_firmwareVersionSum >= FIRMWARE2_2) {
		if(_digitalPinMode[pin]==ARD_SERVO && (_digitalPinValue[pin]!=value || force)){
			sendByte(FIRMATA_ANALOG_MESSAGE+pin);
			sendValueAsTwo7bitBytes(value);
			_digitalPinValue[pin] = value;
		}
	}
	// for versions prior to 2.2
	else {
		if(_digitalPinMode[pin]==ARD_SERVO && (_servoValue[pin]!=value || force)){
			sendByte(FIRMATA_START_SYSEX);
			sendByte(SYSEX_SERVO_WRITE);
			sendByte(pin);
			sendValueAsTwo7bitBytes(value);
			sendByte(FIRMATA_END_SYSEX);
			_servoValue[pin]=value;
		}
	}
}

// angle parameter is no longer supported. keeping for backwards compatibility
void ofArbotix::sendServoAttach(int pin, int minPulse, int maxPulse, int angle) {
	sendByte(FIRMATA_START_SYSEX);
	// for firmata v2.2 and greater
	if (_firmwareVersionSum >= FIRMWARE2_2) {
		sendByte(FIRMATA_SYSEX_SERVO_CONFIG);
	}
	// for versions prior to 2.2
	else {
		sendByte(SYSEX_SERVO_ATTACH);
	}
	sendByte(pin);
	sendValueAsTwo7bitBytes(minPulse);
	sendValueAsTwo7bitBytes(maxPulse);
	sendByte(FIRMATA_END_SYSEX);
	_digitalPinMode[pin]=ARD_SERVO;
}

// sendServoDetach depricated as of Firmata 2.2
void ofArbotix::sendServoDetach(int pin) {
	sendByte(FIRMATA_START_SYSEX);
	sendByte(SYSEX_SERVO_DETACH);
	sendByte(pin);
	sendByte(FIRMATA_END_SYSEX);
	_digitalPinMode[pin]=ARD_OUTPUT;
}

int ofArbotix::getServo(int pin){
	if(_digitalPinMode[pin]==ARD_SERVO)
		// for firmata v2.2 and greater
		if (_firmwareVersionSum >= FIRMWARE2_2) {
			return _digitalPinValue[pin];
		}
		// for versions prior to 2.2
		else {
			return _servoValue[pin];
		}
	else
		return -1;
}

//Tells an arbotix board to monitor a given servo and report back its data.
//Please note that this will ONLY work for an arbotix arduino board
void ofArbotix::sendDynamixelServoAttach(unsigned char servo) {
	//Send a sysex to report a dynamixel servo
	std::vector<unsigned char> sysexData;
	sysexData.push_back(servo);
	sysexData.push_back(1);
	this->sendSysEx(SYSEX_DYNAMIXEL_CONFIG, sysexData);
}

//void ofArbotix::resetDynamixelServo(unsigned char servo)
//{
//    printf("ofArbotix::resetDynamixelServo\n");


//}

//Tells an arbotix board to quit monitoring a given servo and report back its data.
//Please note that this will ONLY work for an arbotix arduino board
void ofArbotix::sendDynamixelServoDetach(unsigned char servo) {
	//Send a sysex to report a dynamixel servo
	std::vector<unsigned char> sysexData;
	sysexData.push_back(servo);
	sysexData.push_back(0);
	this->sendSysEx(SYSEX_DYNAMIXEL_CONFIG, sysexData);
}

//Starts a synch move command for the dynamixel motors. You can then set up to 4 motors to be moved at once.
//Please note that this will ONLY work for an arbotix arduino board
void ofArbotix::sendDynamixelSynchMoveStart() {
	//Send a sysex to let the arbotix know that we are starting a new synch move command
	std::vector<unsigned char> sysexData;
	this->sendSysEx(SYSEX_DYNAMIXEL_SYNCH_MOVE_START, sysexData);
	_dynamixelMoveAdds = 0;
}

//Adds a move command for the given servo to the list to be send.
//Please note that this will ONLY work for an arbotix arduino board
void ofArbotix::sendDynamixelSynchMoveAdd(unsigned char servo, int pos, int speed) {
	unsigned char pos0 = getLowByte(pos);
	unsigned char pos1 = getHighByte(pos);
	unsigned char speed0 = getLowByte(speed);
	unsigned char speed1 = getHighByte(speed);
	int checksum = (~(servo + pos0 + pos1 + speed0 + speed1)) & 0xFF;

	//Send a sysex to add this motor to the list for the synch move
	std::vector<unsigned char> sysexData;
	sysexData.push_back(servo);
	sysexData.push_back(pos0);
	sysexData.push_back(pos1);
	sysexData.push_back(speed0);
	sysexData.push_back(speed1);
	sysexData.push_back(checksum);
        this->sendSysEx(SYSEX_DYNAMIXEL_SYNCH_MOVE_ADD, sysexData);

	_dynamixelMoveAdds++;
}

//Transmits the command to move the servos that have been setup using addDynamixelSynchMove
//Please note that this will ONLY work for an arbotix arduino board
void ofArbotix::sendDynamixelSynchMoveExecute() {

	if(	_dynamixelMoveAdds > 0) {
		//Send a sysex to tell the arbotix to execute the synch move
		std::vector<unsigned char> sysexData;
		this->sendSysEx(SYSEX_DYNAMIXEL_SYNCH_MOVE_EXECUTE, sysexData);
		_dynamixelMoveAdds = 0;
	}
}

//Transmits the command to move a single motor. Does not use the synch move.
//Please note that this will ONLY work for an arbotix arduino board
void ofArbotix::sendDynamixelMove(unsigned char servo, int pos, int speed) {

        //printf("send pos : %i - to servo %i - speed = %i\n",pos, servo, speed);
        unsigned char pos0 = getLowByte(pos);
	unsigned char pos1 = getHighByte(pos);
	unsigned char speed0 = getLowByte(speed);
	unsigned char speed1 = getHighByte(speed);
	int checksum = (~(servo + pos0 + pos1 + speed0 + speed1)) & 0xFF;

	//Send a sysex to let the arbotix know that we are starting a new synch move command
	std::vector<unsigned char> sysexData;
	sysexData.push_back(servo);
	sysexData.push_back(pos0);
	sysexData.push_back(pos1);
	sysexData.push_back(speed0);
	sysexData.push_back(speed1);
	sysexData.push_back(checksum);
	this->sendSysEx(SYSEX_DYNAMIXEL_MOVE, sysexData);
}

//Transmits the stop to move a single motor.
//Please note that this will ONLY work for an arbotix arduino board
void ofArbotix::sendDynamixelStop(unsigned char servo) {
	int checksum = (~(servo)) & 0xFF;

	//Send a sysex to let the arbotix know that we are starting a new synch move command
	std::vector<unsigned char> sysexData;
	sysexData.push_back(servo);
	sysexData.push_back(checksum);
	this->sendSysEx(SYSEX_DYNAMIXEL_STOP, sysexData);
}


//Transmits the command to set a byte of the servo register.
void ofArbotix::sendDynamixelSetRegister(unsigned char servo, unsigned char reg, unsigned char length, unsigned int value) {
	unsigned char val0 = getLowByte(value);
	unsigned char val1 = getHighByte(value);
	int checksum = (~(servo + reg + length + val0 + val1)) & 0xFF;

	//Send a sysex to let the arbotix know that we are starting a new synch move command
	std::vector<unsigned char> sysexData;
	sysexData.push_back(servo);
	sysexData.push_back(reg);
	sysexData.push_back(length);
	sysexData.push_back(val0);
	sysexData.push_back(val1);
	sysexData.push_back(checksum);
	this->sendSysEx(SYSEX_DYNAMIXEL_SET_REGISTER, sysexData);
}

void ofArbotix::sendCmdToPic(const char picId, const char picCmd) {

        printf("ofArbotix::setLight() - cmd = %i\n",picCmd);

        int messageLength;
        switch (picCmd) {
        case 0x01 : messageLength = 12; break;
        default : return;
        }
        //sendSysEx
        std::vector < unsigned char> sysexData;
        printf("sysEx.pushBack : %i\n",picId);
        sysexData.push_back(picId);
        printf("sysEx.pushBack : %i\n",messageLength+3);
        sysexData.push_back(messageLength+3);
        printf("sysEx.pushBack : %i\n",0x03);
        sysexData.push_back(0x03);
        printf("sysEx.pushBack : %i\n",picCmd);
        sysexData.push_back(picCmd);
        for (int i=0;i<messageLength;i++)
        {
            printf("sysEx.pushBack : %i\n",picMessage[i]);
            sysexData.push_back(picMessage[i]);
        }

        // calc checksum
        unsigned int checksum = picId + messageLength + 3 + 0x03 + picCmd;
        for ( int i = 0 ; i<messageLength ;i++ )
        {
            checksum +=picMessage[i];
        }

        while (checksum>=256)
        {
            checksum-=256;
        }
        printf("sysEx.pushBack : %i\n",checksum);
        sysexData.push_back(checksum);
        //sendSysEx
        this->sendSysEx(SYSEX_SET_LIGHT, sysexData);
}

//Transmits the command to get a byte of the servo register.
void ofArbotix::sendDynamixelGetRegister(unsigned char servo, unsigned char reg, unsigned char length) {
	int checksum = (~(servo + reg + length)) & 0xFF;

	//Send a sysex to let the arbotix know that we are starting a new synch move command
	std::vector<unsigned char> sysexData;
	sysexData.push_back(servo);
	sysexData.push_back(reg);
	sysexData.push_back(length);
	sysexData.push_back(checksum);
	this->sendSysEx(SYSEX_DYNAMIXEL_GET_REGISTER, sysexData);
}

//Transmits the command to get a byte of the servo register.
void ofArbotix::sendDynamixelConfigureServo(unsigned char servo, unsigned int cwlimit, unsigned int ccwlimit,
											unsigned int maxtorque, unsigned char delaytime, 
										    unsigned char cwcomplmargin, unsigned char ccwcomplmargin,
											unsigned char cwcomplslope, unsigned char ccwcomplslope) {
	unsigned char cwlimit0 = getLowByte(cwlimit);
	unsigned char cwlimit1 = getHighByte(cwlimit);
	unsigned char ccwlimit0 = getLowByte(ccwlimit);
	unsigned char ccwlimit1 = getHighByte(ccwlimit);
	unsigned char maxtorque0 = getLowByte(maxtorque);
	unsigned char maxtorque1 = getHighByte(maxtorque);
	int checksum = (~(servo + cwlimit0 + cwlimit1 + ccwlimit0 + ccwlimit1 + maxtorque0 + maxtorque1 + delaytime + cwcomplmargin + ccwcomplmargin + cwcomplslope + ccwcomplslope)) & 0xFF;

	//Send a sysex to let the arbotix know that we are starting a new synch move command
	std::vector<unsigned char> sysexData;
	sysexData.push_back(servo);
	sysexData.push_back(cwlimit0);
	sysexData.push_back(cwlimit1);
	sysexData.push_back(ccwlimit0);
	sysexData.push_back(ccwlimit1);
	sysexData.push_back(maxtorque0);
	sysexData.push_back(maxtorque1);
	sysexData.push_back(delaytime);
	sysexData.push_back(cwcomplmargin);
	sysexData.push_back(ccwcomplmargin);
	sysexData.push_back(cwcomplslope);
	sysexData.push_back(ccwcomplslope);
	sysexData.push_back(checksum);
	this->sendSysEx(SYSEX_DYNAMIXEL_CONFIGURE_SERVO, sysexData);
}

//Tells an arbotix board to monitor a given servo and report back when it stops moving
void ofArbotix::sendDynamixelStopped(unsigned char servo) {
	//Send a sysex to report a dynamixel servo
	std::vector<unsigned char> sysexData;
	sysexData.push_back(servo);
	sysexData.push_back(1);
	this->sendSysEx(SYSEX_DYNAMIXEL_STOPPED, sysexData);
}

void ofArbotix::checkIncomingSysExMessage(unsigned char cmd) {
	if(_waitingForSysExMessage >= 0 && _waitingForSysExMessage == cmd)
		_sysExMessageFound = true;
}

bool ofArbotix::waitForSysExMessage(unsigned char cmd, unsigned int timeout_sec) {
	_waitingForSysExMessage = cmd;
	_sysExMessageFound = false;
	boost::timer _Timer;

	//Process messages until we find the message we are waiting on or we timeout
	while(!_sysExMessageFound) {

		//Check if we need to timeout
		if(timeout_sec> 0 && _Timer.elapsed()> timeout_sec)
			return false;

		update();
	}

	_waitingForSysExMessage = -1;
	_sysExMessageFound = false;
	return true;
}
