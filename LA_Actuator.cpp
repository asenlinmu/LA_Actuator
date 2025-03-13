// LA_Actuator.cpp
#include "LA_Actuator.h"

LAActuator::LAActuator(HardwareSerial &ser, uint8_t id, uint32_t baud)
    : _serial(ser), _id(id), _baud(baud), _timeout(50) {}

void LAActuator::begin() {
    _serial.begin(_baud);
    while (!_serial){
		delay(10u);
	}
}

bool LAActuator::setID(uint8_t newID) {
    uint8_t data[] = {0x02, newID};
    if (!sendCommand(CMD_WR, 0x02, data, sizeof(data))) return false;
    
    // Verify ID change
    _id = newID;
    ActuatorStatus status;
    return getStatus(status);
}

bool LAActuator::setTargetPosition(uint16_t position, bool feedback) {
    uint8_t cmd = feedback ? CMD_WR_DRV_FB : CMD_WR_DRV;
    uint8_t posData[] = {lowByte(position), highByte(position)};
    return sendCommand(cmd, 0x37, posData, sizeof(posData));
}

bool LAActuator::emergencyStop() {
	uint8_t data[] = {0x23};
    return sendCommand(CMD_MC, 0x00, data, 1);
}

bool LAActuator::resumeWork() {
	uint8_t data[] = {0x04};
    return sendCommand(CMD_MC, 0x00, data, 1);
}

bool LAActuator::saveParameters() {
	uint8_t data[] = {0x20};
    return sendCommand(CMD_MC, 0x00, data, 1);
}

bool LAActuator::clearFaults() {
	uint8_t data[] = {0x1E};
    return sendCommand(CMD_MC, 0x00, data, 1);
}

bool LAActuator::getStatus(ActuatorStatus &status) {
    uint8_t buffer[21];
	uint8_t data[] = {0x22};
    if (!sendCommand(CMD_MC, 0x00, data, 1) || 
        !receiveResponse(CMD_MC, buffer, sizeof(buffer))) {
        return false;
    }

    status.target_pos = buffer[7] | (buffer[8] << 8);
    status.current_pos = buffer[9] | (buffer[10] << 8);
    status.temperature = buffer[11];
    status.current_ma = buffer[12] | (buffer[13] << 8);
    status.force_sensor = buffer[14] | (buffer[15] << 8);
    status.error_flags = buffer[16];
    return true;
}

// Private methods implementation
bool LAActuator::sendCommand(uint8_t cmd, uint8_t index, const uint8_t *data, uint8_t len) {
    uint8_t frame[16] = {0x55, 0xAA};
    uint8_t frameLen = 6 + len; // Header(2) + Len(1) + ID(1) + CMD(1) + Index(1) + Data + Checksum(1)
    
    frame[2] = len + 2;         // Length
    frame[3] = _id;             // ID
    frame[4] = cmd;             // Command
    frame[5] = index;           // Index
    
    if(data && len > 0) {
        memcpy(&frame[6], data, len);
    }
    
    frame[frameLen] = calculateChecksum(&frame[2], 4 + len);
    
    _serial.write(frame, 7 + len);
    _serial.flush();
    return true;
}

bool LAActuator::receiveResponse(uint8_t cmd, uint8_t *buffer, uint8_t len) {
    uint32_t start = millis();
    uint8_t idx = 0;
    
    while(millis() - start < _timeout) {
        if(_serial.available()) {
            buffer[idx++] = _serial.read();
            if(idx == len) {
                return validateResponse(buffer, len);
            }
        }
    }
    return false;
}

uint8_t LAActuator::calculateChecksum(const uint8_t *data, uint8_t len) {
    uint16_t sum = 0;
    for(uint8_t i=0; i<len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

bool LAActuator::validateResponse(const uint8_t *frame, uint8_t len) {
    if(frame[0] != 0xAA || frame[1] != 0x55) return false;
    if(frame[3] != _id) return false;
    
    uint8_t expectedSum = calculateChecksum(&frame[2], len-3);
    return (frame[len-1] == expectedSum);
}

// Temperature and current protection settings
bool LAActuator::setOverTempProtection(float temp) {
    uint16_t value = temp * 10;
    uint8_t data[] = {lowByte(value), highByte(value)};
    return sendCommand(CMD_WR, 0x62, data, sizeof(data));
}

bool LAActuator::setRecoveryTemp(float temp) {
    uint16_t value = temp * 10;
    uint8_t data[] = {lowByte(value), highByte(value)};
    return sendCommand(CMD_WR, 0x64, data, sizeof(data));
}

bool LAActuator::setOverCurrent(uint16_t ma) {
    uint8_t data[] = {lowByte(ma), highByte(ma)};
    return sendCommand(CMD_WR, 0x20, data, sizeof(data));
}