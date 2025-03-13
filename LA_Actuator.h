// LA_Actuator.h
#ifndef LA_Actuator_h
#define LA_Actuator_h

#include <Arduino.h>

class LAActuator {
public:
    typedef struct {
        uint16_t target_pos;
        int16_t current_pos;
        int8_t temperature;
        uint16_t current_ma;
        int16_t force_sensor;
        uint8_t error_flags;
    } ActuatorStatus;

    LAActuator(HardwareSerial &ser, uint8_t id = 1, uint32_t baud = 921600);
    
    void begin();
    bool setID(uint8_t newID);
    bool setTargetPosition(uint16_t position, bool feedback = true);
    bool emergencyStop();
    bool resumeWork();
    bool saveParameters();
    bool clearFaults();
    bool getStatus(ActuatorStatus &status);
    bool setOverTempProtection(float temp);
    bool setRecoveryTemp(float temp);
    bool setOverCurrent(uint16_t ma);

private:
    HardwareSerial &_serial;
    uint8_t _id;
    uint32_t _baud;
    uint8_t _timeout;

    enum CMD_TYPE {
        CMD_RD = 0x01,
        CMD_WR = 0x02,
        CMD_MC = 0x04,
        CMD_WR_DRV_FB = 0x21,
        CMD_WR_DRV = 0x03
    };

    bool sendCommand(uint8_t cmd, uint8_t index, const uint8_t *data = nullptr, uint8_t len = 0);
    bool receiveResponse(uint8_t cmd, uint8_t *buffer, uint8_t len);
    uint8_t calculateChecksum(const uint8_t *data, uint8_t len);
    bool validateResponse(const uint8_t *frame, uint8_t len);
};

#endif