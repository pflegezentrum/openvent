/**
 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#pragma once

#include <components/interfaces/IPressureSensor.hpp>
#include <components/interfaces/ITemperatureSensor.hpp>
#include <components/RegistersI2c8Bit.hpp>


class DpsCommon :
        public IPressureSensor,
        public ITemperatureSensor
{
public:
    static const uint8_t defaultAddress = 0x77;
    static const uint8_t alternativeAddress = 0x76;

    enum Mode : uint8_t
    {
        IDLE = 0x00,
        CMD_PRS = 0x01,
        CMD_TMP = 0x02,
        CMD_BOTH = 0x03,
        CONT_PRS = 0x05,
        CONT_TMP = 0x06,
        CONT_BOTH = 0x07
    };

    enum Rate : uint8_t
    {
        Rate_1 = 0,
        Rate_2 = 1,
        Rate_4 = 2,
        Rate_8 = 3,
        Rate_16 = 4,
        Rate_32 = 5,
        Rate_64 = 6,
        Rate_128 = 7
    };

    enum ValueType : uint8_t
    {
        Temperature = 0,
        Pressure = 1
    };

    DpsCommon(uint8_t devAddr, II2c *access, uint8_t FIFO_STS, uint8_t RESET);
    ~DpsCommon();

    sr_t initialize();

    virtual sr_t configureTemperature(Rate oversamplingRate, Rate measurementRate = Rate_1);
    virtual sr_t configurePressure(Rate oversamplingRate, Rate measurementRate = Rate_1);

    sr_t startTemperatureMeasurements(bool continuous = false);
    sr_t startPressureMeasurements(bool continuous = false);
    virtual sr_t startBothMeasurements(bool continuous = false) = 0;
    sr_t stopMeasurements();

    float getTemperature();
    float getPressure();
    sr_t getFifoMeasurement(float &value, ValueType &type);
    sr_t getFifoMeasurements(float tmpBuffer[], uint8_t tmpCount, float prsBuffer[], uint8_t prsCount);

    sr_t softReset();

    sr_t flushFifo();
    sr_t enableFifo();
    sr_t disableFifo();

    sr_t clearInterrupts();

protected:
    static constexpr uint8_t MEAS_CFG = 0x08;
    static constexpr uint8_t CFG_REG = 0x09;
    const uint8_t m_FIFO_STS;
    const uint8_t m_RESET;

    uint8_t m_tempSensor;

    virtual sr_t loadCalibration() = 0;
    virtual float calculateTemperature(int32_t raw) = 0;
    virtual float calculatePressure(int32_t raw) = 0;

    sr_t fuseBitFixTemperature();
    sr_t setOpMode(Mode mode);
    sr_t startMeasurements(Mode mode);
    int32_t getRawMeasurementSingle(uint8_t regBlock, uint8_t readyBit, bool waitUntilReady = true);
    int32_t getRawMeasurement(uint8_t regBlock);

    constexpr inline unsigned int calculateMeasurementTime(Rate oversamplingRate)
    {
        const unsigned int overhead = 2; // ms
        const unsigned int single = 16; // 1/10 ms
        return overhead + ((1 << oversamplingRate) * single + 9) / 10; // rounded to ms
    }

    RegistersI2c8Bit m_registers;

    bool m_initialized;
    bool m_interruptPolarity;
    Mode m_opMode;

    float m_scalingFactorTemperature;
    float m_scalingFactorPressure;

    // compensation coefficients for both dps310 and dps422
    float m_c00;
    float m_c10;
    float m_c01;
    float m_c11;
    float m_c20;
    float m_c21;
    float m_c30;

    // last measured scaled temperature (necessary for pressure compensation)
    float m_lastTempScal;

    bool m_bothSingleTemperature;
    bool m_bothSinglePressure;
};
