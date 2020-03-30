/**
 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#include "DpsCommon.hpp"

#include <components/exception/ESensor.hpp>
#include <common/BinUtils.hpp>

namespace
{
    // since the factors are powers of two, their inverse can be exactly represented in float
    const float scalingFactors[] =
    {
            1.0f / 524288,
            1.0f / 1572864,
            1.0f / 3670016,
            1.0f / 7864320,
            1.0f / 253952,
            1.0f / 516096,
            1.0f / 1040384,
            1.0f / 2088960
    };

    constexpr uint8_t _B_count = 3;
    constexpr uint8_t PRS_B = 0x00;
    constexpr uint8_t TMP_B = 0x03;
    constexpr uint8_t PRS_CFG = 0x06;
    constexpr uint8_t TMP_CFG = 0x07;
    constexpr uint8_t INT_STS = 0x0A;

    constexpr uint8_t FIFO_EN = (1u << 1);

    constexpr uint8_t PRS_RDY = (1u << 4);
    constexpr uint8_t TMP_RDY = (1u << 5);

    constexpr uint8_t INT_HL = (1u << 7);
    constexpr uint8_t INT_PRS = (1u << 4);
    constexpr uint8_t INT_TMP = (1u << 5);

    constexpr uint8_t FIFO_EMPTY = (1u << 0);
    constexpr uint8_t FIFO_FULL = (1u << 1);


    constexpr uint8_t COEF = 0x10;
    constexpr uint8_t COEF_count = 18;

    constexpr uint8_t FIFO_FLUSH = (1u << 7);
    constexpr uint8_t SOFT_RESET = 0x09;

    constexpr uint8_t TMP_EXT = (1u << 7);
}


DpsCommon::DpsCommon(uint8_t devAddr, II2c *access, uint8_t FIFO_STS, uint8_t RESET) :
    m_FIFO_STS {FIFO_STS},
    m_RESET {RESET},
    m_tempSensor {TMP_EXT},
    m_registers(devAddr, access),
    m_initialized {false},
    m_interruptPolarity {(devAddr & 1) == 0},
    m_opMode {IDLE},
    m_bothSingleTemperature {false},
    m_bothSinglePressure {false}
{
}

DpsCommon::~DpsCommon()
{
}

sr_t DpsCommon::initialize()
{
    stopMeasurements();

    if (!m_initialized)
    {
//        softReset();
//        delay(3 + 12 + 40);

        // wait until sensor is ready
        while ((m_registers.readRegister(MEAS_CFG) & 0xC0) != 0xC0);

        loadCalibration();

        m_initialized = true;
    }

    configureTemperature(Rate_4, Rate_8);
    configurePressure(Rate_4, Rate_8);

    getTemperature(); // update m_lastTempScal for pressure compensation
    fuseBitFixTemperature(); // Fix for ICs with a fuse bit problem leading to a wrong temperature. (should not affect ICs without this problem)

    //clearInterrupts();
}

sr_t DpsCommon::configureTemperature(Rate oversamplingRate, Rate measurementRate)
{
    const uint8_t value = m_tempSensor | (measurementRate << 4) | oversamplingRate;
    m_registers.writeRegister(TMP_CFG, value);
    m_scalingFactorTemperature = scalingFactors[oversamplingRate];
}

sr_t DpsCommon::configurePressure(Rate oversamplingRate, Rate measurementRate)
{
    const uint8_t value = (measurementRate << 4) | oversamplingRate;
    m_registers.writeRegister(PRS_CFG, value);
    m_scalingFactorPressure = scalingFactors[oversamplingRate];
}

sr_t DpsCommon::startTemperatureMeasurements(bool continuous)
{
    m_bothSingleTemperature = false;
    startMeasurements(continuous?CONT_TMP:CMD_TMP);
}

sr_t DpsCommon::startPressureMeasurements(bool continuous)
{
    m_bothSinglePressure = false;
    startMeasurements(continuous?CONT_PRS:CMD_PRS);
}

sr_t DpsCommon::stopMeasurements()
{
    setOpMode(IDLE);
    //disableFifo();
}

float DpsCommon::getTemperature()
{
    if (m_opMode == CONT_TMP)
    {
        clearInterrupts();
        const int32_t rawTemp = getRawMeasurement(TMP_B);
        return calculateTemperature(rawTemp);
    }
    else
    {
        if (m_bothSingleTemperature)
        {
            m_bothSingleTemperature = false;
        }
        else if (m_opMode != CMD_TMP)
        {
            startMeasurements(CMD_TMP);
        }

        const int32_t rawTemp = getRawMeasurementSingle(TMP_B, TMP_RDY);
        return calculateTemperature(rawTemp);
    }
}

int32_t DpsCommon::getRawMeasurementSingle(uint8_t regBlock, uint8_t readyBit, bool waitUntilReady)
{
    const bool ready = m_registers.readRegister(MEAS_CFG) & readyBit;
    if (!ready)
    {
        if (waitUntilReady)
        {
            while(!(m_registers.readRegister(MEAS_CFG) & readyBit));
        }
        else
        {
            RET_THROW ESensor("Measurement not ready");
        }
    }

    m_opMode = IDLE;
    return getRawMeasurement(regBlock);
}

int32_t DpsCommon::getRawMeasurement(uint8_t regBlock)
{
    uint8_t buf[_B_count];
    m_registers.readRegisterBurst(regBlock, buf, sizeof(buf));
    return getTwosComplement<24>((buf[0] << 16) | (buf[1] << 8) | buf[2]);
}

sr_t DpsCommon::startMeasurements(Mode mode)
{
    if (!m_initialized)
    {
        RET_THROW ESensor("Not initialized");
    }

    if (m_opMode != IDLE)
    {
        RET_THROW ESensor("Sensor busy");
    }

    const bool continuous = (mode > 4);
    if (continuous)
    {
        //enableFifo();
    }

    setOpMode(mode);
}

float DpsCommon::calculatePressure(int32_t raw)
{
    float p_sc = m_scalingFactorPressure * raw;

    //Calculate compensated pressure
    p_sc = m_c00 + p_sc * (m_c10 + p_sc * (m_c20 + p_sc * m_c30)) + m_lastTempScal * (m_c01 + p_sc * (m_c11 + p_sc * m_c21));
    return p_sc;
}

float DpsCommon::getPressure()
{
    if (m_opMode == CONT_PRS)
    {
        clearInterrupts();
        const int32_t rawPressure = getRawMeasurement(PRS_B);
        return calculatePressure(rawPressure);
    }
    else
    {
        if (m_bothSinglePressure)
        {
            m_bothSinglePressure = false;
        }
        else if (m_opMode != CMD_PRS)
        {
            startMeasurements(CMD_PRS);
        }

        const int32_t rawPressure = getRawMeasurementSingle(PRS_B, PRS_RDY);
        return calculatePressure(rawPressure);
    }
}

sr_t DpsCommon::setOpMode(Mode mode)
{
    m_registers.modifyRegisterBits(MEAS_CFG, 0x07, mode);
    m_opMode = mode;
}

sr_t DpsCommon::enableFifo()
{
    m_registers.setRegisterBits(CFG_REG, FIFO_EN);
}

sr_t DpsCommon::disableFifo()
{
    flushFifo();
    m_registers.clearRegisterBits(CFG_REG, FIFO_EN);
}

sr_t DpsCommon::fuseBitFixTemperature()
{
    m_registers.writeRegister(0x0E, 0xA5);
    m_registers.writeRegister(0x0F, 0x96);
    m_registers.writeRegister(0x62, 0x02);
    m_registers.writeRegister(0x0E, 0x00);
    m_registers.writeRegister(0x0F, 0x00);

    //perform a first temperature measurement (again)
    //the most recent temperature will be saved internally
    //and used for compensation when calculating pressure
    getTemperature();
}

sr_t DpsCommon::clearInterrupts()
{
    m_registers.readRegister(INT_STS);
}

sr_t DpsCommon::softReset()
{
    m_registers.writeRegister(m_RESET, SOFT_RESET);
}

sr_t DpsCommon::flushFifo()
{
    m_registers.writeRegister(m_RESET, FIFO_FLUSH);
}

sr_t DpsCommon::getFifoMeasurements(float tmpBuffer[], uint8_t tmpCount, float prsBuffer[], uint8_t prsCount)
{
    const auto tmpMax = tmpCount;
    const auto prsMax = prsCount;
    tmpCount = 0;
    prsCount = 0;

    while ((m_registers.readRegister(m_FIFO_STS) & FIFO_EMPTY) == 0)
    {
        const auto raw = getRawMeasurement(PRS_B);
        if (raw & 1)
        {
            if (prsCount == prsMax)
            {
                RET_THROW ESensor("prs buffer full");
            }
            prsBuffer[prsCount++] = calculatePressure(raw);
        }
        else
        {
          if (tmpCount == tmpMax)
          {
              RET_THROW ESensor("tmp buffer full");
          }
          tmpBuffer[tmpCount++] = calculateTemperature(raw);
      }
    }
}

sr_t DpsCommon::getFifoMeasurement(float& value, ValueType &type)
{
    if (m_registers.readRegister(m_FIFO_STS) & FIFO_EMPTY)
    {
        RET_THROW ESensor("FIFO Empty");
    }
    else
    {
        const auto raw = getRawMeasurement(PRS_B);
        if (raw & 1)
        {
            value = calculatePressure(raw);
            type = Temperature;
        }
        else
        {
            value = calculateTemperature(raw);
            type = Pressure;
        }
    }

    RETURN
}
