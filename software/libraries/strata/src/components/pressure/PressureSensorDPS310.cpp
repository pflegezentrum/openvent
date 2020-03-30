/**
 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#include "PressureSensorDPS310.hpp"

#include <components/exception/ESensor.hpp>
#include <common/BinUtils.hpp>

namespace
{
    constexpr uint8_t FIFO_STS = 0x0B;
    constexpr uint8_t RESET = 0x0C;

    constexpr uint8_t PRS_SHIFT_EN = (1u << 2);
    constexpr uint8_t TMP_SHIFT_EN = (1u << 3);

    constexpr uint8_t INT_HL = (1u << 7);
    constexpr uint8_t INT_PRS = (1u << 4);
    constexpr uint8_t INT_TMP = (1u << 5);
    constexpr uint8_t INT_FIFO = (1u << 6);

    constexpr uint8_t COEF_SRCE = 0x28;
    constexpr uint8_t TMP_COEF_SRCE_mask = (1u << 7);
    constexpr uint8_t COEF = 0x10;
    constexpr uint8_t COEF_count = 18;
}

PressureSensorDPS310::PressureSensorDPS310(uint8_t devAddr, II2c *access) :
    DpsCommon(devAddr, access, FIFO_STS, RESET)
{
}

PressureSensorDPS310::~PressureSensorDPS310()
{
}

sr_t PressureSensorDPS310::loadCalibration()
{
    m_tempSensor = m_registers.readRegister(COEF_SRCE) & TMP_COEF_SRCE_mask;

    readCoefficients();
}

sr_t PressureSensorDPS310::configureTemperature(Rate oversamplingRate, Rate measurementRate)
{
    DpsCommon::configureTemperature(measurementRate, oversamplingRate);

    const uint8_t val = (oversamplingRate > Rate_8) ? TMP_SHIFT_EN : 0;
    m_registers.modifyRegisterBits(CFG_REG, TMP_SHIFT_EN, val);
}

sr_t PressureSensorDPS310::configurePressure(Rate oversamplingRate, Rate measurementRate)
{
    DpsCommon::configurePressure(measurementRate, oversamplingRate);

    const uint8_t val = (oversamplingRate > Rate_8) ? TMP_SHIFT_EN : 0;
    m_registers.modifyRegisterBits(CFG_REG, PRS_SHIFT_EN, val);
}

float PressureSensorDPS310::calculateTemperature(int32_t raw)
{
    //scale temperature according to scaling table and oversampling
    //update last measured temperature
    //it will be used for pressure compensation
    m_lastTempScal = m_scalingFactorTemperature * raw;

    //Calculate compensated temperature
    return m_c0Half + m_c1 * m_lastTempScal;
}

float PressureSensorDPS310::calculatePressure(int32_t raw)
{
    float p_sc = m_scalingFactorPressure * raw;

    //Calculate compensated pressure
    p_sc = m_c00 + p_sc * (m_c10 + p_sc * (m_c20 + p_sc * m_c30)) + m_lastTempScal * (m_c01 + p_sc * (m_c11 + p_sc * m_c21));
    return p_sc;
}

sr_t PressureSensorDPS310::readCoefficients()
{
    uint8_t buf[COEF_count];
    m_registers.readRegisterBurst(COEF, buf, sizeof(buf));

    //compose coefficients from buf content
    m_c0Half = getTwosComplement<12>((buf[0] << 4) | (buf[1] >> 4));
    //c0 is only used as c0*0.5, so c0_half is calculated immediately
    m_c0Half = m_c0Half / 2U;

    //now do the same thing for all other coefficients
    m_c1 = getTwosComplement<12>(((buf[1] & 0x0F) << 8) | buf[2]);

    m_c00 = getTwosComplement<20>((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
    m_c10 = getTwosComplement<20>(((buf[5] & 0x0Fu) << 16) | (buf[6] << 8) | buf[7]);
    m_c01 = getTwosComplement<16>((buf[8] << 8) | buf[9]);
    m_c11 = getTwosComplement<16>((buf[10] << 8) | buf[11]);
    m_c20 = getTwosComplement<16>((buf[12] << 8) | buf[13]);
    m_c21 = getTwosComplement<16>((buf[14] << 8) | buf[15]);
    m_c30 = getTwosComplement<16>((buf[16] << 8) | buf[17]);
}

sr_t PressureSensorDPS310::configureInterrupts(bool PRS, bool TMP, bool FIFO_FULL)
{
    //Interrupts are not supported with 4 Wire SPI

    if (FIFO_FULL && (PRS || TMP))
    {
        RET_THROW ESensor("Invalid interrupt");
    }

    const uint8_t clearMask = INT_HL | INT_PRS | INT_TMP | INT_FIFO;
    uint8_t setMask = 0;
    if (m_interruptPolarity)
    {
        setMask |= INT_HL;
    }
    if (PRS)
    {
        setMask |= INT_PRS;
    }
    if (TMP)
    {
        setMask |= INT_TMP;
    }
    if (FIFO_FULL)
    {
        setMask |= INT_FIFO;
    }

    m_registers.modifyRegisterBits(CFG_REG, clearMask, setMask);
}

sr_t PressureSensorDPS310::startBothMeasurements(bool continuous)
{
    if (continuous)
    {
        m_bothSingleTemperature = false;
        m_bothSinglePressure = false;
        startMeasurements(CONT_BOTH);
    }
    else
    {
        m_bothSingleTemperature = true;
        m_bothSinglePressure = true;
        startMeasurements(CMD_TMP);
        //delay
        startMeasurements(CMD_PRS);
        m_opMode = CMD_BOTH;
    }
}
