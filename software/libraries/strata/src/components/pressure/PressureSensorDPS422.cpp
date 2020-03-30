/**
 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#include "PressureSensorDPS422.hpp"

#include <components/exception/ESensor.hpp>
#include <common/BinUtils.hpp>

namespace
{
    constexpr uint8_t WM_CFG = 0x0B;
    constexpr uint8_t WM_LEVEL_mask = 0x1F;
    constexpr uint8_t FIFO_STS = 0x0C;
    constexpr uint8_t RESET = 0x0D;

    constexpr uint8_t INT_HL = (1u << 3);
    constexpr uint8_t INT_PRS = (1u << 4);
    constexpr uint8_t INT_TMP = (1u << 5);
    constexpr uint8_t INT_WM = (1u << 6);
    constexpr uint8_t INT_FIFO = (1u << 7);

    constexpr uint8_t COEF_TEMP = 0x20;
    constexpr uint8_t COEF_TEMP_count = 3;
    constexpr uint8_t COEF_PRS = 0x26;
    constexpr uint8_t COEF_PRS_count = 20;


    const float DPS422_T_REF = 27;
    const float DPS422_V_BE_TARGET = 0.687027;
    const float DPS422_ALPHA = 9.45;
    const float DPS422_T_C_VBE = -1.735e-3;
    const float DPS422_K_PTAT_CORNER = -0.8;
    const float DPS422_K_PTAT_CURVATURE = 0.039;
    const float DPS422_A_0 = 5030;
}

PressureSensorDPS422::PressureSensorDPS422(uint8_t devAddr, II2c *access) :
        DpsCommon(devAddr, access, FIFO_STS, RESET)
{
}

PressureSensorDPS422::~PressureSensorDPS422()
{
}

sr_t PressureSensorDPS422::loadCalibration()
{
    // m_lastTempScal = 0.08716583251; // in case temperature reading disabled, the default raw temperature value correspond the reference temperature of 27 degress.

    readCoefficients();
}

float PressureSensorDPS422::calculateTemperature(int32_t raw)
{
    const float factor = 1.0f / 1048576;
    m_lastTempScal = factor * raw;
    const float u = m_lastTempScal / (1 + DPS422_ALPHA * m_lastTempScal);
    return (a_prime * u + b_prime);
}

float PressureSensorDPS422::calculatePressure(int32_t raw)
{
    float prs = m_scalingFactorPressure * raw;

    const float temp = (8.5 * m_lastTempScal) / (1 + 8.8 * m_lastTempScal);

    prs = m_c00 + m_c10 * prs + m_c01 * temp + m_c20 * prs * prs + m_c02 * temp * temp + m_c30 * prs * prs * prs +
          m_c11 * temp * prs + m_c12 * prs * temp * temp + m_c21 * prs * prs * temp;
    return prs;
}

sr_t PressureSensorDPS422::startBothMeasurements(bool continuous)
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
        startMeasurements(CMD_BOTH);
    }
}

sr_t PressureSensorDPS422::readCoefficients()
{
    uint8_t buf_tmp[COEF_TEMP_count];
    uint8_t buf_prs[COEF_PRS_count];
    m_registers.readRegisterBurst(COEF_TEMP, buf_tmp, sizeof(buf_tmp));
    m_registers.readRegisterBurst(COEF_PRS, buf_prs, sizeof(buf_prs));

    // refer to datasheet
    // 1. read T_Vbe, T_dVbe and T_gain
    const int t_gain = getTwosComplement<8>(buf_tmp[0]);
    const int t_dVbe = getTwosComplement<7>(buf_tmp[1] >> 1);
    const int t_Vbe = getTwosComplement<9>((buf_tmp[2] << 1) | (buf_tmp[1] & 0x01));

    // 2. Vbe, dVbe and Aadc
    const float Vbe = 1.05031e-4 * t_Vbe + 0.463232422;
    const float dVbe = 1.25885e-5 * t_dVbe + 0.04027621;
    const float Aadc = 8.4375e-5 * t_gain + 0.675;

    // 3. Vbe_cal and dVbe_cal
    float Vbe_cal = Vbe / Aadc;
    float dVbe_cal = dVbe / Aadc;

    // 4. T_calib
    const float T_calib = DPS422_A_0 * dVbe_cal - 273.15;
    // 5. Vbe_cal(T_ref): Vbe value at reference temperature
    const float Vbe_cal_tref = Vbe_cal - (T_calib - DPS422_T_REF) * DPS422_T_C_VBE;
    // 6. calculate PTAT correction coefficient
    const float k_ptat = (DPS422_V_BE_TARGET - Vbe_cal_tref) * DPS422_K_PTAT_CORNER + DPS422_K_PTAT_CURVATURE;
    // 7. calculate A' and B'
    a_prime = DPS422_A_0 * (Vbe_cal + DPS422_ALPHA * dVbe_cal) * (1 + k_ptat);
    b_prime = -273.15 * (1 + k_ptat) - k_ptat * T_calib;

    // c00, c01, c02, c10 : 20 bits
    // c11, c12: 17 bits
    // c20: 15 bits; c21: 14 bits; c30 12 bits
    m_c00 = getTwosComplement<20>((buf_prs[0] << 12) | (buf_prs[1] << 4) | (buf_prs[2] >> 4));
    m_c10 = getTwosComplement<20>(((buf_prs[2] & 0x0Fu) << 16) | (buf_prs[3] << 8) | buf_prs[4]);

    m_c01 = getTwosComplement<20>((buf_prs[5] << 12) | (buf_prs[6] << 4) | (buf_prs[7] >> 4));
    m_c02 = getTwosComplement<20>(((buf_prs[7] & 0x0Fu) << 16) | (buf_prs[8] << 8) | buf_prs[9]);
    m_c20 = getTwosComplement<15>(((buf_prs[10] & 0x7F) << 8) | buf_prs[11]);
    m_c30 = getTwosComplement<12>(((buf_prs[12] & 0x0F) << 8) | buf_prs[13]);
    m_c11 = getTwosComplement<17>((buf_prs[14] << 9) | (buf_prs[15] << 1) | (buf_prs[16] >> 7));
    m_c12 = getTwosComplement<17>(((buf_prs[16] & 0x7F) << 10) | (buf_prs[17] << 2) | (buf_prs[18] >> 6));
    m_c21 = getTwosComplement<14>(((buf_prs[18] & 0x3F) << 8) | (buf_prs[19]));
}

sr_t PressureSensorDPS422::configureInterrupts(bool PRS, bool TMP, bool FIFO_FULL)
{
    //Interrupts are not supported with 4 Wire SPI

    if (FIFO_FULL && (PRS || TMP))
    {
        RET_THROW ESensor("Invalid interrupt");
    }

    const uint8_t clearMask = INT_HL | INT_PRS | INT_TMP | INT_FIFO | INT_WM;
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

sr_t PressureSensorDPS422::configureWatermarkInterrupt(uint8_t level)
{
    //Interrupts are not supported with 4 Wire SPI
    m_registers.writeRegister(WM_CFG, level & WM_LEVEL_mask);

    const uint8_t clearMask = INT_HL | INT_PRS | INT_TMP | INT_FIFO;
    const uint8_t setMask = INT_WM;

    m_registers.modifyRegisterBits(CFG_REG, clearMask, setMask);
}

