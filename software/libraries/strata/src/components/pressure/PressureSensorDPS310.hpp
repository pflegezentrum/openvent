/**
 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#pragma once

#include "DpsCommon.hpp"


class PressureSensorDPS310 :
    public DpsCommon
{
public:
    PressureSensorDPS310(uint8_t devAddr = defaultAddress, II2c *access = defaultII2c);
    ~PressureSensorDPS310();

    sr_t configureInterrupts(bool PRS = false, bool TMP = false, bool FIFO_FULL = false);
    sr_t configureTemperature(Rate oversamplingRate, Rate measurementRate) override;
    sr_t configurePressure(Rate oversamplingRate, Rate measurementRate) override;
    sr_t startBothMeasurements(bool continuous = false) override;

protected:
    sr_t loadCalibration() override;
    float calculateTemperature(int32_t raw) override;
    float calculatePressure(int32_t raw) override;

private:
    sr_t readCoefficients();

    //compensation coefficients
    float m_c0Half;
    float m_c1;
};
