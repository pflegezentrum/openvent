/**
 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#pragma once

#include "Registers.hpp"
#include <platform/interfaces/access/II2c.hpp>


using RegType = uint8_t;


class RegistersI2c8Bit :
    public Registers<RegType>
{
public:
    RegistersI2c8Bit(uint8_t devAddr, II2c *access);
    virtual ~RegistersI2c8Bit() = default;

    // IRegisters
    RegType readRegister(RegType regAddr) override;
    sr_t writeRegister(RegType regAddr, RegType value) override;

private:
    const uint8_t m_devAddr;
    II2c *m_access;
};
