/**
 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#include "RegistersI2c8Bit.hpp"


namespace
{
    constexpr RegType RegIncrement = 1;
}


RegistersI2c8Bit::RegistersI2c8Bit(uint8_t devAddr, II2c *access) :
    Registers(RegIncrement),
    m_devAddr {devAddr},
    m_access {access}
{
}

RegType RegistersI2c8Bit::readRegister(RegType regAddr)
{
    uint8_t value;
    m_access->readI2c8BitRegister(m_devAddr, regAddr, 1, &value);
    return value;
}

sr_t RegistersI2c8Bit::writeRegister(RegType regAddr, RegType value)
{
    THROWS(m_access->writeI2c8BitRegister(m_devAddr, regAddr, 1, &value);)
    RETURN
}
