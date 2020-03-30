/**
 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#include "Registers.hpp"

#include <components/exception/ERegisters.hpp>


template class Registers<uint8_t>;


template<typename RegType>
Registers<RegType>::Registers(RegType increment) :
    m_increment {increment}
{
}

template <typename RegType>
sr_t Registers<RegType>::readRegisterBurst(RegType regAddr, RegType values[], RegType count)
{
    for (RegType i = 0; i < count; i++)
    {
        THROWS(readRegister(regAddr, values[i]);)
        regAddr += m_increment;
    }

    RETURN
}

template <typename RegType>
sr_t Registers<RegType>::writeRegisterBurst(RegType regAddr, const RegType values[], RegType count)
{
    for (RegType i = 0; i < count; i++)
    {
        THROWS(writeRegister(regAddr, values[i]);)
        regAddr += m_increment;
    }

    RETURN
}

template <typename RegType>
sr_t Registers<RegType>::readRegister(RegType regAddr, RegType &value)
{
    value = readRegister(regAddr);

    RETURN
}

template <typename RegType>
sr_t Registers<RegType>::writeRegisterBatch(const RegType regVals[][2], RegType count)
{
    for (RegType i = 0; i < count; i++)
    {
        THROWS(writeRegister(regVals[i][0], regVals[i][1]);)
    }

    RETURN
}

template <typename RegType>
sr_t Registers<RegType>::setRegisterBits(RegType regAddr, RegType bitmask)
{
    RegType value;
    THROWS(readRegister(regAddr, value);)

    value |= bitmask;

    THROWS(writeRegister(regAddr, value);)

    RETURN
}

template <typename RegType>
sr_t Registers<RegType>::clearRegisterBits(RegType regAddr, RegType bitmask)
{
    RegType value;
    THROWS(readRegister(regAddr, value);)

    value &= ~bitmask;

    THROWS(writeRegister(regAddr, value);)

    RETURN
}

template <typename RegType>
sr_t Registers<RegType>::modifyRegisterBits(RegType regAddr, RegType clearBitmask, RegType setBitmask)
{
    RegType value;
    THROWS(readRegister(regAddr, value);)

    value &= ~clearBitmask;
    value |= setBitmask;

    THROWS(writeRegister(regAddr, value);)

    RETURN
}
