/**
 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#pragma once

#include <common/ErrorHandling.hpp>
#include <cstdint>


template <typename RegType>
class IRegisters
{
public:
    virtual ~IRegisters() = default;

	// the following functions have to be implemented
    virtual RegType readRegister(RegType regAddr) = 0;
    virtual sr_t writeRegister(RegType regAddr, RegType value) = 0;

	// the following functions should be overridden with optimized implementations for a certain physical interface if possible
    virtual sr_t readRegisterBurst(RegType regAddr, RegType values[], RegType count) = 0;
    virtual sr_t writeRegisterBurst(RegType regAddr, const RegType values[], RegType count) = 0;

	// the following functions are typically not required to be overridden, but it can still make sense
    virtual sr_t readRegister(RegType regAddr, RegType &value) = 0;
    virtual sr_t writeRegisterBatch(const RegType regVals[][2], RegType count) = 0;

    virtual sr_t setRegisterBits(RegType regAddr, RegType bitmask) = 0;
    virtual sr_t clearRegisterBits(RegType regAddr, RegType bitmask) = 0;
    virtual sr_t modifyRegisterBits(RegType regAddr, RegType clearBitmask, RegType setBitmask) = 0;
};
