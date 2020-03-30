/**
 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#pragma once

#include <components/interfaces/IRegisters.hpp>


template <typename RegType>
class Registers :
    public IRegisters<RegType>
{
public:
    Registers(RegType increment);
    virtual ~Registers() = default;

    using IRegisters<RegType>::readRegister;
    using IRegisters<RegType>::writeRegister;

	// the following functions should be overridden with optimized implementations for a certain physical interface if possible
    sr_t readRegisterBurst(RegType regAddr, RegType values[], RegType count);
    sr_t writeRegisterBurst(RegType regAddr, const RegType values[], RegType count);

	// the following functions are typically not required to be overridden, but it can still make sense
    sr_t readRegister(RegType regAddr, RegType &value) override;
    sr_t writeRegisterBatch(const RegType regVals[][2], RegType count) override;

    sr_t setRegisterBits(RegType regAddr, RegType bitmask) override;
    sr_t clearRegisterBits(RegType regAddr, RegType bitmask) override;
    sr_t modifyRegisterBits(RegType regAddr, RegType clearBitmask, RegType setBitmask) override;

private:
    RegType m_increment;
};
