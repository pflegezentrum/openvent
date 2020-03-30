/**
 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#pragma once

#include <common/exception/EGenericException.hpp>


class ERegisters :
    public EGenericException
{
public:
    ERegisters(const char desc[] = "Radar Error", int code = 0, const char type[] = "Radar Exception") :
        EGenericException(desc, code, type)
    {}
};
