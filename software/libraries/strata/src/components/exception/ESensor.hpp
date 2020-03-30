/**
 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#pragma once

#include <common/exception/EGenericException.hpp>


class ESensor :
    public EGenericException
{
public:
    ESensor(const char desc[] = "Sensor Error", int code = 0, const char type[] = "Sensor Exception") :
        EGenericException(desc, code, type)
    {}

    EXCEPTION_CODE(2)
};
