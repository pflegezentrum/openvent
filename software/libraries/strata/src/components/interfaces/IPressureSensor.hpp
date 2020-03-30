/**
 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#pragma once

//namespace
class IPressureSensor
{
public:
    virtual ~IPressureSensor() = default;

    /**
     * The measurement is in Celsius.
     */
    virtual float getPressure() = 0;
};
