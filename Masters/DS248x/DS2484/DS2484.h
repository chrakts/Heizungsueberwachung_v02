/**********************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
**********************************************************************/


#ifndef OneWire_Masters_DS2484
#define OneWire_Masters_DS2484

#include "Masters/DS248x/DS248x.h"

namespace OneWire
{
    ///DS2484 I2C to 1-wire Master
    class DS2484: public DS248x
    {
        public:
        
        static const uint8_t I2C_ADRS = 0x18;
        
        /// 1-Wire port adjustment parameters.
        /// @note See datasheet page 13.
        enum OwAdjustParam
        {
            tRSTL = 0,
            tRSTL_OD,
            tMSP,
            tMSP_OD,
            tW0L,
            tW0L_OD,
            tREC0, // OD N/A
            RWPU = 8 // OD N/A
        };
        
        /// Default Construct 
        DS2484();
        
        ///Destroys DS2484 object
        virtual ~DS2484(){};
        
        /// Adjust 1-Wire port paramaters.
        /// @note DS2484 only
        /// @param param Parameter to adjust.
        /// @param val New parameter value to set. Consult datasheet for value mappings.
        OneWireMaster::CmdResult adjustOwPort(OwAdjustParam param, uint8_t val);
    };
}

#endif /* OneWire_Masters_DS2484 */