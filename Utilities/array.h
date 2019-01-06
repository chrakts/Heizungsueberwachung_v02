/******************************************************************//**
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

#ifndef OneWire_array
#define OneWire_array

#include <stdint.h>
#include <stddef.h>

namespace OneWire
{
    /// Generic array class similar to std::array.
    template <typename T, size_t N>
    class array
    {
    public:
        /// Number of elements contained in the array.
        static const size_t length = N;

        /// Built-in array representation.
        typedef T Buffer[N];

    private:
        Buffer m_buffer;

    public:
        const array<T, N> & operator=(const array<T, N> & rhs)
        {
            if (this != &rhs)
            {
                memcpy(this->m_buffer, rhs.m_buffer, N * sizeof(T));
            }
            return rhs;
        }

        bool operator==(const array<T, N> & rhs) const
        {
            return (memcmp(this->m_buffer, rhs.m_buffer, N * sizeof(T)) == 0);
        }

        bool operator!=(const array<T, N> & rhs) const
        {
            return !operator==(rhs);
        }

        /// Conversion to array reference.
        operator Buffer &()
        {
            return m_buffer;
        }

        /// Conversion to const array reference.
        operator const Buffer &() const
        {
            return m_buffer;
        }

        array<T, N>() { }

        array<T, N>(const array<T, N> & copy)
        {
            operator=(copy);
        }

        array<T, N>(const Buffer & buffer)
        {
            memcpy(m_buffer, buffer, N * sizeof(T));
        }
    };
}

#endif
