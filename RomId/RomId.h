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

#ifndef OneWire_RomId
#define OneWire_RomId

#include <string.h>
#include "Utilities/array.h"
#include "Utilities/crc.h"

namespace OneWire
{
  /// Standard container for a 1-Wire ROM ID.
  class RomId
  {
  public:
      /// Length of the buffer in bytes.
      static const size_t byteLen = 8;
      
      /// Built-in array representation.
      typedef array<uint8_t, byteLen>::Buffer ByteBuffer;

  private:
      static const size_t familyCodeIdx = 0;
      static const size_t crc8Idx = 7;
      
      /// Default starting value is all bytes 0x00.
      static const uint8_t defaultByteVal = 0x00;

      array<uint8_t, byteLen> m_romId;

  public:
      RomId() { reset(); }
      RomId(const RomId & romId) : m_romId(romId.m_romId) { }
      RomId(const ByteBuffer & romIdBytes) : m_romId(romIdBytes) { }

      const RomId & operator=(const RomId & rhs)
      {
          this->m_romId = rhs.m_romId;
          return rhs;
      }
      
      bool operator==(const RomId & rhs) const
      {
          return (this->m_romId == rhs.m_romId);
      }
      
      bool operator!=(const RomId & rhs) const
      {
          return !operator==(rhs);
      }
      
      /// Conversion to array reference.
      operator ByteBuffer &()
      {
          return m_romId;
      }
      
      /// Conversion to const array reference.
      operator const ByteBuffer &() const
      {
          return m_romId;
      }

      /// Reset to the default starting value.
      void reset()
      {
          memset(m_romId, defaultByteVal, byteLen);
      }

      /// Read the Family Code byte.
      uint8_t familyCode() const
      {
          return m_romId[familyCodeIdx];
      }
      
      /// Set the family code byte.
      void setFamilyCode(uint8_t familyCode)
      {
          m_romId[familyCodeIdx] = familyCode;
      }

      /// Read the CRC8 byte.
      uint8_t crc8() const
      {
          return m_romId[crc8Idx];
      }
      
      /// Set the CRC8 byte.
      void setCrc8(uint8_t crc8)
      {
          m_romId[crc8Idx] = crc8;
      }
      
      /// Check if the CRC8 is valid for the ROM ID.
      /// @returns True if the CRC8 is valid.
      bool crc8Valid() const
      {
          return (crc::calculateCrc8(m_romId, (byteLen - 1), 0x00) == crc8());
      }
      
      /// Calculate and set the CRC8 for the ROM ID.
      void setValidCrc8()
      {
          setCrc8(crc::calculateCrc8(m_romId, (byteLen - 1), 0x00));
      }

      /// Check if the ROM ID is valid (Family Code and CRC8 are both valid).
      /// @returns True if the ROM ID is valid.
      bool valid() const
      {
          return (crc8Valid() && (familyCode() != defaultByteVal));
      }
  };
}

#endif
