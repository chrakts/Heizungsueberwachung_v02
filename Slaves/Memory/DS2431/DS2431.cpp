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

#include "Slaves/Memory/DS2431/DS2431.h"

using namespace OneWire;
using namespace OneWire::crc;

enum DS2431_CMDS
{
    WRITE_SCRATCHPAD = 0x0F,
    READ_SCRATCHPAD = 0xAA,
    COPY_SCRATCHPAD = 0x55,
    READ_MEMORY = 0xF0
};

//*********************************************************************
DS2431::DS2431(RandomAccessRomIterator &selector):OneWireSlave(selector)
{
}

//*********************************************************************
OneWireSlave::CmdResult DS2431::writeMemory(uint16_t targetAddress, const uint8_t *data, uint8_t numBytes)
{
    OneWireSlave::CmdResult result = OneWireSlave::OperationFailure;

    if((targetAddress + numBytes) <= 0x88)
    {
        result = OneWireSlave::Success;

        uint8_t startOffset = (targetAddress & 0x0007);
        uint16_t startRowAddress = (targetAddress & 0xFFF8);
        uint8_t endOffset = ((targetAddress + numBytes) & 0x0007);
        uint16_t endRowAddress = ((targetAddress + numBytes) & 0xFFF8);
        const uint8_t *dataIdx = data;

        DS2431::Scratchpad scratchpadData;
        uint8_t esByte;

        if(startOffset != 0)
        {
            result = this->readMemory(startRowAddress, scratchpadData, 8);
            if(result == OneWireSlave::Success)
            {
                memcpy((scratchpadData + startOffset), data, (8 - startOffset));
                result = this->writeScratchpad(startRowAddress, scratchpadData);
                if(result == OneWireSlave::Success)
                {
                    result = this->readScratchpad(scratchpadData , esByte);
                    if(result == OneWireSlave::Success)
                    {
                        result = this->copyScratchpad(startRowAddress, esByte);
                        startRowAddress += 8;
                        dataIdx = (data + (8 - startOffset));
                    }
                }
            }
        }

        if(result == OneWireSlave::Success)
        {
            for(uint16_t row = startRowAddress; row < endRowAddress; row += 8)
            {
                memcpy(scratchpadData, dataIdx, 8);

                result = this->writeScratchpad(row, scratchpadData);
                if(result != OneWireSlave::Success)
                {
                    break;
                }

                result = this->readScratchpad(scratchpadData, esByte);
                if(result != OneWireSlave::Success)
                {
                    break;
                }

                result = this->copyScratchpad(row, esByte);
                if(result != OneWireSlave::Success)
                {
                    break;
                }

                dataIdx += 8;
            }
        }

        if(result == OneWireSlave::Success)
        {
            if(endOffset != 0)
            {
                result = this->readMemory(endRowAddress, scratchpadData, 8);
                if(result == OneWireSlave::Success)
                {
                    memcpy(scratchpadData, dataIdx, endOffset);
                    result = this->writeScratchpad(endRowAddress, scratchpadData);
                    if(result == OneWireSlave::Success)
                    {
                        result = this->readScratchpad(scratchpadData, esByte);
                        if(result == OneWireSlave::Success)
                        {
                            result = this->copyScratchpad(endRowAddress, esByte);
                        }
                    }
                }
            }
        }
    }

    return result;
}

//*********************************************************************
OneWireSlave::CmdResult DS2431::readMemory(uint16_t targetAddress, uint8_t *data, uint8_t numBytes)
{
    OneWireSlave::CmdResult result = OneWireSlave::OperationFailure;

    if((targetAddress + numBytes) <= 0x88)
    {
        OneWireMaster::CmdResult owmResult = selectDevice();
        if(owmResult == OneWireMaster::Success)
        {
            uint8_t sendBlock[] = {READ_MEMORY, (uint8_t)(targetAddress & 0x00FF), (uint8_t)((targetAddress >> 8) & 0x00FF)};
            owmResult = master().OWWriteBlock(sendBlock, 3);
            if(owmResult == OneWireMaster::Success)
            {
                owmResult = master().OWReadBlock(data, numBytes);
                if(owmResult == OneWireMaster::Success)
                {
                    result = OneWireSlave::Success;
                }
            }
        }
    }

    return result;
}

//*********************************************************************
OneWireSlave::CmdResult DS2431::writeScratchpad(uint16_t targetAddress, const Scratchpad &data)
{
    OneWireSlave::CmdResult result = OneWireSlave::OperationFailure;

    OneWireMaster::CmdResult owmResult = selectDevice();
    if(owmResult == OneWireMaster::Success)
    {
        uint8_t sendBlock[11];
        sendBlock[0] = WRITE_SCRATCHPAD;
        sendBlock[1] = (targetAddress &0xFF);
        sendBlock[2] = ((targetAddress >> 8) &0xFF);
        memcpy((sendBlock + 3), data, 8);

        owmResult = master().OWWriteBlock(sendBlock, 11);

        uint16_t invCRC16;
        uint8_t recvbyte;
        master().OWReadByteSetLevel(recvbyte, OneWireMaster::NormalLevel);
        invCRC16 = recvbyte;
        master().OWReadByteSetLevel(recvbyte, OneWireMaster::NormalLevel);
        invCRC16 |= (recvbyte << 8);

        //calc our own inverted CRC16 to compare with one returned
        uint16_t calculatedInvCRC16 = ~calculateCrc16(sendBlock, 0, 11);

        if(invCRC16 == calculatedInvCRC16)
        {
            result = OneWireSlave::Success;
        }
    }

    return result;
}

//*********************************************************************
OneWireSlave::CmdResult DS2431::readScratchpad(Scratchpad &data, uint8_t &esByte)
{
    OneWireSlave::CmdResult result = OneWireSlave::OperationFailure;

    OneWireMaster::CmdResult owmResult = selectDevice();
    if(owmResult == OneWireMaster::Success)
    {
        owmResult = master().OWWriteByteSetLevel(READ_SCRATCHPAD, OneWireMaster::NormalLevel);
        if(owmResult == OneWireMaster::Success)
        {
            uint8_t recvBlock[13];
            owmResult = master().OWReadBlock(recvBlock, 13);

            uint16_t invCRC16 = ((recvBlock[12] << 8) | recvBlock[11]);

            uint8_t idx = 12;
            while(--idx)
            {
                recvBlock[idx] = recvBlock[idx - 1];
            }
            recvBlock[0] = READ_SCRATCHPAD;

            //calc our own inverted CRC16 to compare with one returned
            uint16_t calculatedInvCRC16 = ~calculateCrc16(recvBlock, 0, 12);

            if(invCRC16 == calculatedInvCRC16)
            {
                esByte = recvBlock[3];
                memcpy(data, (recvBlock + 4), 8);
                result = OneWireSlave::Success;
            }
        }
    }

    return result;
}

//*********************************************************************
OneWireSlave::CmdResult DS2431::copyScratchpad(uint16_t targetAddress, uint8_t esByte)
{
    OneWireSlave::CmdResult result = OneWireSlave::OperationFailure;

    OneWireMaster::CmdResult owmResult = selectDevice();
    if(owmResult == OneWireMaster::Success)
    {
        uint8_t sendBlock[] = {COPY_SCRATCHPAD, (uint8_t)(targetAddress & 0x00FF), (uint8_t)((targetAddress >> 8) & 0x00FF), esByte};
        owmResult = master().OWWriteBlock(sendBlock, 4);

        master().OWSetLevel(OneWireMaster::StrongLevel);
        _delay_ms(10);
        master().OWSetLevel(OneWireMaster::NormalLevel);

        uint8_t check;
        master().OWReadByteSetLevel(check, OneWireMaster::NormalLevel);
        if(check == 0xAA)
        {
            result = OneWireSlave::Success;
        }
    }

    return result;
}
