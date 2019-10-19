/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"

//#include "io/serial.h"

#if defined(USE_OPFLOW_PMW3901)
// #include "drivers/io.h"
#include "drivers/bus.h"
#include "drivers/opflow/opflow_pmw3901.h"
#include "drivers/opflow/opflow_virtual.h"
#include "drivers/time.h"
#include "io/opflow.h"

//#define PMW3901_PACKET_SIZE 9

#define PMW3901_REG_RESET 0x3A
#define PMW3901_REG_RESET_SET_VALUE 0x5A
#define PMW3901_REG_CHIPID 0x00
#define PMW3901_REG_CHIPID_READ_VALUE 0x49
#define PMW3901_REG_DIPIHC 0x5F
#define PMW3901_REG_DIPIHC_READ_VALUE 0xB8


// Port Define -----------------------------------------------
//#define LOW 0x0
//#define HIGH 0x1

//#define SPI                      SPI3
#define SPI_BAUDRATE_2MHZ   SPI_BaudRatePrescaler_64    // 1.3125MHz

// static bool isInit = false;

// typedef struct __attribute__((packed))
// {
//     uint8_t header; // 0xFE
//     uint8_t res0;   // Seems to be 0x04 all the time
//     int16_t motionX;
//     int16_t motionY;
//     int8_t motionT; // ???
//     uint8_t squal;  // Not sure about this
//     uint8_t footer; // 0xAA
// } pmw3901Packet_t;

// SPI Func Port -----------------------------------------------------
static void spiConfigureWithSpeed(uint16_t baudRatePrescaler)
{
    //inav : bus_spi.c/spiInitDevice()
    SPI_InitTypeDef SPI_InitStructure;

    SPI_I2S_DeInit(SPI3);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    // SPI_MODE3
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;

    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;

    //inav default :7
    SPI_InitStructure.SPI_CRCPolynomial = 0; // Not used

    SPI_InitStructure.SPI_BaudRatePrescaler = baudRatePrescaler;
    SPI_Init(SPI3, &SPI_InitStructure);
}


static bool PMW3901_Write(const busDevice_t * busDev, uint8_t reg, uint8_t data)
{
    spiBusSelectDevice(busDev);
    delayMicroseconds(50);

    SPI_TypeDef * instance = spiInstanceByDevice(busDev->busdev.spi.spiBus);
    // Set MSB to 1 for write
    reg |= 0x80u;
    spiTransferByte(instance, reg);

    delayMicroseconds(50);
    
    //recive data
    spiTransferByte(instance, data);

    delayMicroseconds(50);
    spiBusDeselectDevice(busDev);
    delayMicroseconds(200);
    return true;
}


static bool PMW3901_Read(const busDevice_t * busDev, uint8_t reg, uint8_t * data)
{
    spiBusSelectDevice(busDev);
    delayMicroseconds(50);
    //bool readsucc = busRead(busDev, reg & (~0x80u), data);

    //send request Reg Address
    SPI_TypeDef * instance = spiInstanceByDevice(busDev->busdev.spi.spiBus);
    reg &= ~0x80u;
    spiTransferByte(instance, reg);

    delayMicroseconds(50);
    
    //recive data
    uint8_t nullByteTx[2] = {0, 0};
    bool readsucc = spiTransfer(instance, data, nullByteTx, 1);

    delayMicroseconds(50);
    spiBusDeselectDevice(busDev);
    delayMicroseconds(200);
    return readsucc;
}

bool pmw3901OpflowDetect(opflowDev_t *dev)
{
    dev->busDev = busDeviceInit(BUSTYPE_SPI, DEVHW_PMW3901, 0, OWNER_FLOW);
    busDevice_t *busDev = dev->busDev;

    if (busDev == NULL)
    {
        return false;
    }
    dev->initFn = pmw3901OpflowInit;
    dev->updateFn = pmw3901OpflowUpdate;
    return true;
}

bool pmw3901OpflowInit(opflowDev_t *dev)
{
    busDevice_t *busDev = dev->busDev;

    spiConfigureWithSpeed(SPI_BAUDRATE_2MHZ);
    busSetSpeed(busDev, BUS_SPEED_FAST);

    spiBusDeselectDevice(busDev);
    spiBusSelectDevice(busDev);
    spiBusDeselectDevice(busDev);
    
    //timeUs_t
    uint8_t chipId[1]={100};
    uint8_t dipihc[1]={100};
    PMW3901_Read(busDev, PMW3901_REG_DIPIHC, dipihc);
    delayMicroseconds(200);
    PMW3901_Read(busDev, PMW3901_REG_CHIPID, chipId);
    delayMicroseconds(200);
    // uint8_t dIpihc[2] = {100,100};
    // PMW3901_Read(busDev, 0x5F, dIpihc);
    // delayMicroseconds(200);
    DEBUG_SET(DEBUG_FLOW_RAW, 4, (chipId[0]));
    DEBUG_SET(DEBUG_FLOW_RAW, 5, (dipihc[0]));
    // delayMicroseconds(15);
    delay(50);
    // Device Reset
    PMW3901_Write(busDev, PMW3901_REG_RESET, PMW3901_REG_RESET_SET_VALUE);
    delay(50);

    delay(5);
    pmw3901RegisterInit(dev);
    delay(5);
    delayMicroseconds(15);

    //busSetSpeed(busDev, BUS_SPEED_FAST);

    return true;
}

void pmw3901RegisterInit(opflowDev_t *dev)
{
    //DEBUG_SET(DEBUG_FLOW_RAW, 6, 4);
    busDevice_t *busDev = dev->busDev;
    PMW3901_Write(busDev, 0x7F, 0x00);
    PMW3901_Write(busDev, 0x61, 0xAD);
    PMW3901_Write(busDev, 0x7F, 0x03);
    PMW3901_Write(busDev, 0x40, 0x00);
    PMW3901_Write(busDev, 0x7F, 0x05);
    PMW3901_Write(busDev, 0x41, 0xB3);
    PMW3901_Write(busDev, 0x43, 0xF1);
    PMW3901_Write(busDev, 0x45, 0x14);
    PMW3901_Write(busDev, 0x5B, 0x32);
    PMW3901_Write(busDev, 0x5F, 0x34);
    PMW3901_Write(busDev, 0x7B, 0x08);
    PMW3901_Write(busDev, 0x7F, 0x06);
    PMW3901_Write(busDev, 0x44, 0x1B);
    PMW3901_Write(busDev, 0x40, 0xBF);
    PMW3901_Write(busDev, 0x4E, 0x3F);
}

bool pmw3901OpflowUpdate(opflowDev_t *dev)
{
    busDevice_t *busDev = dev->busDev;
    opflowData_t *data;
    //static timeUs_t previousTimeUs = 0;
    const timeUs_t currentTimeUs = micros();

    bool newPacket = false;
    opflowData_t tmpData = {0};

    uint8_t chipId[1]={100};
    uint8_t dipihc[1]={100};
    PMW3901_Read(busDev, PMW3901_REG_CHIPID, chipId);
    delayMicroseconds(200);
    PMW3901_Read(busDev, PMW3901_REG_DIPIHC, dipihc);
    delayMicroseconds(200);
    // uint8_t dIpihc[2] = {100,100};
    // PMW3901_Read(busDev, 0x5F, dIpihc);
    // delayMicroseconds(200);
    DEBUG_SET(DEBUG_FLOW_RAW, 6, (chipId[0]));
    DEBUG_SET(DEBUG_FLOW_RAW, 7, (dipihc[0]));
    
    DEBUG_SET(DEBUG_FLOW_RAW, 3, currentTimeUs%255);
    //DEBUG_SET(DEBUG_FLOW_RAW, 2, busDev->busdev.spi.csnPin);
    //536907740 PD2?

    if (newPacket)
    {
        *data = tmpData;
    }

    return newPacket;
}

#endif