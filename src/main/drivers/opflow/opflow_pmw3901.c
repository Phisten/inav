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
#define PMW3901_REG_DIPIHC_READ_VALUE 0xB6


// Port Define -----------------------------------------------
//#define LOW 0x0
//#define HIGH 0x1

//#define SPI                      SPI3
#define SPI_BAUDRATE_2MHZ   SPI_BaudRatePrescaler_64    // 1.3125MHz

#define OULIER_LIMIT 100

typedef struct motionBurst_s {
  union {
    uint8_t motion;
    struct {
      uint8_t frameFrom0    : 1;
      uint8_t runMode       : 2;
      uint8_t reserved1     : 1;
      uint8_t rawFrom0      : 1;
      uint8_t reserved2     : 2;
      uint8_t motionOccured : 1;
    };
  };

  uint8_t observation;
  int16_t deltaX;
  int16_t deltaY;

  uint8_t squal;

  uint8_t rawDataSum;
  uint8_t maxRawData;
  uint8_t minRawData;

  uint16_t shutter;
} __attribute__((packed)) motionBurst_t;


static bool isInit = false;

// SPI Func Port -----------------------------------------------------
static void spiConfigureWithSpeed(const busDevice_t * dev,uint16_t baudRatePrescaler)
{
#define BR_CLEAR_MASK 0xFFC7
    //inav : bus_spi.c/spiInitDevice()
    SPI_TypeDef * instance = spiInstanceByDevice(dev->busdev.spi.spiBus);

    SPI_InitTypeDef SPI_InitStructure;

    SPI_I2S_DeInit(instance);

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

    SPI_Init(instance, &SPI_InitStructure);

    SPI_Cmd(instance, DISABLE);
    uint16_t tempRegister = instance->CR1;
    tempRegister &= BR_CLEAR_MASK;
    tempRegister |= baudRatePrescaler;
    instance->CR1 = tempRegister;
    SPI_Cmd(instance, ENABLE);
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
    if (isInit) {
        return true;
    }
    busDevice_t *busDev = dev->busDev;

    spiConfigureWithSpeed(busDev,SPI_BAUDRATE_2MHZ);
    //busSetSpeed(busDev, BUS_SPEED_FAST);

    spiBusDeselectDevice(busDev);
    spiBusSelectDevice(busDev);
    spiBusDeselectDevice(busDev);
    
    delay(50);

    //timeUs_t
    uint8_t chipId[1]={100};
    uint8_t dipihc[1]={100};
    PMW3901_Read(busDev, PMW3901_REG_DIPIHC, dipihc);
    delayMicroseconds(200);
    PMW3901_Read(busDev, PMW3901_REG_CHIPID, chipId);
    delayMicroseconds(200);
    DEBUG_SET(DEBUG_FLOW_RAW, 4, (chipId[0]));
    DEBUG_SET(DEBUG_FLOW_RAW, 5, (dipihc[0]));
    
    if (chipId[0] == 0x49u && dipihc[0] == 0xB6u)
    {
        // Device Reset
        PMW3901_Write(busDev, PMW3901_REG_RESET, PMW3901_REG_RESET_SET_VALUE);
        delay(50);

        // Reading the motion registers one time
        // PMW3901_Read(busDev, 0x02);
        // PMW3901_Read(busDev, 0x03);
        // PMW3901_Read(busDev, 0x04);
        // PMW3901_Read(busDev, 0x05);
        // PMW3901_Read(busDev, 0x06);
        delayMicroseconds(1000);

        pmw3901RegisterInit(dev);

        isInit = true;
    }
    return isInit;
}

void pmw3901RegisterInit(opflowDev_t *dev)
{
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

    PMW3901_Write(busDev, 0x7F, 0x08);
    PMW3901_Write(busDev, 0x65, 0x20);
    PMW3901_Write(busDev, 0x6A, 0x18);
    PMW3901_Write(busDev, 0x7F, 0x09);
    PMW3901_Write(busDev, 0x4F, 0xAF);
    PMW3901_Write(busDev, 0x5F, 0x40);
    PMW3901_Write(busDev, 0x48, 0x80);
    PMW3901_Write(busDev, 0x49, 0x80);
    PMW3901_Write(busDev, 0x57, 0x77);
    PMW3901_Write(busDev, 0x60, 0x78);
    PMW3901_Write(busDev, 0x61, 0x78);
    PMW3901_Write(busDev, 0x62, 0x08);
    PMW3901_Write(busDev, 0x63, 0x50);
    PMW3901_Write(busDev, 0x7F, 0x0A);
    PMW3901_Write(busDev, 0x45, 0x60);
    PMW3901_Write(busDev, 0x7F, 0x00);
    PMW3901_Write(busDev, 0x4D, 0x11);
    PMW3901_Write(busDev, 0x55, 0x80);
    PMW3901_Write(busDev, 0x74, 0x1F);
    PMW3901_Write(busDev, 0x75, 0x1F);
    PMW3901_Write(busDev, 0x4A, 0x78);
    PMW3901_Write(busDev, 0x4B, 0x78);
    PMW3901_Write(busDev, 0x44, 0x08);
    PMW3901_Write(busDev, 0x45, 0x50);
    PMW3901_Write(busDev, 0x64, 0xFF);
    PMW3901_Write(busDev, 0x65, 0x1F);
    PMW3901_Write(busDev, 0x7F, 0x14);
    PMW3901_Write(busDev, 0x65, 0x67);
    PMW3901_Write(busDev, 0x66, 0x08);
    PMW3901_Write(busDev, 0x63, 0x70);
    PMW3901_Write(busDev, 0x7F, 0x15);
    PMW3901_Write(busDev, 0x48, 0x48);
    PMW3901_Write(busDev, 0x7F, 0x07);
    PMW3901_Write(busDev, 0x41, 0x0D);
    PMW3901_Write(busDev, 0x43, 0x14);
    PMW3901_Write(busDev, 0x4B, 0x0E);
    PMW3901_Write(busDev, 0x45, 0x0F);
    PMW3901_Write(busDev, 0x44, 0x42);
    PMW3901_Write(busDev, 0x4C, 0x80);
    PMW3901_Write(busDev, 0x7F, 0x10);
    PMW3901_Write(busDev, 0x5B, 0x02);
    PMW3901_Write(busDev, 0x7F, 0x07);
    PMW3901_Write(busDev, 0x40, 0x41);
    PMW3901_Write(busDev, 0x70, 0x00);

    delay(10); // delay 10ms

    PMW3901_Write(busDev, 0x32, 0x44);
    PMW3901_Write(busDev, 0x7F, 0x07);
    PMW3901_Write(busDev, 0x40, 0x40);
    PMW3901_Write(busDev, 0x7F, 0x06);
    PMW3901_Write(busDev, 0x62, 0xF0);
    PMW3901_Write(busDev, 0x63, 0x00);
    PMW3901_Write(busDev, 0x7F, 0x0D);
    PMW3901_Write(busDev, 0x48, 0xC0);
    PMW3901_Write(busDev, 0x6F, 0xD5);
    PMW3901_Write(busDev, 0x7F, 0x00);
    PMW3901_Write(busDev, 0x5B, 0xA0);
    PMW3901_Write(busDev, 0x4E, 0xA8);
    PMW3901_Write(busDev, 0x5A, 0x50);
    PMW3901_Write(busDev, 0x40, 0x80);

    PMW3901_Write(busDev, 0x7F, 0x00);
    PMW3901_Write(busDev, 0x5A, 0x10);
    PMW3901_Write(busDev, 0x54, 0x00);
}

bool pmw3901OpflowUpdate(opflowDev_t *dev)
{
    busDevice_t *busDev = dev->busDev;

    static timeUs_t previousTimeUs = 0;
    const timeUs_t currentTimeUs = micros();

    bool newPacket = false;
    opflowData_t tmpData = {0};

    //read data
    uint8_t address = 0x16;
    
    spiBusSelectDevice(busDev);
    delayMicroseconds(50);

    SPI_TypeDef * instance = spiInstanceByDevice(busDev->busdev.spi.spiBus);
    spiTransferByte(instance, address);
    motionBurst_t motion;
    spiTransfer(instance, (uint8_t*)&motion, (uint8_t*)&motion,sizeof(motionBurst_t));
    
    delayMicroseconds(50);
    spiBusDeselectDevice(busDev);

    //port data
    if (abs(motion.deltaX) < OULIER_LIMIT && abs(motion.deltaY) < OULIER_LIMIT) {
        tmpData.deltaTime += (currentTimeUs - previousTimeUs);
        tmpData.flowRateRaw[0] += motion.deltaX;
        tmpData.flowRateRaw[1] += motion.deltaY;
        tmpData.flowRateRaw[2] = 0;
        tmpData.quality = (constrain(motion.squal, 64, 78) - 64) * 100 / 14;
        previousTimeUs = currentTimeUs;
        newPacket = true;

        uint16_t realShutter = (motion.shutter >> 8) & 0x0FF;
        realShutter |= (motion.shutter & 0x0ff) << 8;
        motion.shutter = realShutter;
    }
    if (newPacket)
    {
        dev->rawData = tmpData;
    }

    return newPacket;
}

#endif