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

#include "io/serial.h"

#if defined(USE_OPFLOW_PMW3901)
#include "drivers/opflow/opflow_virtual.h"
#include "drivers/time.h"
#include "io/opflow.h"

#define PMW3901_PACKET_SIZE 9

//static serialPort_t *flowPort = NULL;
//static serialPortConfig_t *portConfig;
static uint8_t buffer[10];
static int bufferPtr;

typedef struct __attribute__((packed))
{
    uint8_t header; // 0xFE
    uint8_t res0;   // Seems to be 0x04 all the time
    int16_t motionX;
    int16_t motionY;
    int8_t motionT; // ???
    uint8_t squal;  // Not sure about this
    uint8_t footer; // 0xAA
} pmw3901Packet_t;

static bool pmw3901OpflowDetect(void)
{
    busDevice_t *busDev = opflow->dev->busDev;
    busDev = busDeviceInit(BUSTYPE_SPI, DEVHW_PMW3901, opflow->magSensorToUse, OWNER_COMPASS);

    if (busDev == NULL)
    {
        return false;
    }
    opflow->dev->initFn = pmw3901OpflowInit;
    opflow->dev->updateFn = pmw3901OpflowUpdate;
    return true;
}

static bool pmw3901OpflowInit(void)
{

    busDevice_t *busDev = opflow->dev->busDev;
    //const gyroFilterAndRateConfig_t *config = mpuChooseGyroConfig(gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs);
    //gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;

    busSetSpeed(busDev, BUS_SPEED_INITIALIZATION);
    debug[0] = 100;

    // Make sure the SPI bus is reset
    // digitalWrite(_cs, HIGH);
    // delay(1);
    // digitalWrite(_cs, LOW);
    // delay(1);
    // digitalWrite(_cs, HIGH);
    // delay(1);

    // Device Reset
    busWrite(busDev, 0x3a, 0x5a);

    delay(5);

    //timeUs_t
    uint8_t chipId = busRead(busDev, 0x00);
    uint8_t dIpihc = busRead(busDev, 0x5F);

    //busWrite(busDev, MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delayMicroseconds(15);

    // Clock Source PPL with Z axis gyro reference
    //busWrite(busDev, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delayMicroseconds(15);

    // Disable Primary I2C Interface

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled

    // Gyro +/- 2000 DPS Full Scale

    // Accel +/- 16 G Full Scale

    // Accel and Gyro DLPF Setting

    busSetSpeed(busDev, BUS_SPEED_FAST);

    if (!portConfig)
    {
        return false;
    }

    flowPort = openSerialPort(portConfig->identifier, FUNCTION_OPTICAL_FLOW, NULL, NULL, baudRates[BAUD_19200], MODE_RX, SERIAL_NOT_INVERTED);
    if (!flowPort)
    {
        return false;
    }

    bufferPtr = 0;

    return true;
}

static void pmw3901RegisterInit(void)
{
    busWrite(busDev, 0x7F, 0x00);
    busWrite(busDev, 0x61, 0xAD);
    busWrite(busDev, 0x7F, 0x03);
    busWrite(busDev, 0x40, 0x00);
    busWrite(busDev, 0x7F, 0x05);
    busWrite(busDev, 0x41, 0xB3);
    busWrite(busDev, 0x43, 0xF1);
    busWrite(busDev, 0x45, 0x14);
    busWrite(busDev, 0x5B, 0x32);
    busWrite(busDev, 0x5F, 0x34);
    busWrite(busDev, 0x7B, 0x08);
    busWrite(busDev, 0x7F, 0x06);
    busWrite(busDev, 0x44, 0x1B);
    busWrite(busDev, 0x40, 0xBF);
    busWrite(busDev, 0x4E, 0x3F);

    // busWrite(busDev, 0x7F, 0x08);
    // busWrite(busDev, 0x65, 0x20);
    // busWrite(busDev, 0x6A, 0x18);
    // busWrite(busDev, 0x7F, 0x09);
    // busWrite(busDev, 0x4F, 0xAF);
    // busWrite(busDev, 0x5F, 0x40);
    // busWrite(busDev, 0x48, 0x80);
    // busWrite(busDev, 0x49, 0x80);
    // busWrite(busDev, 0x57, 0x77);
    // busWrite(busDev, 0x60, 0x78);
    // busWrite(busDev, 0x61, 0x78);
    // busWrite(busDev, 0x62, 0x08);
    // busWrite(busDev, 0x63, 0x50);
    // busWrite(busDev, 0x7F, 0x0A);
    // busWrite(busDev, 0x45, 0x60);
    // busWrite(busDev, 0x7F, 0x00);
    // busWrite(busDev, 0x4D, 0x11);
    // busWrite(busDev, 0x55, 0x80);
    // busWrite(busDev, 0x74, 0x1F);
    // busWrite(busDev, 0x75, 0x1F);
    // busWrite(busDev, 0x4A, 0x78);
    // busWrite(busDev, 0x4B, 0x78);
    // busWrite(busDev, 0x44, 0x08);
    // busWrite(busDev, 0x45, 0x50);
    // busWrite(busDev, 0x64, 0xFF);
    // busWrite(busDev, 0x65, 0x1F);
    // busWrite(busDev, 0x7F, 0x14);
    // busWrite(busDev, 0x65, 0x60);
    // busWrite(busDev, 0x66, 0x08);
    // busWrite(busDev, 0x63, 0x78);
    // busWrite(busDev, 0x7F, 0x15);
    // busWrite(busDev, 0x48, 0x58);
    // busWrite(busDev, 0x7F, 0x07);
    // busWrite(busDev, 0x41, 0x0D);
    // busWrite(busDev, 0x43, 0x14);
    // busWrite(busDev, 0x4B, 0x0E);
    // busWrite(busDev, 0x45, 0x0F);
    // busWrite(busDev, 0x44, 0x42);
    // busWrite(busDev, 0x4C, 0x80);
    // busWrite(busDev, 0x7F, 0x10);
    // busWrite(busDev, 0x5B, 0x02);
    // busWrite(busDev, 0x7F, 0x07);
    // busWrite(busDev, 0x40, 0x41);
    // busWrite(busDev, 0x70, 0x00);
    // delay(100);
    // busWrite(busDev, 0x32, 0x44);
    // busWrite(busDev, 0x7F, 0x07);
    // busWrite(busDev, 0x40, 0x40);
    // busWrite(busDev, 0x7F, 0x06);
    // busWrite(busDev, 0x62, 0xf0);
    // busWrite(busDev, 0x63, 0x00);
    // busWrite(busDev, 0x7F, 0x0D);
    // busWrite(busDev, 0x48, 0xC0);
    // busWrite(busDev, 0x6F, 0xd5);
    // busWrite(busDev, 0x7F, 0x00);
    // busWrite(busDev, 0x5B, 0xa0);
    // busWrite(busDev, 0x4E, 0xA8);
    // busWrite(busDev, 0x5A, 0x50);
    // busWrite(busDev, 0x40, 0x80);
}

static bool pmw3901OpflowUpdate(opflowData_t *data)
{
    // static timeUs_t previousTimeUs = 0;
    // const timeUs_t currentTimeUs = micros();

    // bool newPacket = false;
    // opflowData_t tmpData = {0};

    // while (serialRxBytesWaiting(flowPort) > 0)
    // {
    //     uint8_t c = serialRead(flowPort);

    //     // Wait for header
    //     if (bufferPtr == 0)
    //     {
    //         if (c != 0xFE)
    //         {
    //             break;
    //         }
    //     }

    //     // Consume received byte
    //     if (bufferPtr < PMW3901_PACKET_SIZE)
    //     {
    //         buffer[bufferPtr++] = c;

    //         if (bufferPtr == PMW3901_PACKET_SIZE)
    //         {
    //             cxofPacket_t *pkt = (cxofPacket_t *)&buffer[0];

    //             if (pkt->header == 0xFE && pkt->footer == 0xAA)
    //             {
    //                 // Valid packet
    //                 tmpData.deltaTime += (currentTimeUs - previousTimeUs);
    //                 tmpData.flowRateRaw[0] += pkt->motionX;
    //                 tmpData.flowRateRaw[1] += pkt->motionY;
    //                 tmpData.flowRateRaw[2] = 0;
    //                 tmpData.quality = (constrain(pkt->squal, 64, 78) - 64) * 100 / 14;

    //                 previousTimeUs = currentTimeUs;
    //                 newPacket = true;
    //             }

    //             // Reset the decoder
    //             bufferPtr = 0;
    //         }
    //     }
    //     else
    //     {
    //         // In case of buffer overflow - reset the decoder
    //         bufferPtr = 0;
    //     }
    // }

    // if (newPacket)
    // {
    //     *data = tmpData;
    // }

    return newPacket;
}

virtualOpflowVTable_t opflowPmw3901Vtable = {
    .detect = pmw3901OpflowDetect,
    .init = pmw3901OpflowInit,
    .update = pmw3901OpflowUpdate};

#endif