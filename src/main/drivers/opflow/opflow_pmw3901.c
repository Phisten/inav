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
#include "drivers/bus.h"
#include "drivers/opflow/opflow_pmw3901.h"
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
    //const gyroFilterAndRateConfig_t *config = mpuChooseGyroConfig(gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs);
    //gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;

    //busSetSpeed(busDev, BUS_SPEED_FAST);
    delay(50);
    // ioTag_t csnPin = busDev->busdev.spi.csnPin;
    // IOHi(csnPin);
    // delay(1);
    // IOLo(csnPin);
    // delay(1);
    // IOHi(csnPin);
    // delay(1);

    // Device Reset
    busWrite(busDev, 0x3a, 0x5a);
    delay(50);


    //timeUs_t
    uint8_t chipId = 100;
    for (int attempts = 0; attempts < 5; attempts++) {
        uint8_t tchipId;

        delay(100);


        // IOLo(csnPin);
        // delayMicroseconds(50);
        busRead(busDev, 0x00, &tchipId);
        delayMicroseconds(200);
        // IOHi(csnPin);
        // delayMicroseconds(200);
        if (tchipId != 0xFF) {
            chipId = tchipId;
        }
    }
    uint8_t dIpihc = 100;
    busRead(busDev, 0x5F, &dIpihc);
    delayMicroseconds(200);

    

    DEBUG_SET(DEBUG_FLOW_RAW, 4, (chipId));
    DEBUG_SET(DEBUG_FLOW_RAW, 5, (dIpihc));
    // delayMicroseconds(15);
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
}

bool pmw3901OpflowUpdate(opflowDev_t *dev)
{
    busDevice_t *busDev = dev->busDev;
    opflowData_t *data;
    static timeUs_t previousTimeUs = 0;
    const timeUs_t currentTimeUs = micros();

    bool newPacket = false;
    opflowData_t tmpData = {0};

    uint8_t chipId = 1;
    busRead(busDev, 0x00, &chipId);
    uint8_t dIpihc = 1;
    busRead(busDev, 0x5F, &dIpihc);

    DEBUG_SET(DEBUG_FLOW_RAW, 6, (chipId));
    DEBUG_SET(DEBUG_FLOW_RAW, 7, (dIpihc));
    
    DEBUG_SET(DEBUG_FLOW_RAW, 3, currentTimeUs%255);

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

    if (newPacket)
    {
        *data = tmpData;
    }

    return newPacket;
}

// virtualOpflowVTable_t opflowPmw3901Vtable = {
//     .detect = pmw3901OpflowDetect,
//     .init = pmw3901OpflowInit,
//     .update = pmw3901OpflowUpdate};

#endif