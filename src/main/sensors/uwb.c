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
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/time.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "io/serial.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/uwb.h"

//#include "scheduler/scheduler.h"

#include "build/debug.h"

uwb_t uwb;

#ifdef USE_UWB

/******************************* driver bypass ***********************/
#define UWB_PACKET_SIZE    24
static serialPort_t * uwbPort = NULL;
static serialPortConfig_t * portConfig;
static uint8_t  buffer[24];
static int      bufferPtr;

static uint32_t grab_fields(uint8_t *src)
{                               // convert string to uint32
    uint32_t i;
    uint32_t tmp = 0;
    for (i = 0; src[i] != 0; i++) {
        char c = (char)src[i];
        if (c >= '0' && c <= '9')
            tmp = tmp * 10 + c - '0';
        else if (c == 'c')
            break;
        if (i >= 15)
            return 0; // out of bounds
    }
    return tmp;
}

bool dwm1000UbwDetect(void)
{
    portConfig = findSerialPortConfig(FUNCTION_UWB);
    if (!portConfig) {
        return false;
    }
    
    return true;
}
bool dwm1000UbwInit(void)
{
    // if (!portConfig) {
    //     return false;
    // }

    //SERIAL_PORT_USART2 == 1
    uwbPort = openSerialPort(portConfig->identifier, FUNCTION_UWB, NULL, NULL, baudRates[BAUD_115200], MODE_RXTX, SERIAL_NOT_INVERTED);
    //uwbPort = openSerialPort(SERIAL_PORT_USART2, FUNCTION_UWB, NULL, NULL, baudRates[BAUD_115200], MODE_RX, SERIAL_NOT_INVERTED);
    if (!uwbPort) {
        return false;
    }
    DEBUG_SET(DEBUG_UWB, 3, portConfig->identifier + 100);
    bufferPtr = 0;

    return true;
}
bool dwm1000UbwUpdate(timeUs_t currentTimeUs)
{
    if (!uwbPort) {
        return false;
    }

    // true true 115200  allpass
    // DEBUG_SET(DEBUG_UWB, 6, serialIsConnected(uwbPort));
    // DEBUG_SET(DEBUG_UWB, 5, serialIsPortAvailable(uwbPort->identifier));
    // DEBUG_SET(DEBUG_UWB, 4, serialGetBaudRate(uwbPort));
    bool newPacket = false;

    while (serialRxBytesWaiting(uwbPort) > 0) {
        uint8_t readByte = serialRead(uwbPort);
        char c = (char)readByte;
        // Wait for header
        if (bufferPtr == 0) {
            if (c != 'X' && c != 'Y') {
                break;
            }
        }

        // Consume received byte
        if (c != 0x20) {
            buffer[bufferPtr++] = readByte;

            if (c == 'c') {
                if ((char)buffer[0] == 'X') {
                    // Valid packet
                    uwb.uwbPosRaw[0] = grab_fields(buffer);
                }else if((char)buffer[0] == 'Y')
                {
                    uwb.deltaTime += (currentTimeUs - uwb.lastValidUpdate);
                    uwb.uwbPosRaw[1] = grab_fields(buffer);
                    uwb.lastValidUpdate = currentTimeUs;
                    newPacket = true;

                    DEBUG_SET(DEBUG_UWB, 0, uwb.uwbPosRaw[0]);
                    DEBUG_SET(DEBUG_UWB, 1, uwb.uwbPosRaw[1]);
                    //DEBUG_SET(DEBUG_UWB, 2, uwb.deltaTime);
                }
                // Reset the decoder
                bufferPtr = 0;
            }
        }
        else {
            // In case of buffer uwb - reset the decoder
            bufferPtr = 0;
        }
    }

    return newPacket;
}
/******************************* driver detect bypass ***********************/

void uwbZero(void)
{
    uwb.uwbPosRaw[0] = 0;
    uwb.uwbPosRaw[1] = 0;
    uwb.lastValidUpdate = micros();
    uwb.deltaTime = 0;
}

static bool uwbDetect(void)
{
    //uwbSensor_e uwbHardware = UWB_DWM1000;
    //requestedSensors[SENSOR_INDEX_UWB] = uwbHardwareToUse;

    dwm1000UbwDetect();

    detectedSensors[SENSOR_INDEX_UWB] = UWB_DWM1000;

    sensorsSet(SENSOR_UWB);
    return true;
}

bool uwbInit(void)
{
    if (!uwbDetect()) {
        return false;
    }

    if (!dwm1000UbwInit()) {
        sensorsClear(SENSOR_UWB);
        return false;
    }
    uwbZero();

    return true;
}

/*
 * This is called periodically by the scheduler
 */
void uwbUpdate(timeUs_t currentTimeUs)
{
    //bool hasNewData =
    dwm1000UbwUpdate(currentTimeUs);
}

#endif
