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

#pragma once

#include "common/time.h"
#include "drivers/sensor.h"

//unused(TODO)
typedef enum {
    UWB_NONE         = 0,
    UWB_DWM1000      = 1,
} uwbSensor_e;

typedef struct uwb_s {
    timeUs_t        deltaTime;
    timeUs_t        lastValidUpdate;
    //bool            isHwHealty;
    int32_t         uwbPosRaw[2]; 
    //uint8_t         rawQuality;
} uwb_t;

extern uwb_t uwb;
bool uwbInit(void);
void uwbUpdate(timeUs_t currentTimeUs);
