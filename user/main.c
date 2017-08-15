/*
 *  DSMotion user plugin
 *  Copyright (c) 2017 OperationNT
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:

 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.

 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */
#include <psp2/kernel/modulemgr.h>
#include <psp2/kernel/processmgr.h>
#include <psp2/kernel/clib.h>
#include <psp2/kernel/sysmem.h>
#include <psp2/motion.h>
#include <taihen.h>

#include "log.h"
#include "../DSMotionLibrary.h"

#define abs(val) ((val < 0) ? -val : val)
#define sign(val) ((val > 0) ? 1 : ((val < 0) ? -1 : 0))

#define DECL_FUNC_HOOK(name, ...) \
	static tai_hook_ref_t name##_ref; \
	static SceUID name##_hook_uid = -1; \
	static int name##_hook_func(__VA_ARGS__)

DECL_FUNC_HOOK(SceMotion_sceMotionStartSampling)
{
	int ret = TAI_CONTINUE(int, SceMotion_sceMotionStartSampling_ref);
    if (ret >= 0)
        dsResetAccelGyroSampling();
    return ret;
}

DECL_FUNC_HOOK(SceMotion_sceMotionGetState, SceMotionState *motionState)
{
	int ret = TAI_CONTINUE(int, SceMotion_sceMotionGetState_ref, motionState);
    
    if (ret >= 0 && NULL != motionState)
    {
        signed short accel[3];
        signed short gyro[3];

        if (dsGetSampledAccelGyro(accel, gyro) >= 0)
        {
            motionState->acceleration.x = (float)gyro[2] / 0x2000;
            motionState->acceleration.y = -(float)gyro[0] / 0x2000;
            motionState->acceleration.z = -(float)gyro[1] / 0x2000;

            // 2608.6 = 0x2000 / PI
            motionState->angularVelocity.x = (float)accel[0] / 2607.6f;
            motionState->angularVelocity.y = (float)accel[2] / 2608.6f;
            motionState->angularVelocity.z = (float)accel[1] / 2608.6f;
            
            int maxComp = (abs(gyro[1]) > abs(gyro[0])) ? 1 : 0;
            maxComp = (abs(gyro[2]) > abs(gyro[maxComp])) ? 2 : maxComp;

            motionState->basicOrientation.x = (2 == maxComp) ? sign(gyro[2]) : 0.f;
            motionState->basicOrientation.y = (0 == maxComp) ? -sign(gyro[0]) : 0.f;
            motionState->basicOrientation.z = (1 == maxComp) ? sign(gyro[1]) : 0.f;
        }
    }

    return ret;
}

DECL_FUNC_HOOK(SceMotion_sceMotionGetSensorState, SceMotionSensorState *sensorState, int numRecords)
{
	int ret = TAI_CONTINUE(int, SceMotion_sceMotionGetSensorState_ref, sensorState, numRecords);
    return ret;
}

DECL_FUNC_HOOK(SceMotion_sceMotionGetBasicOrientation, SceFVector3 *basicOrientation)
{
	int ret = TAI_CONTINUE(int, SceMotion_sceMotionGetBasicOrientation_ref, basicOrientation);
    return ret;
}

void _start() __attribute__ ((weak, alias ("module_start")));

#define BIND_FUNC_IMPORT_HOOK(name, module_nid, lib_nid, func_nid) \
	name##_hook_uid = taiHookFunctionImport(&name##_ref, (module_nid), (lib_nid), (func_nid), name##_hook_func)

int module_start(SceSize argc, const void *args)
{
    log_reset();
    LOG("Starting module\n");

    /* SceMotion hooks */
    BIND_FUNC_IMPORT_HOOK(SceMotion_sceMotionStartSampling, TAI_MAIN_MODULE, 0xDC571B3F, 0x28034AC9);
    LOG("sceMotionStartSampling hook result: %x\n", SceMotion_sceMotionStartSampling_hook_uid);

    BIND_FUNC_IMPORT_HOOK(SceMotion_sceMotionGetState, TAI_MAIN_MODULE, 0xDC571B3F, 0xBDB32767);
    LOG("sceMotionGetState hook result: %x\n", SceMotion_sceMotionGetState_hook_uid);

    BIND_FUNC_IMPORT_HOOK(SceMotion_sceMotionGetSensorState, TAI_MAIN_MODULE, 0xDC571B3F, 0x47D679EA);
    LOG("sceMotionGetSensorState hook result: %x\n", SceMotion_sceMotionGetSensorState_hook_uid);

    BIND_FUNC_IMPORT_HOOK(SceMotion_sceMotionGetBasicOrientation, TAI_MAIN_MODULE, 0xDC571B3F, 0x4F28BFE0);
    LOG("sceMotionGetBasicOrientation hook result: %x\n", SceMotion_sceMotionGetBasicOrientation_hook_uid);

    log_flush();

    return SCE_KERNEL_START_SUCCESS;
}

#define UNBIND_FUNC_HOOK(name) \
	do { \
		if (name##_hook_uid > 0) { \
			taiHookRelease(name##_hook_uid, name##_ref); \
		} \
	} while(0)

int module_stop(SceSize argc, const void *args)
{
	UNBIND_FUNC_HOOK(SceMotion_sceMotionStartSampling);
	UNBIND_FUNC_HOOK(SceMotion_sceMotionGetState);
    UNBIND_FUNC_HOOK(SceMotion_sceMotionGetSensorState);
    UNBIND_FUNC_HOOK(SceMotion_sceMotionGetBasicOrientation);

	log_flush();

    return SCE_KERNEL_STOP_SUCCESS;
}
