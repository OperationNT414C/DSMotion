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

//#include "log.h"
#include <string.h>
#include "../DSMotionLibrary.h"

#define abs(val) ((val < 0) ? -val : val)
#define sign(val) ((val > 0) ? 1 : ((val < 0) ? -1 : 0))

static float fastsqrt(float val)
{
    union
    {
        int tmp;
        float f;
    } u;
    u.f = val;
    u.tmp -= 1 << 23; /* Subtract 2^m. */
    u.tmp >>= 1; /* Divide by 2. */
    u.tmp += 1 << 29; /* Add ((b + 1) / 2) * 2^m. */
    return u.f;
}

static float identityMat[16] = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};
static SceFVector3 initDir = {0.f, -1.f, 0.f};

#define DECL_FUNC_HOOK(name, ...) \
	static tai_hook_ref_t name##_ref; \
	static SceUID name##_hook_uid = -1; \
	static int name##_hook_func(__VA_ARGS__)

static unsigned int initTimestamp;
static unsigned int initCounter;

DECL_FUNC_HOOK(SceMotion_sceMotionStartSampling)
{
	int ret = TAI_CONTINUE(int, SceMotion_sceMotionStartSampling_ref);
    if (ret >= 0)
    {
        initTimestamp = dsGetCurrentTimestamp();
        initCounter = dsGetCurrentCounter();
    }
    return ret;
}

DECL_FUNC_HOOK(SceMotion_sceMotionGetState, SceMotionState *motionState)
{
	int ret = TAI_CONTINUE(int, SceMotion_sceMotionGetState_ref, motionState);
    
    if (ret >= 0 && NULL != motionState)
    {
        signed short accel[3];
        signed short gyro[3];

        if (dsGetSampledAccelGyro(100, accel, gyro) > 0)
        {
            motionState->hostTimestamp = sceKernelGetProcessTimeWide();

            motionState->acceleration.x = -(float)accel[2] / 0x2000;
            motionState->acceleration.y = (float)accel[0] / 0x2000;
            motionState->acceleration.z = -(float)accel[1] / 0x2000;

            // 2607.6 = 0x2000 / PI
            motionState->angularVelocity.x = (float)gyro[0] / 2607.6f;
            motionState->angularVelocity.y = -(float)gyro[2] / 2607.6f;
            motionState->angularVelocity.z = (float)gyro[1] / 2607.6f;
            
            int maxComp = (abs(accel[1]) > abs(accel[0])) ? 1 : 0;
            maxComp = (abs(accel[2]) > abs(accel[maxComp])) ? 2 : maxComp;

            motionState->basicOrientation.x = (2 == maxComp) ? sign(accel[2]) : 0.f;
            motionState->basicOrientation.y = (0 == maxComp) ? -sign(accel[0]) : 0.f;
            motionState->basicOrientation.z = (1 == maxComp) ? sign(accel[1]) : 0.f;
            
            float accelNorm = fastsqrt(motionState->acceleration.x*motionState->acceleration.x
                                     + motionState->acceleration.y*motionState->acceleration.y
                                     + motionState->acceleration.z*motionState->acceleration.z);
            if (accelNorm > 0.001f)
            {
                SceFVector3 normAccel;
                normAccel.x = motionState->acceleration.x / accelNorm;
                normAccel.y = motionState->acceleration.y / accelNorm;
                normAccel.z = motionState->acceleration.z / accelNorm;
                
                motionState->deviceQuat.x = initDir.z*normAccel.y - initDir.y*normAccel.z;
                motionState->deviceQuat.y = initDir.x*normAccel.z - initDir.z*normAccel.x;
                motionState->deviceQuat.z = initDir.y*normAccel.x - initDir.x*normAccel.y;
                motionState->deviceQuat.w = initDir.x*normAccel.x + initDir.y*normAccel.y + initDir.z*normAccel.z;
                
                float sqx = motionState->deviceQuat.x*motionState->deviceQuat.x;
                float sqy = motionState->deviceQuat.y*motionState->deviceQuat.y;
                float sqz = motionState->deviceQuat.z*motionState->deviceQuat.z;
                float sqw = motionState->deviceQuat.w*motionState->deviceQuat.w;
                float invs = 1.f / (sqx + sqy + sqz + sqw);
                
                float* rotMat = (float*)&motionState->rotationMatrix;
                rotMat[0]  = ( sqx - sqy - sqz + sqw) * invs;
                rotMat[5]  = (-sqx + sqy - sqz + sqw) * invs;
                rotMat[10] = (-sqx - sqy + sqz + sqw) * invs;
                
                float tmp1 = motionState->deviceQuat.x*motionState->deviceQuat.y;
                float tmp2 = motionState->deviceQuat.z*motionState->deviceQuat.w;
                rotMat[4] = 2.f * (tmp1 + tmp2) * invs;
                rotMat[1] = 2.f * (tmp1 - tmp2) * invs;
                
                tmp1 = motionState->deviceQuat.x*motionState->deviceQuat.z;
                tmp2 = motionState->deviceQuat.y*motionState->deviceQuat.w;
                rotMat[8] = 2.f * (tmp1 - tmp2) * invs ;
                rotMat[2] = 2.f * (tmp1 + tmp2) * invs ;
                
                tmp1 = motionState->deviceQuat.y*motionState->deviceQuat.z;
                tmp2 = motionState->deviceQuat.x*motionState->deviceQuat.w;
                rotMat[9] = 2.f * (tmp1 + tmp2) * invs;
                rotMat[6] = 2.f * (tmp1 - tmp2) * invs;
                
                rotMat[3] = rotMat[7] = rotMat[11] = rotMat[12] = rotMat[13] = rotMat[14] = 0.f;
                rotMat[15] = 1.f;
            }
            else
            {
                motionState->deviceQuat.x = 0.f;
                motionState->deviceQuat.y = 0.f;
                motionState->deviceQuat.z = 0.f;
                motionState->deviceQuat.w = 1.f;
                
                memcpy(&motionState->rotationMatrix, identityMat, sizeof(identityMat));
            }
            
            memcpy(&motionState->nedMatrix, identityMat, sizeof(identityMat));
        }
    }

    return ret;
}

DECL_FUNC_HOOK(SceMotion_sceMotionGetSensorState, SceMotionSensorState *sensorState, int numRecords)
{
	int ret = TAI_CONTINUE(int, SceMotion_sceMotionGetSensorState_ref, sensorState, numRecords);
    if (ret >= 0 && NULL != sensorState)
    {
        struct accelGyroData data;
        for (int i = 0 ; i < numRecords ; i++)
        {
            if (dsGetInstantAccelGyro(numRecords-1-i, &data) >= 0)
            {
                SceMotionSensorState* curState = &sensorState[i];

                curState->accelerometer.x = -(float)data.accel[2] / 0x2000;
                curState->accelerometer.y = (float)data.accel[0] / 0x2000;
                curState->accelerometer.z = -(float)data.accel[1] / 0x2000;

                // 2608.6 = 0x2000 / PI
                curState->gyro.x = (float)data.gyro[0] / 2607.6f;
                curState->gyro.y = -(float)data.gyro[2] / 2608.6f;
                curState->gyro.z = (float)data.gyro[1] / 2608.6f;

                curState->timestamp = data.timestamp - initTimestamp;
                curState->counter = data.counter - initCounter;
            }
        }
    }
    return ret;
}

void _start() __attribute__ ((weak, alias ("module_start")));

#define BIND_FUNC_IMPORT_HOOK(name, module_nid, lib_nid, func_nid) \
	name##_hook_uid = taiHookFunctionImport(&name##_ref, (module_nid), (lib_nid), (func_nid), name##_hook_func)

int module_start(SceSize argc, const void *args)
{
    //log_reset();
    //LOG("Starting module\n");

    /* SceMotion hooks */
    BIND_FUNC_IMPORT_HOOK(SceMotion_sceMotionStartSampling, TAI_MAIN_MODULE, 0xDC571B3F, 0x28034AC9);
    //LOG("sceMotionStartSampling hook result: %x\n", SceMotion_sceMotionStartSampling_hook_uid);

    BIND_FUNC_IMPORT_HOOK(SceMotion_sceMotionGetState, TAI_MAIN_MODULE, 0xDC571B3F, 0xBDB32767);
    //LOG("sceMotionGetState hook result: %x\n", SceMotion_sceMotionGetState_hook_uid);

    BIND_FUNC_IMPORT_HOOK(SceMotion_sceMotionGetSensorState, TAI_MAIN_MODULE, 0xDC571B3F, 0x47D679EA);
    //LOG("sceMotionGetSensorState hook result: %x\n", SceMotion_sceMotionGetSensorState_hook_uid);

    //log_flush();

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

	//log_flush();

    return SCE_KERNEL_STOP_SUCCESS;
}
