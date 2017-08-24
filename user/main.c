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

// Comment this define to have smoother orientation (but some movements will be ignored)
#define EULER_ANGLES

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

#define M_PI 3.14159265359f

static float sine(float x)
{
    static float B = 4.f/M_PI;
    static float C = -4.f/(M_PI*M_PI);

    float y = B * x + C * x * abs(x);

    //  const float Q = 0.775;
    float P = 0.225f;
    y = P * (y * abs(y) - y) + y;   // Q * y + P * y * abs(y)
    return y;
}

static float cosine(float x)
{
    return sine(x + (M_PI / 2.f));
}

#ifdef EULER_ANGLES

float atan2_approx(float y, float x)
{
    static float ONEQTR_PI = M_PI / 4.0;
	static float THRQTR_PI = 3.0 * M_PI / 4.0;
	float r, angle;
	float abs_y = abs(y) + 1e-10f;
	if ( x < 0.0f )
	{
		r = (x + abs_y) / (abs_y - x);
		angle = THRQTR_PI;
	}
	else
	{
		r = (x - abs_y) / (x + abs_y);
		angle = ONEQTR_PI;
	}
	angle += (0.1963f * r * r - 0.9817f) * r;
	if ( y < 0.0f )
		return -angle;

    return angle;
}

static void eulerToQuaternion(SceFQuaternion* quat, float x, float y, float z)
{
	float cy = cosine(z * 0.5f);
	float sy = sine(z * 0.5f);
	float cr = cosine(y * 0.5f);
	float sr = sine(y * 0.5f);
	float cp = cosine(x * 0.5f);
	float sp = sine(x * 0.5f);

	quat->w = cy * cr * cp + sy * sr * sp;
	quat->x = cy * sr * cp - sy * cr * sp;
	quat->y = cy * cr * sp + sy * sr * cp;
	quat->z = sy * cr * cp - cy * sr * sp;
}

#else

static float arccosine(float x)
{
    float a=1.43f+0.59f*x;
    a=(a+(2.f+2.f*x)/a)/2.f;
    float b=1.65f-1.41f*x;
    b=(b+(2.f-2.f*x)/b)/2.f;
    float c=0.88f-0.77f*x;
    c=(c+(2.f-a)/c)/2.f;
    return 8.f/3.f*c-b/3.f;
}

#endif

/*static void quaternionProduct(SceFQuaternion* res, SceFQuaternion* q1, SceFQuaternion* q2)
{
    res->w = q1->w*q2->w - q1->x*q2->x - q1->y*q2->y - q1->z*q2->z;
    res->x = q1->w*q2->x + q1->x*q2->w - q1->y*q2->z + q1->z*q2->y;
    res->y = q1->w*q2->y + q1->x*q2->z + q1->y*q2->w - q1->z*q2->x;
    res->z = q1->w*q2->z - q1->x*q2->y + q1->y*q2->x + q1->z*q2->w;
}*/

static int computeQuaternionFromAccel(SceFQuaternion* oRes, SceFVector3* iAccel)
{
    float accelNorm = fastsqrt(iAccel->x*iAccel->x + iAccel->y*iAccel->y + iAccel->z*iAccel->z);
    if (accelNorm < 0.001f)
        return 0;

    SceFVector3 normAccel = { iAccel->x / accelNorm , iAccel->y / accelNorm , iAccel->z / accelNorm };
    
#ifdef EULER_ANGLES
    float pitch = atan2_approx(normAccel.z, -normAccel.y);
    float roll = atan2_approx(-normAccel.x, -normAccel.z*sign(-pitch));

    eulerToQuaternion(oRes, 0.f, pitch, roll);
#else
    static SceFVector3 initDir = {0.f, -1.f, 0.f};
    
    oRes->x = initDir.z*normAccel.y - initDir.y*normAccel.z;
    oRes->y = initDir.x*normAccel.z - initDir.z*normAccel.x;
    oRes->z = initDir.y*normAccel.x - initDir.x*normAccel.y;
    
    float angle = arccosine(initDir.x*normAccel.x + initDir.y*normAccel.y + initDir.z*normAccel.z);
    float half_sin = sine(0.5f * angle);
    float half_cos = cosine(0.5f * angle);
    oRes->w = half_cos;
    
    float crossNorm = fastsqrt(oRes->x*oRes->x + oRes->y*oRes->y + oRes->z*oRes->z);
    oRes->x *= half_sin / crossNorm;
    oRes->y *= half_sin / crossNorm;
    oRes->z *= half_sin / crossNorm;
#endif
    
    return 1;
}

static float identityMat[16] = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};
static SceFQuaternion identityQuat = {0.f, 0.f, 0.f, 1.f};

static unsigned int initTimestamp;
static unsigned int initCounter;

#define DECL_FUNC_HOOK(name, ...) \
	static tai_hook_ref_t name##_ref; \
	static SceUID name##_hook_uid = -1; \
	static int name##_hook_func(__VA_ARGS__)

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
            motionState->angularVelocity.x = (float)gyro[0] / 2607.6f;  // Pitch
            motionState->angularVelocity.y = -(float)gyro[2] / 2607.6f; // Roll
            motionState->angularVelocity.z = (float)gyro[1] / 2607.6f;  // Yaw
            
            int maxComp = (abs(accel[1]) > abs(accel[0])) ? 1 : 0;
            maxComp = (abs(accel[2]) > abs(accel[maxComp])) ? 2 : maxComp;

            motionState->basicOrientation.x = (2 == maxComp) ? sign(accel[2]) : 0.f;
            motionState->basicOrientation.y = (0 == maxComp) ? -sign(accel[0]) : 0.f;
            motionState->basicOrientation.z = (1 == maxComp) ? sign(accel[1]) : 0.f;

            if (computeQuaternionFromAccel(&motionState->deviceQuat, &motionState->acceleration))
            {
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
                memcpy(&motionState->deviceQuat, &identityQuat, sizeof(identityQuat));
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
