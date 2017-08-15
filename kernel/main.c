/*
 *  DSMotion kernel plugin
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
#include <psp2kern/kernel/modulemgr.h>
#include <psp2kern/kernel/threadmgr.h>
#include <psp2kern/kernel/sysmem.h>
#include <psp2kern/kernel/suspend.h>
#include <psp2kern/bt.h>
#include <psp2kern/ctrl.h>
#include <psp2/touch.h>
#include <psp2/motion.h>
#include <taihen.h>

#include "log.h"
#include "../DSMotionLibrary.h"

#define DS4_VID   0x054C
#define DS4_PID   0x05C4
#define DS4_2_PID 0x09CC

#define abs(x) (((x) < 0) ? -(x) : (x))

struct ds4_input_report {
	unsigned char report_id;
	unsigned char left_x;
	unsigned char left_y;
	unsigned char right_x;
	unsigned char right_y;

	unsigned char dpad     : 4;
	unsigned char square   : 1;
	unsigned char cross    : 1;
	unsigned char circle   : 1;
	unsigned char triangle : 1;

	unsigned char l1      : 1;
	unsigned char r1      : 1;
	unsigned char l2      : 1;
	unsigned char r2      : 1;
	unsigned char share   : 1;
	unsigned char options : 1;
	unsigned char l3      : 1;
	unsigned char r3      : 1;

	unsigned char ps   : 1;
	unsigned char tpad : 1;
	unsigned char cnt1 : 6;

	unsigned char l_trigger;
	unsigned char r_trigger;

	unsigned char cnt2;
	unsigned char cnt3;

	unsigned char battery;

	signed short accel_x;
	signed short accel_y;
	signed short accel_z;

	union {
		signed short roll;
		signed short gyro_z;
	};
	union {
		signed short yaw;
		signed short gyro_y;
	};
	union {
		signed short pitch;
		signed short gyro_x;
	};

	unsigned char unk1[5];

	unsigned char battery_level : 4;
	unsigned char usb_plugged   : 1;
	unsigned char headphones    : 1;
	unsigned char microphone    : 1;
	unsigned char padding       : 1;

	unsigned char unk2[2];
	unsigned char trackpadpackets;
	unsigned char packetcnt;

	unsigned int finger1_id        : 7;
	unsigned int finger1_activelow : 1;
	unsigned int finger1_x         : 12;
	unsigned int finger1_y         : 12;

	unsigned int finger2_id        : 7;
	unsigned int finger2_activelow : 1;
	unsigned int finger2_x         : 12;
	unsigned int finger2_y         : 12;

} __attribute__((packed, aligned(32)));

static int ds4_connected = 0;
static unsigned int ds4_mac0 = 0;
static unsigned int ds4_mac1 = 0;

static struct ds4_input_report ds4_input;
static unsigned char* recv_buff = NULL;

static int accel_x_sum = 0;
static int accel_y_sum = 0;
static int accel_z_sum = 0;

static int gyro_x_sum = 0;
static int gyro_y_sum = 0;
static int gyro_z_sum = 0;

static int samplesCount = 0;

int dsResetAccelGyroSampling()
{
    if (!ds4_connected)
        return -1;
    
    accel_x_sum = 0;
    accel_y_sum = 0;
    accel_z_sum = 0;

    gyro_x_sum = 0;
    gyro_y_sum = 0;
    gyro_z_sum = 0;

    samplesCount = 0;

    return 0;
}

int dsGetSampledAccelGyro(signed short oAccel[3], signed short oGyro[3])
{
    if (!ds4_connected)
        return -1;
    
    if (samplesCount > 0)
    {
        signed short accel[3] = {accel_x_sum / samplesCount, accel_y_sum / samplesCount, accel_z_sum / samplesCount};
        signed short gyro[3] = {gyro_x_sum / samplesCount, gyro_y_sum / samplesCount, gyro_z_sum / samplesCount};
    
        ksceKernelMemcpyKernelToUser((uintptr_t)oAccel, (const void *)accel, sizeof(accel));
        ksceKernelMemcpyKernelToUser((uintptr_t)oGyro, (const void *)gyro, sizeof(gyro));
        
        dsResetAccelGyroSampling();
    }
    else
        dsGetInstantAccelGyro(oAccel, oGyro);
    
    return 0;
}

int dsGetInstantAccelGyro(signed short oAccel[3], signed short oGyro[3])
{
    if (!ds4_connected)
        return -1;

    signed short accel[3] = {ds4_input.accel_x, ds4_input.accel_y, ds4_input.accel_z};
    signed short gyro[3] = {ds4_input.gyro_x, ds4_input.gyro_y, ds4_input.gyro_z};
    
    ksceKernelMemcpyKernelToUser((uintptr_t)oAccel, (const void *)accel, sizeof(accel));
    ksceKernelMemcpyKernelToUser((uintptr_t)oGyro, (const void *)gyro, sizeof(gyro));
    
    return 0;
}

static inline void ds4_input_reset(void)
{
	memset(&ds4_input, 0, sizeof(ds4_input));
}

static int is_ds4(const unsigned short vid_pid[2])
{
	return (vid_pid[0] == DS4_VID) &&
		((vid_pid[1] == DS4_PID) || (vid_pid[1] == DS4_2_PID));
}

#define DECL_FUNC_HOOK(name, ...) \
	static tai_hook_ref_t name##_ref; \
	static SceUID name##_hook_uid = -1; \
	static int name##_hook_func(__VA_ARGS__)

DECL_FUNC_HOOK(SceBt_ksceBtReadEvent, SceBtEvent *events, int num_events)
{
	int ret = TAI_CONTINUE(int, SceBt_ksceBtReadEvent_ref, events, num_events);

	if (ret >= 0)
    {
		//LOG("Communication detected: %d events\n", num_events);

        for (int i = 0 ; i < num_events ; i++)
        {
            SceBtEvent* event = &events[i];
            //LOG("Connection event %d with %d %d\n", event->id, event->mac0, event->mac1);
            if (!ds4_connected && 0x05 == event->id)
            {
                unsigned short vid_pid[2];
                ksceBtGetVidPid(event->mac0, event->mac1, vid_pid);
                //LOG("Vendor ID %d ; Product ID %d\n", vid_pid[0], vid_pid[1]);

                if (is_ds4(vid_pid))
                {
                    LOG("DualShock 4 connected\n");
                    log_flush();

                    ds4_mac0 = event->mac0;
                    ds4_mac1 = event->mac1;
                    ds4_input_reset();
                    ds4_connected = 1;
                    
                    dsResetAccelGyroSampling();
                }
            }
            else if (ds4_connected && event->mac0 == ds4_mac0 && event->mac1 == ds4_mac1)
            {
                if (0x06 == event->id)
                {
                    ds4_connected = 0;
                    
                    LOG("DualShock 4 disconnected\n");
                    log_flush();
                }
                else if (NULL != recv_buff)
                {
                    if (0x0A == event->id)
                    {
                        if (0x11 == recv_buff[0])
                        {
                            memcpy(&ds4_input, recv_buff, sizeof(ds4_input));
                            //LOG("Timestamp: %d, %d\n", ds4_input.cnt2, ds4_input.cnt3);
                            //LOG("Accel: %d, %d, %d\n", ds4_input.accel_x, ds4_input.accel_y, ds4_input.accel_z);
                            //LOG("Gyro: %d, %d, %d\n", ds4_input.gyro_x, ds4_input.gyro_y, ds4_input.gyro_z);
                            //log_flush();
                            
                            accel_x_sum += ds4_input.accel_x;
                            accel_y_sum += ds4_input.accel_y;
                            accel_z_sum += ds4_input.accel_z;

                            gyro_x_sum += ds4_input.gyro_x;
                            gyro_y_sum += ds4_input.gyro_y;
                            gyro_z_sum += ds4_input.gyro_z;
                            
                            samplesCount++;
                        }
                        recv_buff = NULL;
                    }
                    else if (0x0B == event->id)
                        recv_buff = NULL;
                }
            }
        }
	}

	return ret;
}

DECL_FUNC_HOOK(SceBt_ksceBtHidTransfer, unsigned int mac0, unsigned int mac1, SceBtHidRequest *request)
{
	int ret = TAI_CONTINUE(int, SceBt_ksceBtHidTransfer_ref, mac0, mac1, request);

    if (ret >= 0 && ds4_connected && mac0 == ds4_mac0 && mac1 == ds4_mac1)
    {
        if (NULL != request && NULL != request->buffer && request->length >= sizeof(ds4_input))
        {
            recv_buff = (unsigned char*)request->buffer;
            //LOG("Buffer %x ready for answer\n", (unsigned int)recv_buff);
            //log_flush();
        }
        else
            recv_buff = NULL;
    }
    
    return ret;
}

void _start() __attribute__ ((weak, alias ("module_start")));

#define BIND_FUNC_EXPORT_HOOK(name, pid, module, lib_nid, func_nid) \
	name##_hook_uid = taiHookFunctionExportForKernel((pid), \
		&name##_ref, (module), (lib_nid), (func_nid), name##_hook_func)

int module_start(SceSize argc, const void *args)
{
	int ret;
	tai_module_info_t SceBt_modinfo;

	log_reset();

	LOG("ds4hook by OperationNT\n");

	SceBt_modinfo.size = sizeof(SceBt_modinfo);
	ret = taiGetModuleInfoForKernel(KERNEL_PID, "SceBt", &SceBt_modinfo);
	if (ret < 0) {
		LOG("Error finding SceBt module\n");
		goto error_find_scebt;
	}

	/* SceBt hooks */
	BIND_FUNC_EXPORT_HOOK(SceBt_ksceBtReadEvent, KERNEL_PID,
		"SceBt", TAI_ANY_LIBRARY, 0x5ABB9A9D);
    LOG("ksceBtReadEvent hook result: %x\n", SceBt_ksceBtReadEvent_hook_uid);

	BIND_FUNC_EXPORT_HOOK(SceBt_ksceBtHidTransfer, KERNEL_PID,
		"SceBt", TAI_ANY_LIBRARY, 0xF9DCEC77);
    LOG("ksceBtHidTransfer hook result: %x\n", SceBt_ksceBtHidTransfer_hook_uid);
    
	LOG("module_start finished successfully!\n");
    
    log_flush();

	return SCE_KERNEL_START_SUCCESS;

error_find_scebt:
	return SCE_KERNEL_START_FAILED;
}

#define UNBIND_FUNC_HOOK(name) \
	do { \
		if (name##_hook_uid > 0) { \
			taiHookReleaseForKernel(name##_hook_uid, name##_ref); \
		} \
	} while(0)

int module_stop(SceSize argc, const void *args)
{
	UNBIND_FUNC_HOOK(SceBt_ksceBtReadEvent);
    UNBIND_FUNC_HOOK(SceBt_ksceBtHidTransfer);

	log_flush();

	return SCE_KERNEL_STOP_SUCCESS;
}
