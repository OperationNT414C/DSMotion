#ifndef DSMotionLibrary_H
#define DSMotionLibrary_H

struct accelGyroData
{
    signed short accel[3];
    signed short gyro[3];
    unsigned int timestamp;
    unsigned int counter;
};

unsigned int dsGetCurrentTimestamp();
unsigned int dsGetCurrentCounter();

int dsResetAccelGyroSampling();
int dsGetSampledAccelGyro(signed short oAccel[3], signed short oGyro[3]);
int dsGetInstantAccelGyro(unsigned int iIndex, struct accelGyroData* oData);

#endif
