#ifndef DSMotionLibrary_H
#define DSMotionLibrary_H

int dsResetAccelGyroSampling();
int dsGetSampledAccelGyro(signed short oAccel[3], signed short oGyro[3]);
int dsGetInstantAccelGyro(signed short oAccel[3], signed short oGyro[3]);

#endif
