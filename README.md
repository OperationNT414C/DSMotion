# DSMotion

Henkaku plugins which adds motion control support for PlayStation TV with DualShock 4 controllers

There are 2 plugins:
 * Kernel plugin **dsmotion.skprx**: it hooks BlueTooth calls to catch DualShock controller packets and exposes user services to get the intercepted motion data
 * User plugin **dsmotion.suprx**: it calls the kernel plugin services and reinterprets the given data for SceMotion functions return


### Installation

In order to activate those plugins featues, you must edit your `ux0:tai/config.txt` file:

```
*KERNEL
ux0:tai/dsmotion.skprx

*TITLEID00
ux0:tai/dsmotion.suprx
```

Replace **TITLEID00** by a title identifier which needs motion control or by **ALL** to affect all titles.


### Limitations

 * It doesn't work well on classic PS Vita with ds4vita: for an unknown reason, motion control samples seems to be too much spaced over time
 * It hooks documented "SceMotion" user functions instead of undocumented "SceMotionDev" kernel functions: if we could understand those kernel functions, we could have more compatibility with a single kernel plugin (no more need for a user plugin)
 * It doesn't try to compute orientation gyroscope and acceleration data so only pitch rotation is supported on some returned data: it seems to be enough for most cases, please let me know if you find a game which would need orientation computation

 
### Compatibility

 * PCSF00214 - Tearaway - Parts where you need to shake the PS Vita works now by shaking the controller!


### Credits

 * Most credit goes to **xerpi** for his "ds3vita" and "ds4vita" plugins source code which helped me a lot!
 
