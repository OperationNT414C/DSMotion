## 1.1

User plugin changes:
 * Change cross product way for device quaternion to fix some vertical inversion
 * Remove useless hook on "sceMotionGetBasicOrientation": this function always returns (0,1,0) even on PS Vita

Documentation changes:
 * Compatibility list updated (special thanks to "stranno" who made a lot of tests!)
 * Limitation on signed titles removed: after some tests on original titles, it works fine
 * New limitation discovered: apparently, PS Vita can activate a magnometer which is used specially by games with horizontal conrol issues


## 1.0

Initial release
