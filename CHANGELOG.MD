## 1.2

User plugin changes:
 * Fix device orientation computation: highly improve camera stability and fix some inverted axis issues

Documentation changes:
 * New limitation discovered: don't turn your controller upside down or the orientation computation will go wrong
 * Explanation about some perceived inverted axis issues: please try to test the game behavior on a real PS Vita before reporting issues on this subject!


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
