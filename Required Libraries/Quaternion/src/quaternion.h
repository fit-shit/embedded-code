#ifndef __quaternion_H__
#define __quaternion_H__

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class quaternion
{
public:
	float w,x,y,z;

	// quaternion -- quaternion constructor
	quaternion();


	// void functions
	void normalizeMagnitude();


	// get functions
	float getMagnitude();

	quaternion getInverted();

	quaternion multiply(quaternion p); 

	quaternion getRotated(quaternion w);

	quaternion undoRotation(quaternion w);


	// set functions
	void set(float w, float x, float y, float z);
	
	void set(quaternion q);
};

#endif // quaternion_H //