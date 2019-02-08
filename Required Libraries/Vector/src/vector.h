#ifndef __vector_H__
#define __vector_H__

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class vector
{
public:
	float x,y,z;

	// quaternion -- quaternion constructor
	vector();


	// void functions
	void normalizeMagnitude();


	// get functions
	float getMagnitude();

	vector crossProduct(vector v);


	// set functions
	void set(float x, float y, float z);
	
	void set(vector v);
};

#endif // vector_H //