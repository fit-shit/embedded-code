#include "vector.h"

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

vector::vector()
{
	x = 0.0;
	y = 0.0;
	z = 0.0;
}

void vector::normalizeMagnitude() {
	float magnitude = this->getMagnitude();
	x /= magnitude;
	y /= magnitude;
	z /= magnitude;
}

float vector::getMagnitude() {
	float xx = x*x;
	float yy = y*y;
	float zz = z*z;
	float sum = xx + yy + zz;
	float magnitude = sqrt(sum);

	return magnitude;
}

vector vector::crossProduct(vector v) {
	vector result = vector();
	
	result.x = y*v.z - z*v.y;
	result.y = z*v.x - x*v.z;
	result.z = x*v.y - y*v.x;

	return result;
}


void vector::set(float new_x, float new_y, float new_z) {
	this->x = new_x;
	this->y = new_y;
	this->z = new_z;
}

void vector::set(vector v) {
	this->x = v.x;
	this->y = v.y;
	this->z = v.z;
}

