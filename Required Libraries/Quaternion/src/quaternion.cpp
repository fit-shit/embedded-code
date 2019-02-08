#include "quaternion.h"

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

quaternion::quaternion()
{
	w = 1.0;
	x = 0.0;
	y = 0.0;
	z = 0.0;
}

void quaternion::normalizeMagnitude() {
	float magnitude = this->getMagnitude();
	w /= magnitude;
	x /= magnitude;
	y /= magnitude;
	z /= magnitude;
}

float quaternion::getMagnitude() {
	float ww = w*w;
	float xx = x*x;
	float yy = y*y;
	float zz = z*z;
	float sum = ww + xx + yy + zz;
	float magnitude = sqrt(sum);

	return magnitude;
}

quaternion quaternion::getInverted() {
	quaternion result = quaternion();

	const float quaternionMagSq = w*w + x*x + y*y + z*z;
	result.w = w / quaternionMagSq;
	result.x = -1 * x / quaternionMagSq;
	result.y = -1 * y / quaternionMagSq;
	result.z = -1 * z / quaternionMagSq;

	return result;
}

quaternion quaternion::multiply(quaternion p) {
	quaternion result = quaternion();

	result.w = w*p.w - x*p.x - y*p.y - z*p.z;
	result.x = x*p.w + w*p.x - z*p.y + y*p.z;
	result.y = y*p.w + z*p.x + w*p.y - x*p.z;
	result.z = z*p.w - y*p.x + x*p.y + w*p.z;
  
	return result;
}

quaternion quaternion::getRotated(quaternion w) {
	quaternion w_inv = quaternion();
  	quaternion a_world = quaternion();
  	quaternion w_a = quaternion();
  	quaternion a_body = quaternion();

  	a_body.set(this->w, this->x, this->y, this->z);
	w_inv = w.getInverted();

  	w_a = w.multiply(a_body);
  	a_world = w_a.multiply(w_inv);

	return a_world;
}

quaternion quaternion::undoRotation(quaternion w) {
	quaternion w_inv = quaternion();
	quaternion a_body = quaternion();
	quaternion w_inv_a = quaternion();
	quaternion a_world = quaternion();

	a_world.set(this->w, this->x, this->y, this->z);
	w_inv = w.getInverted();

	w_inv_a = w_inv.multiply(a_world);
	a_body = w_inv_a.multiply(w);

	return a_body;
}

void quaternion::set(float new_w, float new_x, float new_y, float new_z) {
	this->w = new_w;
	this->x = new_x;
	this->y = new_y;
	this->z = new_z;
}

void quaternion::set(quaternion q) {
	this->w = q.w;
	this->x = q.x;
	this->y = q.y;
	this->z = q.z;
}

