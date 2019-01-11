float calculateMagnitude(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

float normalizeToMagnitude(float val, float mag) {
  return val / mag;
}
