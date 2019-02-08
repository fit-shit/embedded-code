float calculateMagnitude(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

float normalizeToMagnitude(float val, float mag) {
  if (mag == 0.0) return 0.0;
  return val / mag;
}

float calculateQuaternionMagnitude(float q[]) {
  return sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
}
