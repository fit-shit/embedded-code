float butterworthModFilter(float currentVal, float prevVal, byte forSensor) {
  //byte forSensor values: Gyro == 0, Accel == 1, Mag == 2
  
  float a, b, N;
  float difference = currentVal - prevVal;
  
  if(forSensor == 0) {
    a = 2.5;
    b = 0.2;
    N = 12.0;
  } else if (forSensor == 1) {
    a = 0.1;
    b = 0.001;
    N = 6.0;
  } else {
    a = 0.02;
    b = 0.001;
    N = 12.0;
  }
  
  float multiplier = -1 / sqrt(pow((abs(difference) / a) - b, 2 * N) + 1) + 1;
  delay(2);
  
  if (multiplier < 0.1 && forSensor == 0) {
    return 0.0f;
  } else {
    return prevVal + (multiplier * difference);
  }
}
