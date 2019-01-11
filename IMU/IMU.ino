#include <SPI.h>
#include <Wire.h>
#include <SFE_LSM9DS0.h>

#include "filters.h"
#include "math_functions.h"

#define LSM9DS0_XM 0x1D
#define LSM9DS0_G  0x6B

LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

const byte INT1XM = 7;
const byte INT2XM = 8;
const byte DRDYG = 9;

const byte LEDRED = 6;
const byte LEDGREEN = 5;

// Set to TRUE to disable printing processing data to console.
const bool TESTING_HARDWARE = true;



float pitch, yaw, roll;
float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
float ax, ay, az, gx, gy, gz, mx, my, mz;
float pax, pay, paz, pgx, pgy, pgz, pmx, pmy, pmz;
float temperature;

float gReading;

float currReadings[9];
float prevReadings[9];

float gamma = 0.0, 
      beta = 0.0, 
      theta = 0.0;
      
float z_pos = 0.0, z_vel = 0.0, z_offset = 0.0;

unsigned long prevTime = 0, 
              currentTime = 0;
              


void setup() {
  Serial.begin(38400);
  
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG,  INPUT);
  
  pinMode(LEDRED, OUTPUT);
  pinMode(LEDGREEN, OUTPUT);
  digitalWrite(LEDGREEN, HIGH);
  
  uint32_t status = dof.begin();
  delay(200);
  
  dof.setAccelScale(dof.A_SCALE_2G);
  dof.setGyroScale(dof.G_SCALE_245DPS);
  dof.setMagScale(dof.M_SCALE_2GS);
  
  dof.setAccelODR(dof.A_ODR_200);
  dof.setAccelABW(dof.A_ABW_50);
  dof.setGyroODR(dof.G_ODR_190_BW_125);
  dof.setMagODR(dof.M_ODR_125);
  
  dof.calLSM9DS0(gbias, abias); 
  
  delay(200);
  
  digitalWrite(LEDGREEN, LOW);
  digitalWrite(LEDRED, HIGH);
 
  //determine init value of g
  float gReadingDifference = 1.0;
  float gReadingPrevious = 1.0;
  int consistencyCounter = 0;
  
  
  Serial.print("\n\n\nHENLO -----\n\n");
  
  if (digitalRead(INT1XM)) {
    dof.readAccel();
    pax = dof.calcAccel(dof.ax) - abias[0];
    pay = dof.calcAccel(dof.ay) - abias[1];
    paz = dof.calcAccel(dof.az) - abias[2];
  }
  
  Serial.print(pax);
  Serial.print(", ");
  Serial.print(pay);
  Serial.print(", ");
  Serial.print(paz);

  Serial.println("\nCalibrating g vector...");

  int totalConsistencyCounter = 0;
  float differenceCuttoff = 0.006;
  
  while (consistencyCounter <= 50) {
    //
    //  G VECTOR CALIBRATION
    //  -----------------------------------------------------------------------------------------------
    //  This while loop serves as an accelerometer calibration method to determine the module's
    //  average inherent gravitational vector (g) bias. The red LED will flash durring this proccess.
    //
    //  The method will read the magnitude of the g vector and compare it to the previously determined
    //  g value. When the difference between the current and previously calculated magnitudes become
    //  less than the value dictated by 'differneceCutoff' for 50 iterations, the module is considered
    //  calibrated and the while loop will exit.
    //
    
    digitalWrite(LEDRED, HIGH);
    
    if (digitalRead(INT1XM)) {
      dof.readAccel();
      ax = dof.calcAccel(dof.ax) - abias[0];
      ay = dof.calcAccel(dof.ay) - abias[1];
      az = dof.calcAccel(dof.az) - abias[2];
    }
    
    // Calculate magnitude and take average of current and last values (IIR).
    gReading = (calculateMagnitude(ax, ay, az) + gReadingPrevious) / 2;
    gReadingDifference = gReading - gReadingPrevious;
    gReadingPrevious = gReading;
        
    // Advance the consistencyCounter value if the difference falls below differenceCutoff. 
    // Otherwise, reset to 0.
    consistencyCounter = abs(gReadingDifference) <= differenceCuttoff ? consistencyCounter + 1 : 0;
    
    // Adjust the differenceCutoff value if calibration can't be achieved after 200 iterations.
    totalConsistencyCounter++;
    if (totalConsistencyCounter >= 200) {
       differenceCuttoff += 0.002;
       totalConsistencyCounter = 0;
       Serial.print("Adjusting differenceCutoff to ");
       Serial.println(differenceCuttoff, 5);
    }
    
    delay(20);
    digitalWrite(LEDRED, LOW);
    
  }
  
  Serial.print("Final g value is: ");
  Serial.println(gReading, 8);
  
  digitalWrite(LEDGREEN, HIGH);
  digitalWrite(LEDRED, LOW);
  
}


void loop() {

  if (prevTime != 0) currentTime = micros();
  
  //
  // READ RAW DATA FROM LSM9DS0 IMU 
  //
  //
  
  if (digitalRead(DRDYG)) {
    dof.readGyro();
    gx = dof.calcGyro(dof.gx) - gbias[0];
    gy = dof.calcGyro(dof.gy) - gbias[1];
    gz = dof.calcGyro(dof.gz) - gbias[2];
  }
  
  if (digitalRead(INT1XM)) {
    dof.readAccel();
    ax = dof.calcAccel(dof.ax) - abias[0];
    ay = dof.calcAccel(dof.ay) - abias[1];
    az = dof.calcAccel(dof.az) - abias[2];
  }
  
  if (digitalRead(INT2XM)) {
    dof.readMag();
    mx = dof.calcMag(dof.mx);
    my = dof.calcMag(dof.my);
    mz = dof.calcMag(dof.mz);
    
    dof.readTemp();
    temperature = 21.0 + (float) dof.temperature / 8;
  }
  
  float currReadings[] = {ax, ay, az, gx, gy, gz, mx, my, mz};
  float prevReadings[] = {pax, pay, paz, pgx, pgy, pgz, pmx, pmy, pmz};
 
 
 
 
 
 
  //
  //  FILTER RAW DATA
  //
  // 
 
  for (int i = 0; i < 9; i++) {
    byte sensorCode;
    int incrimentNum = i / 3;
    if (incrimentNum == 0)
      sensorCode = 1;
    else if (incrimentNum == 1)
      sensorCode = 0;
    else
      sensorCode = 2;
    
    currReadings[i] = butterworthModFilter(currReadings[i], prevReadings[i], sensorCode);
    
    prevReadings[i] = currReadings[i];
  }
  
  
  
  
  
  
  //
  // ACCELEROMETER DATA
  //
  //
  const float filteredMagnitude = calculateMagnitude(currReadings[0], currReadings[1], currReadings[2]);
  const float adj_ax = normalizeToMagnitude(currReadings[0], filteredMagnitude),
              adj_ay = normalizeToMagnitude(currReadings[1], filteredMagnitude),
              adj_az = normalizeToMagnitude(currReadings[2], filteredMagnitude);
  const float adj_mg = calculateMagnitude(adj_ax, adj_ay, adj_az);
  
  const float filterCutoff = 0.001;
  // const float gamma = asin(adj_ax);
  // const float beta = asin(adj_ay);
  
  if (prevTime != 0) {
    const float timeDiff = float(currentTime - prevTime) / 1000000;
        
    if (filteredMagnitude > (gReading + filterCutoff) || filteredMagnitude < (gReading - filterCutoff)) {
      float tempVel = z_vel + ((currReadings[2] - z_offset) * 9.8 * timeDiff);
      
      z_vel = butterworthModFilter(tempVel, z_vel, 1); //(currReadings[2] - z_offset) * 9.8 * timeDiff;
      if (abs(z_vel) < 0.04) z_vel = 0.0; 
      z_pos += z_vel * timeDiff * 500;
    }
    
    if (abs(z_vel) < 0.04) z_vel = 0.0; 
    
    Serial.print("A: ");
    Serial.print(currReadings[2] - z_offset, 3);
    Serial.print("\t\t V: ");
    Serial.print(z_vel, 3);
    Serial.print("\t\t P: ");
    Serial.print(z_pos, 3);
    Serial.print("\n");
  } else {
    z_offset = currReadings[2]; 
  }
  
  if (filteredMagnitude >  (gReading + filterCutoff) || filteredMagnitude < (gReading - filterCutoff))
  {
    digitalWrite(LEDRED, HIGH); 
  } else {
    digitalWrite(LEDRED, LOW); 
  }
  
  
  
  
  
  
  
  //
  // GYROSCOPE DATA
  //
  //
  
  if (prevTime != 0) {
    // If prevTime has been set (ie loop has run more than once), convert raw angular speed
    // values into degree value for each axis.
    const float timeDiff = float(currentTime - prevTime) / 1000000;
    const float tempGamma = currReadings[3] * timeDiff;
    const float tempBeta = currReadings[4] * timeDiff;
    const float tempTheta = currReadings[5] * timeDiff;
    
    // Integrate with current axis angle (degrees)
    gamma += tempGamma;
    beta += tempBeta;
    theta += tempTheta;
  }
  
  
  
  
  
  
  
  //
  // TRANSFER DATA OVER SERIAL TO PROCESSING
  //
  //
  
  // Convert angles from degrees to radians
  const float beta_rad = beta * 0.0174533;
  const float gamma_rad = gamma * 0.0174533;
  const float theta_rad = theta * 0.0174533;
  
  if (TESTING_HARDWARE == false) {
    Serial.print(beta_rad, 5);
    Serial.print(",");
    Serial.print(gamma_rad, 5);
    Serial.print(",");
    Serial.print(theta_rad, 5);
    Serial.print(",");
    Serial.print(z_pos, 5);
    Serial.print("\n");
  }
  
  prevTime = prevTime == 0 ? micros() : currentTime;
}




