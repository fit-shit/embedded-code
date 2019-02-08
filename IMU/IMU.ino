#include <quaternion.h>
#include <vector.h>
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
bool TESTING_HARDWARE = false;


float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
float ax, ay, az, gx, gy, gz, mx, my, mz;
float pax, pay, paz, pgx, pgy, pgz, pmx, pmy, pmz;
float temperature;

float gReading;

float currReadings[9];
float prevReadings[9];

unsigned long prevTime = 0;
unsigned long currentTime = 0;

quaternion q_t, q_dt;

vector n_x, n_y, n_z;


vector axisRotation;
quaternion q_previous_w;
quaternion q_previous_a;

vector base_position;
vector base_velocity;

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
  
  if (digitalRead(INT1XM)) {
    dof.readAccel();
    pax = dof.calcAccel(dof.ax) - abias[0];
    pay = dof.calcAccel(dof.ay) - abias[1];
    paz = dof.calcAccel(dof.az) - abias[2];
  }

  Serial.println("\nCalibrating g vector...");

  int totalConsistencyCounter = 0;
  float differenceCuttoff = 0.01;
  
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



  n_x = vector();
  n_y = vector();
  n_z = vector();
  
  n_x.set(1.0,0.0,0.0);
  n_y.set(0.0,1.0,0.0);
  n_z.set(0.0,0.0,1.0);

  q_t = quaternion();
  q_dt = quaternion();

  axisRotation.set(0.0,0.0,0.0);
  q_previous_w.set(1.0,0.0,0.0,0.0);
  q_previous_a.set(2.0,0.0,0.0,1.0);
  base_position.set(0.0,0.0,0.0);
  base_velocity.set(0.0,0.0,0.0);
  
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
  
  vector gyro = vector();
  vector accel = vector();
  vector unfilteredAccel = vector();

  vector accel_scales;
  vector accel_biases;
  vector gyroScale = vector();
  accel_scales.set(0.959432, 1.0322581, 0.9791723);
  accel_biases.set(0.082649, 0.0652722, 0.0104886);
  gyroScale.set(1.127,1.142857,1.12852);
 
  
  gyro.set(currReadings[3]*0.01745329252*gyroScale.x, currReadings[4]*0.01745329252*gyroScale.y, currReadings[5]*0.01745329252*gyroScale.z);
  accel.set((currReadings[0]*accel_scales.x)+accel_biases.x, (currReadings[1]*accel_scales.y)+accel_biases.y, (currReadings[2]*accel_scales.z)+accel_biases.z);
  accel.normalizeMagnitude();
  
  unfilteredAccel.set(currReadings[0], currReadings[1], currReadings[2]);
  
  
  //
  // GYROSCOPE DATA
  //
  //
  
  if (prevTime != 0) {
    const float dt = float(currentTime - prevTime) / 1000000;

    quaternion q_dt = quaternion();
    quaternion q_w_t_dt = quaternion();
    quaternion q_a = quaternion();
    quaternion q_a_world = quaternion();
    quaternion NX = quaternion();
    quaternion NY = quaternion();
    quaternion NZ = quaternion();


    axisRotation.x += gyro.x*dt*gyroScale.x;
    axisRotation.y += gyro.y*dt*gyroScale.y;
    axisRotation.z += gyro.z*dt*gyroScale.z;
   
    vector NX_final, NY_final, NZ_final;

    float omega_mag = gyro.getMagnitude();
    float omega_mag_dt_2 = omega_mag * dt / 2;
    float sin_omega_mag_dt_2 = sin(omega_mag_dt_2);

    float n1 = normalizeToMagnitude(gyro.x, omega_mag),
          n2 = normalizeToMagnitude(gyro.y, omega_mag),
          n3 = normalizeToMagnitude(gyro.z, omega_mag);


    q_dt.set(cos(omega_mag_dt_2), n1*sin_omega_mag_dt_2, n2*sin_omega_mag_dt_2, n3*sin_omega_mag_dt_2);

    q_w_t_dt = q_t.multiply(q_dt);

    q_a.set(0.0, accel.x, accel.y, accel.z);
    q_a_world = q_a.getRotated(q_w_t_dt);
    
    vector q_a_world_norm = vector();
    q_a_world_norm.set(q_a_world.x,q_a_world.y,q_a_world.z);
    q_a_world_norm.normalizeMagnitude();

    
    
    const float alpha = 1.0;
    quaternion q_a_correction;

    vector v_n = vector();
    vector v_g = vector();
    v_g.set(0.0,0.0,1.0);
    v_n = q_a_world_norm.crossProduct(v_g);
    
    q_a_correction.set(q_a_world_norm.z*(1+alpha), v_n.x, v_n.y, v_n.z); 
    q_a_correction.normalizeMagnitude();

    NX = vectorToQuaternion(n_x);
    NY = vectorToQuaternion(n_y);
    NZ = vectorToQuaternion(n_z);

    quaternion NX_temp = NX.getRotated(q_w_t_dt);
    quaternion NY_temp = NY.getRotated(q_w_t_dt);
    quaternion NZ_temp = NZ.getRotated(q_w_t_dt);
    
    NX_final = quaternionToVector(NX_temp.getRotated(q_a_correction));
    NY_final = quaternionToVector(NY_temp.getRotated(q_a_correction));
    NZ_final = quaternionToVector(NZ_temp.getRotated(q_a_correction));
    

    // REMOVE GRAVITATIONAL VECTOR -----------------------------------------------------

    vector effective_a = vector();
    quaternion q_predicted_g = q_previous_a.getRotated(q_w_t_dt); // rotate the previous body 'a' into the global cord. by the current rotation 
    vector error = vector();
    error.set(q_a_world.x-q_predicted_g.x, q_a_world.y-q_predicted_g.y, q_a_world.z-q_predicted_g.z);
    float error_mag = error.getMagnitude();

    //Serial.println(error_mag,7);
    if (error_mag > 0.1) {
      //  If sufficient error exists, set the previous 

      
      effective_a.set(-1*error.x,-1*error.y,-1*error.z);

      base_velocity.x += effective_a.x * dt; 
      base_velocity.y += effective_a.y * dt; 
      base_velocity.z += effective_a.z * dt;   

      // base_position.x += (base_velocity.x * dt)*100;
      // base_position.y += (base_velocity.y * dt)*100;
      // base_position.z += (base_velocity.z * dt)*100;

      //Serial.println(error_mag);
    } else {
      //  If error doesn't exist, reset previous 'a' quaternion to current body accel values
      
      
      effective_a.set(0,0,0);

      base_velocity.set(0.0,0.0,0.0);
    }

    q_previous_a.set(0.0, accel.x, accel.y, accel.z); 







    vector nadda = vector();
    vector a_world = vector();
    a_world.set(q_a_world.x,q_a_world.y,q_a_world.z);
    float a_world_mag = a_world.getMagnitude();

    Serial.println(acos(a_world.z / a_world_mag)*180/PI);

    //if (a_world_mag != 1.0) Serial.println(a_world_mag,7);
    //printUnitVectors(quaternionToVector(q_predicted_g), a_world, nadda);
    
    

    // PRINT SOME SHIT HERE ------------------------------------------------------------

    
//    printQuaternion(vectorToQuaternion(q_a_world_norm),true);

//    printUnitVectors(NX_final, NY_final, NZ_final);

    
    q_t = q_w_t_dt;

  } else {
    q_previous_a.set(0.0, accel.x, accel.y, accel.z); 
  }
  
  prevTime = prevTime == 0 ? micros() : currentTime;
}



vector quaternionToVector(quaternion q) {
  vector v;

  v.x = q.x;
  v.y = q.y;
  v.z = q.z;

  return v;
}

quaternion vectorToQuaternion(vector v) {
  quaternion q;
  
  q.w = 0.0;
  q.x = v.x;
  q.y = v.y;
  q.z = v.z;

  return q;
}

void printQuaternion(quaternion q, bool forProcessing) {
  String seperator = forProcessing == true ? "," : "\t\t";
  Serial.print(q.w);
  Serial.print(seperator);
  Serial.print(q.x);
  Serial.print(seperator);
  Serial.print(q.y);
  Serial.print(seperator);
  Serial.print(q.z);
  Serial.print("\n");
}

void printUnitVectors(vector x, vector y, vector z) {
  String seperator = ",";
  Serial.print(x.x);
  Serial.print(seperator);
  Serial.print(x.y);
  Serial.print(seperator);
  Serial.print(x.z);
  Serial.print(seperator);
  Serial.print(y.x);
  Serial.print(seperator);
  Serial.print(y.y);
  Serial.print(seperator);
  Serial.print(y.z);
  Serial.print(seperator);
  Serial.print(z.x);
  Serial.print(seperator);
  Serial.print(z.y);
  Serial.print(seperator);
  Serial.print(z.z);
  Serial.print("\n");
}
