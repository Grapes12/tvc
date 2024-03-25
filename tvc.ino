#include <Adafruit_BMP3XX.h>
#include <Adafruit_MPU6050.h>
#include <SD.h>
#include <Servo.h>

Adafruit_BMP3XX bmp;

Adafruit_MPU6050 mpu;

File myFile;

int state = 0;
int numLoop = 0;
float GroundLevelPressure;



Servo myservo;  // create servo object to control a servo
Servo myservo2;
// twelve servo objects can be created on most boards

int serv1 = 90;    // variable to store the servo position
int serv2 = 90;

double angleX = 0;
double angleY = 0;
double angleZ = 0;

typedef struct PID{
    double kp;
    double ki;
    double kd;
    double sp;
    double error_last;
    double integral_error;
    double saturation_max;
    double saturation_min;
    double dt;

    PID(double k_p, double k_i, double k_d, int max, int min, double target = 0, float time_step = 0.05) {
        kp = k_p;
        ki = k_i;
        kd = k_d;
        sp = target;
        saturation_max = max;
        saturation_min = min;
        dt = time_step;
        error_last = 0;
        integral_error = 0;
    }
};

PID myPID(0.45,0.2,0.1, 30, -30, 0, 0.05);

void setup() {
    Serial.begin(115200);

    while (!Serial)
        delay(10);

    if (!SD.begin(10)) {
        Serial.println(F("initialization failed!"));
    }
    SD.remove(F("test.txt"));

    if (!bmp.begin_I2C()) {
        Serial.println(F("Could not find a valid BMP3 sensor, check wiring!"));
    }

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    if (!mpu.begin()) {
        Serial.println(F("Failed to find MPU6050 chip"));
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println(F("initialization done."));

    myservo.attach(4);  // attaches the servo on pin 9 to the servo object
    myservo2.attach(5);


    while (Serial.available() == 0) {
    }

     int menuChoice = Serial.parseInt();
    delay(100);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if (!bmp.performReading()) {
        Serial.println(F("Failed to perform reading"));
    }

    if (state == 0 & numLoop > 0) {
        GroundLevelPressure = bmp.pressure / 100.0;
        state++;
    }

    /*
      WRITING TO SD CARD CODE
      if (state > 0)
      {
        myFile = SD.open(F("test.txt"), FILE_WRITE);
        if (myFile)
        {
          myFile.print(F("Time: "));
          myFile.print(((numLoop - 1) * 100) / 1000);
          myFile.print(F(" s Alt: "));
          myFile.print(bmp.readAltitude(GroundLevelPressure));
          // myFile.print(" m Ax: " + a.acceleration.x +  +  a.acceleration.y + " m/s^2 Az: " +  a.acceleration.z + " m/s^2");
          myFile.print(F(" m Ax: "));
          myFile.print(a.acceleration.x);
          myFile.print(F(" m/s^2 Ay: "));
          myFile.print(a.acceleration.y);
          myFile.print(F(" m/s^2 Az: "));
          myFile.print(a.acceleration.z);
          myFile.println(F(" m/s^2"));
          myFile.close();
        }
        else
        {
          Serial.println(F("error opening test.txt"));
        }
      }

      numLoop++;
     */

    // integrate readings twice to get attitude

    // call PID function twice (for pitch and yaw)
    
    // send output to servo

    // milliseconds
    /*
    
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        neg -= 1;
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        myservo2.write(pos);
        delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
        neg += 1;
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        myservo2.write(pos);
        delay(15);                       // waits 15ms for the servo to reach the position
    }
    */
    
    angleX = angleX + g.gyro.x * 0.05;
    angleY = angleY + g.gyro.y * 0.05;
    angleZ = angleZ + g.gyro.z * 0.05;


    Serial.print("X: ");
    Serial.println(angleX); 
    Serial.print("Y: ");
    Serial.println(angleY);

    myservo.write(getPID(&myPID, angleX, 0.05) + serv1);
    myservo2.write(getPID(&myPID, angleY, 0.05) + serv2);

    serv1 = getPID(&myPID, angleX, 0.05) + serv1;
    serv2 = getPID(&myPID, angleY, 0.05) + serv2;


    Serial.println(getPID(&myPID, angleX, 0.05));
    Serial.println(getPID(&myPID, angleY, 0.05));
    
    
    delay(50);
}

double getPID(PID *pid, double angle, double dt) {
    // IMPLEMENT PID.PY here with P = 0.5, I = 0.1, D = 0.5 and +/-15 degrees max and min
    double error = pid->sp - angle;
    double derivative_error = (error - pid->error_last) / dt;
    pid->integral_error += error * dt;
    double output = pid->kp * error + pid->ki * pid->integral_error + pid->kd * derivative_error;
    pid->error_last = error;

    // max or min of zero means no max or min defined

    if (pid->saturation_max != 0 && output > pid->saturation_max) {
        output = pid->saturation_max;
    } else if (pid->saturation_min != 0 && output < pid->saturation_min) {
        output = pid->saturation_min;
    }

    return output;
}

double getServoOutput(double actuatorCom) {
    // returns the servo output given a certain angle deflection by getPID()
    return;
}