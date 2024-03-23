#include <Adafruit_BMP3XX.h>
#include <Adafruit_MPU6050.h>
#include <SD.h>



Adafruit_BMP3XX bmp;

Adafruit_MPU6050 mpu;

File myFile;

int state = 0;
int numLoop = 0;
float GroundLevelPressure;

void setup()
{
  Serial.begin(9600);

  while (!Serial)
    delay(10); 


  if (!SD.begin(10))
  {
    Serial.println(F("initialization failed!"));
    while (1)
      ;
  }
  SD.remove(F("test.txt"));

  
  if (!bmp.begin_I2C())
  { 
    Serial.println(F("Could not find a valid BMP3 sensor, check wiring!"));
    
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);



  if (!mpu.begin())
  {
    Serial.println(F("Failed to find MPU6050 chip"));
   
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  Serial.println(F("initialization done."));

  delay(100);
}


void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


 
  
  if (! bmp.performReading()) {
    Serial.println(F("Failed to perform reading"));
  }
  
  if (state == 0 & numLoop > 0)
  {
    GroundLevelPressure = bmp.pressure / 100.0;
    state++;
  }

  if (state > 0)
  {
    myFile = SD.open(F("test.txt"), FILE_WRITE);
    if (myFile)
    {
      myFile.print(F("Time: "));
      myFile.print(((numLoop-1) * 100)/1000);
      myFile.print(F(" s Alt: "));
      myFile.print(bmp.readAltitude(GroundLevelPressure));
      //myFile.print(" m Ax: " + a.acceleration.x +  +  a.acceleration.y + " m/s^2 Az: " +  a.acceleration.z + " m/s^2");
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
  
  delay(100);
}
