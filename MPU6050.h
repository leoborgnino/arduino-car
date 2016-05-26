#include <Wire.h>

void init_MPU6050()
{
  
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // PWR_MGMT_1 registro a cero
  Wire.write(0); // se saca el MPU del sleep
  Wire.endTransmission(true);
  
}

void obtener_datos(float *datos, float *offset)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,14,true); // request a total of 14 registers
  
  for( int i = 0; i < 7; i++ )
    {
      datos[i] = Wire.read()<<8|Wire.read();
    }
  
  for( int i = 0; i < 3; i++)
    {
      if (datos[i] >= 0x8000) datos[i] = -((65535 - datos[i])+1);
      datos[i] = datos[i] / 16384.0;
      if (datos[i+4] >= 0x8000) datos[i+4] = -((65535 - datos[i+4])+1);
      datos[i + 4] = datos[i + 4] / 131 - offset[i];	          
    }
  
  Wire.endTransmission(true);
}

void calibrar_MPU6050(float *offset)
{
  float offsetX = 0,offsetY = 0, offsetZ = 0;
  float offsetaX = 0, offsetaY = 0, offsetaZ = 0;
  float temp;
  for (int i = 0; i < 100; i++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,14,true); // request a total of 14 registers
      offsetaX = Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      offsetaY = Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      offsetaZ = Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      temp =	Wire.read()<<8|Wire.read();
      offsetX = Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      offsetY = Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      offsetZ = Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      
      if (offsetX >= 0x8000) offsetX = -((65535 - offsetX)+1);
      offsetX = offsetX / 131;
      if (offsetY >= 0x8000) offsetY = -((65535 - offsetY)+1);
      offsetY = offsetY / 131;
      if (offsetZ >= 0x8000) offsetZ = -((65535 - offsetZ)+1);
      offsetZ = offsetZ / 131;
      if (offsetaX >= 0x8000) offsetaX = -((65535 - offsetaX)+1);
      offsetaX = offsetaX / 16384.0;
      if (offsetaY >= 0x8000) offsetaY = -((65535 - offsetaY)+1);
      offsetaY = offsetaY / 16384.0;
      if (offsetaZ >= 0x8000) offsetaZ = -((65535 - offsetaZ)+1);
      offsetaZ = offsetaZ / 16384.0;
      
      offset[0] = offset[0] + offsetX;
      offset[1] = offset[1] + offsetY;
      offset[2] = offset[2] + offsetZ;
      offset[3] = offset[3] + offsetaX;
      offset[4] = offset[4] + offsetaY;
      offset[5] = offset[5] + offsetaZ;   
    }
  offset[0] = offset[0] / 100;
  offset[1] = offset[1] / 100;
  offset[2] = offset[2] / 100;
  offset[3] = offset[3] / 100;
  offset[4] = offset[4] / 100;
  offset[5] = offset[5] / 100;
}




