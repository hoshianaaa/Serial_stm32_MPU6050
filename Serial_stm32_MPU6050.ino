#include <Wire.h>
#include <MadgwickAHRS.h>
Madgwick MadgwickFilter;

#define MPU6050_PWR_MGMT_1   0x6B
#define MPU_ADDRESS  0x68

//送信データ形式
//[header, roll(上位), roll(下位), pitch(上位), pitch(下位), yaw(上位), yaw(下位)]
uint8_t txbuf[] = {255, 0, 0, 0, 0, 0, 0};


void setup() {
  Wire.begin();
  Serial.begin(115200); //115200bps

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);  //MPU6050_PWR_MGMT_1レジスタの設定
  Wire.write(0x00);
  Wire.endTransmission();

  MadgwickFilter.begin(200); //100Hz
}

void loop() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  while (Wire.available() < 14);
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, Temperature;

  axRaw = Wire.read() << 8 | Wire.read();
  ayRaw = Wire.read() << 8 | Wire.read();
  azRaw = Wire.read() << 8 | Wire.read();
  Temperature = Wire.read() << 8 | Wire.read();
  gxRaw = Wire.read() << 8 | Wire.read();
  gyRaw = Wire.read() << 8 | Wire.read();
  gzRaw = Wire.read() << 8 | Wire.read();

  // 加速度値を分解能で割って加速度(G)に変換する
  float acc_x = axRaw / 16384.0;  //FS_SEL_0 16,384 LSB / g
  float acc_y = ayRaw / 16384.0;
  float acc_z = azRaw / 16384.0;

  // 角速度値を分解能で割って角速度(degrees per sec)に変換する
  float gyro_x = gxRaw / 131.0;  // (度/s)
  float gyro_y = gyRaw / 131.0;
  float gyro_z = gzRaw / 131.0;


  //c.f. Madgwickフィルターを使わずに、PRY（pitch, roll, yaw）を計算
  //double roll  = atan2(acc_y, acc_z) * RAD_TO_DEG;
  //double pitch = atan(-acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * RAD_TO_DEG;


  //Madgwickフィルターを用いて、PRY（pitch, roll, yaw）を計算
  MadgwickFilter.updateIMU(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);

  //PRYの計算結果を取得する
  short roll  = MadgwickFilter.getRoll();
  short pitch = MadgwickFilter.getPitch();
  short yaw   = MadgwickFilter.getYaw();
  /*
    Serial.print(roll);  Serial.print(",");
    Serial.print(pitch);  Serial.print(",");
    Serial.print(pitch);  Serial.println("");
  */
  Serial.write(highByte(roll));
  Serial.write(lowByte(roll));
  Serial.write(highByte(pitch));
  Serial.write(lowByte(pitch));
  Serial.write(highByte(yaw));
  Serial.write(lowByte(yaw));

}
