#include <MPU9255.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh;
MPU9255 mpu;

geometry_msgs::Vector3 acc, gyro, magn;
ros::Publisher mpu_acc("imu/accelerometer", &acc);
ros::Publisher mpu_gyro("imu/gyroscope", &gyro);
ros::Publisher mpu_magn("imu/magnetometer", &magn);

void setup() {
  Serial.begin(115200);//initialize Serial port
  if(mpu.init())
  {
  Serial.println("initialization failed");
  }
  else
  {
  Serial.println("initialization succesful!");
  }

  nh.initNode();
  nh.advertise(mpu_acc);
  nh.advertise(mpu_gyro);
  nh.advertise(mpu_magn);
 
}
 
void loop() {
  mpu.read_acc();//get data from the accelerometer
  mpu.read_gyro();//get data from the gyroscope
  mpu.read_mag();//get data from the magnetometer

  acc.x = mpu.ax;
  acc.y = mpu.ay;
  acc.z = mpu.az;

  gyro.x = mpu.gx;
  gyro.y = mpu.gy;
  gyro.z = mpu.gz;

  magn.x = mpu.mx;
  magn.y = mpu.my;
  magn.z = mpu.mz;
 
  //print all data in serial monitor
  Serial.print("AX: ");
  Serial.print(mpu.ax);
  Serial.print(" AY: ");
  Serial.print(mpu.ay);
  Serial.print(" AZ: ");
  Serial.print(mpu.az);
  Serial.print("    GX: ");
  Serial.print(mpu.gx);
  Serial.print(" GY: ");
  Serial.print(mpu.gy);
  Serial.print(" GZ: ");
  Serial.print(mpu.gz);
  Serial.print("    MX: ");
  Serial.print(mpu.mx);
  Serial.print(" MY: ");
  Serial.print(mpu.my);
  Serial.print(" MZ: ");
  Serial.println(mpu.mz);

  mpu_acc.publish(&acc);
  mpu_gyro.publish(&gyro);
  mpu_magn.publish(&magn);
  nh.spinOnce();
  
  delay(100);
}