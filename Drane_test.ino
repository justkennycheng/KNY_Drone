#include <Wire.h>
#include <SPI.h>
#include "RF24.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include "MS5611.h"

RF24 radio(9, 10); // 建立RFT24对象; CE=D9/pin18, CSN=D10/pin21 ||然而ESP要写D的编号。。。
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); /* Assign a unique ID to this sensor at the same time */
MS5611 MS5611(0x77);

float AccPitch, AccRoll, gyroRoll, gyroPitch, gyroYaw, LPF2_Pitch = 0.0, LPF2_Roll = 0.0;
long int elapsedTime =0 , currentTime =0 , previousTime =0 ;
float headingDegrees;
float pressure, height, LP_height; //, height;

struct Mystruct { //structure for 2.4G RF wireless
  uint16_t s1;  
  uint16_t s2;  
  uint16_t s3;  
  uint16_t s4;  
  uint16_t s5;  
  uint16_t s6;   //arduino unsigned int
  bool b1;  
  bool b2;  
  bool b3;  
  bool b4;
};
Mystruct MyData;





void setup() {
  // put your setup code here, to run once:


 
  Radio_init_for_Read(80,0xF0F0F0F001); 

  Serial.begin(38400);
  delay(500);
  Serial.println("serial started");
  
  Wire.begin();
  Wire.setClock(400000L);   // uncomment this to set I2C clock to 400kHz
  //clockFrequency: the value (in Hertz) of the desired communication clock. Accepted values are 100000 (standard mode) and 400000 (fast mode). Some processors also support 10000 (low speed mode), 1000000 (fast mode plus) and 3400000 (high speed mode). Please refer to the specific processor documentation to make sure the desired mode is supported.
  
  delay(100);

  MPU6050_init();

  HMC5883_init();
  
  MS5611_init();
    
  delay(100);
  //Serial.end();




}

void loop() {
  // put your main code here, to run repeatedly:

  unsigned long ftime;
  ftime = micros() ;

  //////////////////////////////////////////////////////////////////////////////////RADIO///////////////////////////////
  if (radio.available()) {
    radio.read(&MyData, sizeof(MyData));
    //Serial.print(MyData.s1);Serial.print(",");
    //Serial.print(MyData.s2);Serial.print(",");
    //Serial.print(MyData.s3);Serial.print(",");
    //Serial.print(MyData.s4);Serial.print(",");
    Serial.print("radio.s5:");Serial.print(MyData.s5);Serial.print(",");
    //Serial.print(MyData.s6);Serial.print(",");
    //Serial.print(MyData.b1);Serial.print(",");
    //Serial.print(MyData.b2);Serial.print(",");
    //Serial.print(MyData.b3);Serial.print(",");
    //Serial.print(MyData.b4);Serial.println("");
        Serial.print("time_radio:");
        Serial.print(micros()-ftime);ftime = micros();
        Serial.print(",");
        //
  }else{      Serial.print("radio not available  ");  }


  
  //考虑到开机后积分过程，需在setup()中进行初始化运算后再进入循环

  //////////////////////////////////////////////////////////////////////////////////MPU6050///////////////////////////////
  get_MPU6050data();    //耗时0.990ms
        Serial.print("time_6050:");
        Serial.print(micros()-ftime);ftime = micros();
        Serial.print(",");
      Serial.print("LPF2_Pitch:");    //AccPitch, AccRoll, gyroRoll, gyroPitch, gyroYaw;
      Serial.print(LPF2_Pitch);
      Serial.print(",");
      Serial.print("LPF2_Roll:");
      Serial.print(LPF2_Roll);
      Serial.print(",");


  //////////////////////////////////////////////////////////////////////////////////HMC5833///////////////////////////////
  get_HMC5883data();  //耗时0.700ms
        Serial.print("time_5833:");
        Serial.print(micros()-ftime);ftime = micros();
        Serial.print(",");
      Serial.print("Heading_degrees:"); 
      Serial.print(headingDegrees);
      Serial.print(","); 

  //////////////////////////////////////////////////////////////////////////////////MS5611///////////////////////////////
  get_MS5611data();   //耗时2.287ms
        Serial.print("time_5611:");
        Serial.print(micros()-ftime);ftime = micros();
        Serial.print(",");
      Serial.print("LP_height:"); 
      Serial.print(LP_height);


  Serial.println("");


  
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
////////////////////FUNCTIONS//////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////


void Radio_init_for_Read(uint8_t radio_channel, uint64_t pipe_address){
  if (!radio.begin()) {   //使用radio.begin()来启动radio  //Begin operation of the chip.
    Serial.println("radio hardware is not responding");
    while (1) {
      Serial.println("radio hardware is not responding. Please reboot");
    } // hold in infinite loop and never goes down
  } 
   
  radio.setChannel(radio_channel);   //Which RF channel to communicate on, 0-127  物理频率通道
  radio.setPALevel(RF24_PA_MIN);  //RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_MED=-6dBM, and RF24_PA_HIGH=0dBm. 
  radio.setDataRate(RF24_250KBPS);  //在MIN下250可用
  radio.openReadingPipe(1, pipe_address);
  radio.startListening();
  delay(100);
  
}


void MPU6050_init(){

  Serial.println("Adafruit MPU6050 test!");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("MPU6050 Found!");

  //setupt motion detection
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); //定义加速度量程（16位的adc 0~65535 ，量程决定精度）参数有MPU6050_RANGE_2_G，MPU6050_RANGE_4_G,MPU6050_RANGE_8_G,MPU6050_RANGE_16_G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);  //设置陀螺仪量程，参数有MPU6050_RANGE_250_DEG（+/- 250 deg/s），MPU6050_RANGE_500_DEG，MPU6050_RANGE_1000_DEG，MPU6050_RANGE_2000_DEG
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); ////设置数字低通滤波器DLPF，滤波频率应为采样频率的一半/ 参数有MPU6050_BAND_260_HZ，MPU6050_BAND_184_HZ，MPU6050_BAND_94_HZ，MPU6050_BAND_44_HZ，MPU6050_BAND_21_HZ，MPU6050_BAND_10_HZ，MPU6050_BAND_5_HZ
  mpu.setI2CBypass(true);
  //mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  //mpu.setMotionDetectionThreshold(1);
  //mpu.setMotionDetectionDuration(20);
  //mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  //mpu.setInterruptPinPolarity(true);
  //mpu.setMotionInterrupt(true);


}


void get_MPU6050data(){
  
    /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //加速度单位为m/s^2.
  //陀螺仪角速度单位为 rad/s.

  AccPitch=atan( a.acceleration.x/sqrt(a.acceleration.y*a.acceleration.y+a.acceleration.z*a.acceleration.z) ) * 360/(2*3.1415926); //俯仰pitch角θ。，俯仰角 pitch（用 θ 表示）是重力加速度与Y-Z 平面”的夹角
  AccRoll=atan( a.acceleration.y/sqrt(a.acceleration.x*a.acceleration.x+a.acceleration.z*a.acceleration.z) ) * 360/(2*3.1415926); //滚转Roll角Ф，不考虑X分量时可直接用 Y/Z 。横滚角 roll（用 Ф 表示）指重力加速度与“X-Z 平面”的夹角

  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) ; // 注意是毫秒
  //角速度乘以时间进行积分，得到角度。

  gyroRoll = gyroRoll + g.gyro.x * 360/(2*3.1415926) * elapsedTime/1000; // deg/s * s = deg //运算前毫秒/1000得到秒
  gyroPitch = gyroPitch - g.gyro.y * 360/(2*3.1415926) * elapsedTime/1000;
  gyroYaw =  gyroYaw + g.gyro.z * 360/(2*3.1415926) * elapsedTime/1000;  

  
  
  //一阶互补滤波
  //LPF_Pitch=LPF_para*AccAngleXPitch+(1-LPF_para)*(LPF_Pitch - GyroY * elapsedTime/1000);
  //LPF_Roll=LPF_para*AccAngleYRoll+(1-LPF_para)*(LPF_Roll + GyroX * elapsedTime/1000);

  //二阶互补滤波
  float LPF2_para = 0.2; //二阶滤波参数
  float Pitch_x1 = 0.0, Pitch_x2= 0.0, Pitch_y1= 0.0;     //二阶滤波临时变量
  float Roll_x1= 0.0, Roll_x2= 0.0, Roll_y1= 0.0;         //二阶滤波临时变量

  Pitch_x1= (AccPitch-LPF2_Pitch)*(1-LPF2_para)*(1-LPF2_para);  
  Pitch_y1= Pitch_y1 + Pitch_x1 * elapsedTime/1000;
  Pitch_x2= Pitch_y1 + 2*(1-LPF2_para)*(AccPitch-LPF2_Pitch) - g.gyro.y * 360/(2*3.1415926) ;
  LPF2_Pitch = LPF2_Pitch + Pitch_x2 * elapsedTime/1000;  
  
  Roll_x1= (AccRoll-LPF2_Roll)*(1-LPF2_para)*(1-LPF2_para);  
  Roll_y1= Roll_y1 + Roll_x1 * elapsedTime/1000;
  Roll_x2= Roll_y1 + 2*(1-LPF2_para)*(AccRoll-LPF2_Roll) + g.gyro.x * 360/(2*3.1415926) ;
  LPF2_Roll = LPF2_Roll + Roll_x2 * elapsedTime/1000;  
  

  /* Print out the values */
  /*
  Serial.print("AccelX:");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(a.acceleration.z);
  Serial.print(", ");
  Serial.print("GyroX:");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.print(g.gyro.z);


  Serial.print("AccPitch:");    //AccPitch, AccRoll, gyroRoll, gyroPitch, gyroYaw;
  Serial.print(AccPitch);
  Serial.print(",");
  Serial.print("AccRoll:");
  Serial.print(AccRoll);
  Serial.print(",");
  Serial.print("gyroPitch:");
  Serial.print(gyroPitch);
  Serial.print(",");
  Serial.print("gyroRoll:");
  Serial.print(gyroRoll);
  Serial.print(",");
  Serial.print("gyroYaw:");
  Serial.print(gyroYaw);


  Serial.print("LPF2_Pitch:");    //AccPitch, AccRoll, gyroRoll, gyroPitch, gyroYaw;
  Serial.print(LPF2_Pitch);
  Serial.print(",");
  Serial.print("LPF2_Roll:");
  Serial.print(LPF2_Roll);

  
  Serial.println("");

    */


}


void HMC5883_init(){
  
  Serial.println("HMC5883 Magnetometer Test"); 
  if(!mag.begin()){       
    Serial.println("Ooops, no HMC5883 detected ...");
    while(1);
  }


}

void get_HMC5883data(){
  

  sensors_event_t event; 
  mag.getEvent(&event);

  //Serial.print("X:"); 
  //Serial.print(event.magnetic.x); 
  //Serial.print(",");
  //Serial.print("Y:"); 
  //Serial.print(event.magnetic.y); 
  //Serial.print(",");
  //Serial.print("Z:"); 
  //Serial.print(event.magnetic.z); 
  //Serial.print(",");

  float heading = atan2(event.magnetic.y, event.magnetic.x);

  //float declinationAngle = 0.04; 不需要精确方向，只要不漂就行。
  //heading += declinationAngle;

  if(heading < 0){   heading += 2*PI;     }
  if(heading > 2*PI){    heading -= 2*PI;     }
  headingDegrees = heading * 180/M_PI;   

  //一阶滤波  因为有-π到+π的断开，不知道如何滤波
  //LPF_heading=0.2*AccAngleXPitch+0.8*(LPF_Pitch - GyroY * elapsedTime/1000);

  /*
  Serial.print("heading1:"); 
  Serial.print(heading); 
  Serial.print(",");

     

  Serial.print("Heading(degrees):"); 
  Serial.print(headingDegrees);
  Serial.print(","); 
  
  Serial.println("");
  */


}


void MS5611_init(){


  if (MS5611.begin() == true)
  {
    Serial.println("MS5611 found.");
    //MS5611.setOversampling(OSR_ULTRA_LOW); //HIGH为4600微秒，分辨率0.018mbar；STANDARD为2300微秒，分辨率0.027mbar； 详见 https://github.com/RobTillaart/MS5611

    //MS5611.read();
    //pressure = MS5611.getPressure();
    //temperature= MS5611.getTemperature();
    //H_lp = H = ((powf((1013.25/pressure),(1/5.257))-1)*(273.15+temperature))/0.0065;

  }else{
    Serial.println("MS5611 not found. halt.");
    while (1);  
  }
  
}


void get_MS5611data(){

  MS5611.read();           // note no error checking => "optimistic".
  pressure = MS5611.getPressure();
  height = ((powf((1013.25/pressure),(1/5.257))-1)*(273.15+MS5611.getTemperature()))/0.0065; //1013.25 hypsometric 公式
  //height =  44300 * ( 1-(powf((pressure/1013.25),(1/5.256))) );   //barometric 公式，不考虑温度补偿

  //一阶滤波 Y(n)=αX(n) + (1-α)Y(n-1) 
  float a = 0.1 ;
  LP_height = a * height  + (1-a)*LP_height; 

  /*
  Serial.print("height:"); 
  Serial.print(height);
  Serial.print(","); 
  Serial.print("LP_height:"); 
  Serial.print(LP_height);

  Serial.println("");
  */
}















