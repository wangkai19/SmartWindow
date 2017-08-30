#include <dht11.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#define Guangmin A5
#define People A4
#define FengMing 2
/*引脚３被用于舵机*/
#define DHT11PIN 4                          //dht11数据端使用引脚4

/***************************************************************
                       MQ9
***************************************************************/
#define         MQ9PIN                       (3)      //define which analog input channel you are going to use
#define         RL_VALUE_MQ9                 (1)      //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR_MQ9      (9.799)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clea
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the                                                  
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation
#define         GAS_LPG                      (1)
#define         GAS_CARBON_MONOXIDE          (3)
#define         GAS_METHANE                  (2)
#define         accuracy                     (0)   //for linearcurves

/************************************************************
                   变量定义
*************************************************************/
SoftwareSerial mySerial(8, 9);             //RX,TX；开软串口 

Servo myservo;                             //舵机的旋转角度
dht11 DHT11;                               //温湿度对象

int flag = 0;                              //用来标志窗户的开关　０为关闭状态　１为打开状态
float Ro = 10;                            //Ro is initialized to 10 kilo ohms    


/**************************************************************
                  使电机正转180度　打开窗户
***************************************************************/
//使电机正转180度　打开窗户
void forward()
{
  for (int pos = 0; pos <= 180; pos += 1) { 
   
    myservo.write(pos);              
    delay(15);                       
  } 
}

//使电机反转180度　关闭窗户
void backward()
{
   for (int pos = 180; pos >= 0; pos -= 1) { 
    myservo.write(pos);             
    delay(15);                      
  }
}


/*********************************************************
 *                        光敏模块
 ********************************************************/
void sun()
{
  int liangdu = 0;                            //光敏的亮度值
  
  liangdu = analogRead(Guangmin);               
  //Serial.println(liangdu);
  delay(100);
 
  
  if(liangdu > 950 && flag == 0)              //当阳光充足且窗户关闭时
    {
      forward();                              //开窗
      flag = 1;                               //窗已开
    }

    if(liangdu < 15 && flag == 1)            //当窗户开得时候且阳光不好
   {
      backward();                             //关窗
      flag = 0;                               //窗已关
   }
  
}

/*********************************************************
                          蜂鸣器报警
***********************************************************/
void baojing()
{
  digitalWrite(FengMing,HIGH);
  delay(100);
  digitalWrite(FengMing,LOW);  
}

/************************************************************
                          人体红外模块
************************************************************/
void findPeople()
{
    int val = 0;
    val = analogRead(People);
    //Serial.println(val);
    delay(300);

    if(val > 960){
         baojing();  
         delay(1000);  
      }
     
}

/*************************************************************
                        定时模块
*************************************************************/
void setTime()
{
  unsigned long nowtime = millis();           //获取当前系统运行时间
  Serial.println(nowtime);
  
}

/****************************************************************
                        温湿度模块
****************************************************************/
void getDht11()
{
   int temp = DHT11.read(DHT11PIN);                  //temp用来记录pin2口读取到的成功情况

   //在串口打印出温湿度
  mySerial.print("Humidity (%): ");
  mySerial.println((float)DHT11.humidity, 2);       //dht11类中的变量humdidity（湿度）
  mySerial.print("Temperature (oC): ");
  mySerial.println((float)DHT11.temperature, 2);    //dut11类中的变量temperature（温度）
 
}

/********************************************************************************
                        一氧化碳模块
**********************************************************************************/
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE_MQ9*(1023-raw_adc)/raw_adc));
}

float MQCalibration(int mq_pin)
{
  int i;
  float RS_AIR_val=0,r0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {                     //take multiple samples
    RS_AIR_val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  RS_AIR_val = RS_AIR_val/CALIBARAION_SAMPLE_TIMES;              //calculate the average value

  r0 = RS_AIR_val/RO_CLEAN_AIR_FACTOR_MQ9;                      //RS_AIR_val divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                                 //according to the chart in the datasheet 

  return r0; 
}

//读取ＭＱ9传感器
float MQRead(int mq_pin)                                          
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs;  
}

//用来返回数值结果最终的结果
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)                       
{ 
  if ( accuracy == 0 ) {
  if ( gas_id == GAS_LPG ) {
    return (pow(10,((-2.132*(log10(rs_ro_ratio))) + 2.992)));
  } else if ( gas_id == GAS_CARBON_MONOXIDE ) {
    return (pow(10,((-2.199*(log10(rs_ro_ratio))) + 2.766 )));
  } else if ( gas_id == GAS_METHANE ) {
    return (pow(10,((-2.636*(log10(rs_ro_ratio))) + 3.646)));
  }   
} 

  else if ( accuracy == 1 ) {
    if ( gas_id == GAS_LPG ) {
    return (pow(10,((-2.132*(log10(rs_ro_ratio))) + 2.992)));
  } else if ( gas_id == GAS_CARBON_MONOXIDE ) {
    return (pow(10,((-2.199*(log10(rs_ro_ratio))) + 2.766 )));
  } else if ( gas_id == GAS_METHANE ) {
    return (pow(10,(-0.670*pow((log10(rs_ro_ratio)), 2) - 2.399*(log10(rs_ro_ratio)) + 3.650)));
  } 
}    
  return 0;
}

//打印出co等相关信息
void getCo()
{
     Serial.print("LPG:");                              //丙烷和丁烷
   Serial.print(MQGetGasPercentage(MQRead(MQ9PIN)/Ro,GAS_LPG) );          //单位为ppm 1L水中含有１毫克溶质
   Serial.print( "ppm" );                             
   Serial.print("    ");     
   Serial.print("CARBON_MONOXIDE:");                 //一氧化碳
   Serial.print(MQGetGasPercentage(MQRead(MQ9PIN)/Ro,GAS_CARBON_MONOXIDE) );
   Serial.print( "ppm" );
   Serial.print("    ");   
   Serial.print("METHANE:");                         //甲烷
   Serial.print(MQGetGasPercentage(MQRead(MQ9PIN)/Ro,GAS_METHANE) );
   Serial.print( "ppm" );
   Serial.print("\n");
   delay(200);   
}



/**************************************************************
                          主程序
***************************************************************/
void setup()
{
  
  Serial.begin(9600);  
  Ro = MQCalibration(MQ9PIN);                  //Calibrating the sensor. Please make sure the sensor is in clean air 
  myservo.attach(3);                          //舵机由引脚3控制  
  pinMode(FengMing,OUTPUT);                   //蜂鸣器设置       
  pinMode(People,INPUT);                      //设置人体感应为输入模式  
  pinMode(8, INPUT);                          //rx
  pinMode(9, OUTPUT);                         //tx
 
}

void loop()
{
  sun();  
  findPeople();
  getDht11();
  getCo();
  
}

