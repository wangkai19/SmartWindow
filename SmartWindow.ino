#include <dht11.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#define Guangmin A5
#define People A4
#define Flame A2
#define FengMing 2
/*引脚３被用于舵机*/
#define DHT11PIN 5                          //dht11数据端使用引脚4


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
float Ro = 10;                             //Ro is initialized to 10 kilo ohms    
int fire = 0;                              //火焰
char get_message[128];                     //接收到的字符数组

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
 *                        光敏模块  当阳光充足时开窗　黑暗时关窗
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
                          人体红外模块　当人接近３米内蜂鸣器报警
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
                        定时模块　没做完
*************************************************************/
void setTime()
{
  unsigned long nowtime = millis();           //获取当前系统运行时间
  Serial.println(nowtime);
  
}

/****************************************************************
                        温湿度模块 将温湿度打印到串口上
****************************************************************/
void getDht11()
{
   DHT11.read(DHT11PIN);                  //temp用来记录pin2口读取到的成功情况
   char msg_HUM[50];
   char msg_TEM[50];
   //Serial.println((int)DHT11.humidity);
   snprintf (msg_HUM, 75, "Hum: %d ",(int)DHT11.humidity);                          
   mySerial.println(msg_HUM);
   snprintf (msg_TEM, 75, "Tem: %d ",(int)DHT11.temperature);                          
   mySerial.println(msg_TEM);
   //在串口打印出温湿度
  /*mySerial.print("Humidity (%): ");
  mySerial.println((float)DHT11.humidity, 2);       //dht11类中的变量humdidity（湿度）
  mySerial.print("Temperature (oC): ");
  mySerial.println((float)DHT11.temperature, 2);    //dut11类中的变量temperature（温度）
 */
}

/********************************************************************************
                        一氧化碳模块　当co超标时报警
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
   char msg_LPG[50];
   char msg_CAR[50];
   char msg_MET[50];
   snprintf (msg_LPG, 75, "LPG: %d ppm",MQGetGasPercentage(MQRead(MQ9PIN)/Ro,GAS_LPG));                          
   mySerial.println(msg_LPG);
   snprintf (msg_CAR,75,"CAR: %d ppm",MQGetGasPercentage(MQRead(MQ9PIN)/Ro,GAS_CARBON_MONOXIDE));
   mySerial.println(msg_CAR);
   snprintf (msg_MET,75,"MET: %d ppm",MQGetGasPercentage(MQRead(MQ9PIN)/Ro,GAS_METHANE) );
   mySerial.println(msg_MET);
   /*
   mySerial.print("    ");     
   mySerial.print("CARBON_MONOXIDE:");                 //一氧化碳
   mySerial.print(MQGetGasPercentage(MQRead(MQ9PIN)/Ro,GAS_CARBON_MONOXIDE) );
   mySerial.print( "ppm" );
   mySerial.print("    ");   
   mySerial.print("METHANE:");                         //甲烷
   mySerial.print(MQGetGasPercentage(MQRead(MQ9PIN)/Ro,GAS_METHANE) );
   mySerial.print( "ppm" );
   mySerial.print("\n");*/
   //delay(200);
   int i = MQGetGasPercentage(MQRead(MQ9PIN)/Ro,GAS_CARBON_MONOXIDE);
   if(i > 4 && flag == 0)
   { 
      forward();
      flag = 1; 
   }
}

/***************************************************************
　　　　　　　　            火焰报警 当有火焰时开窗
****************************************************************/
void getFire()
{
  fire = analogRead(Flame);
  //Serial.println(fire);
  if(fire < 600 && flag == 0)              //当阳光充足且窗户关闭时
    {
      forward();                              //开窗
      flag = 1;                               //窗已开
    }
  
}

/****************************************************************
 *                  从窗口上读取数据　然后开关窗户 
 ***************************************************************/
void readMes()
{
  String openW = "open";
  String closeW = "close";
  String temp = "";
  
  int i = 0;
  //int n = 0;
  while(mySerial.available() > 0)
  {
      get_message[i++] =  mySerial.read();       
  }
  get_message[i] = '\0';
  i = 0;
  //Serial.println(get_message);
  
  for(int j = 0;j < 128;j++)
   {
      temp += get_message[j];   
   }
  
  if(temp == openW && flag == 0)
  {
      forward();
      flag = 1;
  }

  else if(temp == closeW && flag == 1)
  {
      backward();
      flag = 0;
  }
   
}


 

/**************************************************************
                          主程序
***************************************************************/
void setup()
{
  
  Serial.begin(9600);  
  mySerial.begin(115200);
  Ro = MQCalibration(MQ9PIN);                  //Calibrating the sensor. Please make sure the sensor is in clean air 
  myservo.attach(3);                          //舵机由引脚3控制  
  pinMode(FengMing,OUTPUT);                   //蜂鸣器设置       
  pinMode(People,INPUT);                      //设置人体感应为输入模式  
  pinMode(Flame,INPUT);                       //将火焰传感器设置为输入模式
  pinMode(8, INPUT);                          //rx
  pinMode(9, OUTPUT);                         //tx
 
}

void loop()
{
  int i = 0;
  char n = 0;
  while(mySerial.available() > 0)
  {
      n = mySerial.read();
      if(n >= 'a' && n <= 'z')
        get_message[i++] = n;       
  }
  get_message[i] = '\0';
  i = 0;
  Serial.println(get_message);
  delay(500);
  mySerial.println("wangkai19");
  sun();  
  findPeople();
  getDht11();
  getFire();
  getCo();
 // readMes();
  
}
