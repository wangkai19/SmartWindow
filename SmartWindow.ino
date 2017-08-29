#include <dht11.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#define Guangmin A5
#define People A4
#define FengMing 2
/*引脚３被用于舵机*/
#define DHT11PIN 4                          //dht11数据端使用引脚4

SoftwareSerial mySerial(8, 9);//RX,TX；开软串口 

Servo myservo;                             //舵机的旋转角度
dht11 DHT11;                               //温湿度对象

int flag = 0;                              //用来标志窗户的开关　０为关闭状态　１为打开状态                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  

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

//光敏模块
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

//蜂鸣器报警
void baojing()
{
  digitalWrite(FengMing,HIGH);
  delay(100);
  digitalWrite(FengMing,LOW);  
}

//人体红外模块
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


//定时模块
void setTime()
{
  unsigned long nowtime = millis();           //获取当前系统运行时间
  Serial.println(nowtime);
  
}

//温湿度模块
void getDht11()
{
   int temp = DHT11.read(DHT11PIN);                  //temp用来记录pin2口读取到的成功情况

   //在串口打印出温湿度
  mySerial.print("Humidity (%): ");
  mySerial.println((float)DHT11.humidity, 2);       //dht11类中的变量humdidity（湿度）
  mySerial.print("Temperature (oC): ");
  mySerial.println((float)DHT11.temperature, 2);    //dut11类中的变量temperature（温度）
 
}

void setup()
{
  
  Serial.begin(9600);  
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
}

