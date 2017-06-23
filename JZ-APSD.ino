/*
 * 2017/06/12  MaFeng
 * 大气压力模拟装置
 *    主控板：  Mega2560
 *    
 *    气压检测：BMP280模块
 *      接线方式：I2C  VCC->3.3V   GND->GND   SCL->SCL   SDA->SDA
 *      通信地址：0x76（在库Adafruit_BME280.h中修改）
 *      
 *    显示屏：12864 OLED模块0.96寸 （SSD1306_128X64）
 *      接线方式：I2C  VCC->3.3V   GND->GND   SCL->SCL   SDA->SDA
 *      通信地址：0x3C (Adafruit_SSD1306库，使用find_ad程序查找)
 *      
 *    操控按钮：旋转编码器模块 (RotaryEncoder)
 *      接线方式：CLK->A1  DT->A0  SW->A2  +->5V   GND->GND
 *      库：https://github.com/0xPIT/encoder/tree/arduino
 *      TimerOne库：https://github.com/PaulStoffregen/TimerOne
 *      旋转编码器原理：http://howtomechatronics.com/tutorials/arduino/rotary-encoder-works-use-arduino/
 *      参考：
 *      库：https://github.com/mathertel/RotaryEncoder     
 *      
 *    直流电机驱动： 继电器模块 光耦隔离低触发 12V
 *      接线方式：IN1->D6  IN2->D7
 *    
 *    当前大气压查询网址：中央气象台
 *    http://www.nmc.cn/publish/forecast/AJX/nanchang.html
 *    海平面气压的计算公式是：
 *        P0=Ps×10^[h/18400(1+Tm/273)]
 *    式中
 *        P0为海平面气压，单位hPa；
 *        Ps为本站气压，单位hPa；
 *        h为气压传感器拔海高度，单位为m；
 *        Tm为气柱平均温度，单位为℃，Tm=（t+t12)/2+h/400；
 *        t为干球温度，单位℃；
 *        t12为观测前12小时气温，单位℃。
 *        
 *    本地当天海平面气压查询网址：
 *    http://www.t7online.com/cncnstdf.htm
 *    2017/6/13 14：00 南昌 海平面气压为：1007hpa  办公室海拔20m
 *    
 *    标准海平面气压为：1013.25hpa
 *    
 * 2017.6.18
 * 菜单程序测试成功
 * 2017.6.20
 * 增加继电器控制程序
*/

//----------------------加载库文件--------------------------------------------
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_Sensor.h>  //气压传感器库
#include <Adafruit_BME280.h>  //气压传感器库

#include <ClickEncoder.h>
#include <TimerOne.h>

//----------------------大气压传感器本地海平面设置--------------------------------------
#define SEALEVELPRESSURE_HPA (seaLevelPressure)  //当地当天海平面大气压

//----------------------大气压传感器接线方式定义-------------------------------------------------
Adafruit_BME280 bme; // I2C

//----------------------OLED屏设置------------------------------------
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

//----------------------定义变量------------------------------------
int menuitem = 1;
int frame = 1;
int page = 2;
int lastMenuItem = 1;

String menuItem1 = "Home Screen";
String menuItem2 = "Sea Level Pressure";
String menuItem3 = "Simulation Altitude";
String menuItem4 = "Buff";
String menuItem5 = "Pump: OFF";
String menuItem6 = "Reset";

boolean vacuumPump = false;
int seaLevelPressure = 1013;
int siAltitude = 0;
float siAltitudeHm = 1.0; 
float buff = 10.0;  //默认缓冲高度10m

//String language[3] = { "EN", "ES", "EL" };
//int selectedLanguage = 0;

//String difficulty[2] = { "EASY", "HARD" };
//int selectedDifficulty = 0;

boolean up = false;
boolean down = false;
boolean middle = false;

ClickEncoder *encoder;
int16_t last, value;

int pump1 = 6;    //泵1控制线D6
int pump2 = 7;    //泵2控制线D7


void setup()   
{
  encoder = new ClickEncoder(A1, A0, A2);  //CLK -> A1; DT -> A0; SW -> A2
  encoder->setAccelerationEnabled(false);
  
  Serial.begin(9600);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  display.display();
  display.clearDisplay();

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 
  
  last = encoder->getValue();

//-----------气压传感器启动----------------------------------
  bme.begin();
//------------真空泵初始化-----------------------------------
  pumpInit();
}


//-------------------↓↓↓Loop↓↓↓----------------------------------------------------
void loop() 
{

  turnPumpOn();  //泵启动
  
  siAltitudeHm = siAltitude * 100;   //显示为百米
  
//----------------bme读取温湿度-----------------------------------------------------
  bme.readTemperature();   //必须读取这些值才能计算出准确的大气压
  bme.readHumidity();    //必须读取这些值才能计算出准确的大气压

//------------------显示菜单----------------------------------------------------
  drawMenu();
  
//------------------旋转编码器按钮状态读取-----------------------------
  readRotaryEncoder();

   ClickEncoder::Button b = encoder->getButton();
   if (b != ClickEncoder::Open) 
   {
      switch (b) 
      {
      case ClickEncoder::Clicked:
         middle = true;
        break;
      }
    }    
    
  //----------------------主菜单参数递减----------------------------------------
  if (up && page == 1 ) 
  {     
    up = false;
    if(menuitem == 2 && frame == 2)
    {
      frame--;
    }
    if(menuitem == 3 && frame == 3)
    {
      frame--;
    }
    if(menuitem == 4 && frame == 4)
    {
      frame--;
    }

    
    lastMenuItem = menuitem;
    menuitem--;
    
    if (menuitem==0)
    {
      menuitem=1;
    } 
  }
  
  //----------------------子菜单参数递减-----------------------------------------
  else if (up && page == 2 && menuitem == 2 ) 
  {
    up = false;
    seaLevelPressure--;
  }
  else if (up && page == 2 && menuitem == 3 ) 
  {
    up = false;
    siAltitude--;
  }
  else if (up && page == 2 && menuitem == 4 ) 
  {
    up = false;
    buff--;
    if(buff == -1.00)
    {
      buff = 50;
    }
  }
 // else if (up && page == 2 && menuitem == 5 ) 
 // {
 //   up = false;
 //   selectedDifficulty--;
 //   if(selectedDifficulty == -1)
 //   {
 //     selectedDifficulty = 1;
 //   }
 // }
  
//---------------------------主菜单参数递增---------------------------------------------------
  if (down && page == 1)              //We have turned the Rotary Encoder Clockwise
  {
    down = false;
    if(menuitem == 3 && lastMenuItem == 2)
    {
      frame ++;
    }
    else if(menuitem == 4 && lastMenuItem == 3)
    {
      frame ++;
    }
    else if(menuitem == 5 && lastMenuItem == 4 && frame!=4)
    {
      frame ++;
    }
    
    lastMenuItem = menuitem;
    menuitem++;  
    
    if (menuitem==7) 
    {
      menuitem--;
    }  
  }
  
  //---------------------子菜单参数递增----------------------------------------
  else if (down && page == 2 && menuitem == 2) 
  {
    down = false;
    seaLevelPressure++;
  }
  else if (down && page == 2 && menuitem == 3) 
  {
    down = false;
    siAltitude++;
  }
  else if (down && page == 2 && menuitem == 4) 
  {
    down = false;
    buff++;
    if(buff == 51.00)
    {
      buff = 0;
    }
   }
//  else if (down && page == 2 && menuitem == 5) 
//  {
//    down = false;
//    selectedDifficulty++;
//    if(selectedDifficulty == 2)
//    {
//      selectedDifficulty = 0;
//    }
//   }
  
  //-------------------中间按钮按下---------------------------------------
  if (middle) //Middle Button is Pressed
  {
    middle = false;
   
    if (page == 1 && menuitem==5)       // vacuumPump Control 
    {
      if (vacuumPump) 
      {
        
        menuItem5 = "Pump: ON";
        
        }
      else 
      {
         
        menuItem5 = "Pump: OFF";

       }
    }

    if(page == 1 && menuitem == 6)      // Reset
    {
      resetDefaults();
    }


    else if (page == 1 && menuitem<=4) 
    {
      page=2;
     }
    else if (page == 2) 
     {
      page=1; 
     }
   }   
  }
//--------------------------↑↑↑Loop↑↑↑------------------------------------------
  
  void drawMenu()
  {
    
  if (page == 1) 
  {    
    display.setTextSize(1);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(37, 0);
    display.print("MAIN MENU");
    display.drawFastHLine(0,10,128,WHITE);

    if(menuitem == 1 && frame == 1)
    {   
      displayMenuItem(menuItem1, 15,false);
      displayMenuItem(menuItem2, 25,true);
      displayMenuItem(menuItem3, 35,true);
    }
    else if(menuitem == 2 && frame == 1)
    {
      displayMenuItem(menuItem1, 15,true);
      displayMenuItem(menuItem2, 25,false);
      displayMenuItem(menuItem3, 35,true);
    }
    else if(menuitem == 3 && frame == 1)
    {
      displayMenuItem(menuItem1, 15,true);
      displayMenuItem(menuItem2, 25,true);
      displayMenuItem(menuItem3, 35,false);
    }
     else if(menuitem == 4 && frame == 2)
    {
      displayMenuItem(menuItem2, 15,true);
      displayMenuItem(menuItem3, 25,true);
      displayMenuItem(menuItem4, 35,false);
    }

      else if(menuitem == 3 && frame == 2)
    {
      displayMenuItem(menuItem2, 15,true);
      displayMenuItem(menuItem3, 25,false);
      displayMenuItem(menuItem4, 35,true);
    }
    else if(menuitem == 2 && frame == 2)
    {
      displayMenuItem(menuItem2, 15,false);
      displayMenuItem(menuItem3, 25,true);
      displayMenuItem(menuItem4, 35,true);
    }
    
    else if(menuitem == 5 && frame == 3)
    {
      displayMenuItem(menuItem3, 15,true);
      displayMenuItem(menuItem4, 25,true);
      displayMenuItem(menuItem5, 35,false);
    }

    else if(menuitem == 6 && frame == 4)
    {
      displayMenuItem(menuItem4, 15,true);
      displayMenuItem(menuItem5, 25,true);
      displayMenuItem(menuItem6, 35,false);
    }
    
      else if(menuitem == 5 && frame == 4)
    {
      displayMenuItem(menuItem4, 15,true);
      displayMenuItem(menuItem5, 25,false);
      displayMenuItem(menuItem6, 35,true);
    }
      else if(menuitem == 4 && frame == 4)
    {
      displayMenuItem(menuItem4, 15,false);
      displayMenuItem(menuItem5, 25,true);
      displayMenuItem(menuItem6, 35,true);
    }
    else if(menuitem == 3 && frame == 3)
    {
      displayMenuItem(menuItem3, 15,false);
      displayMenuItem(menuItem4, 25,true);
      displayMenuItem(menuItem5, 35,true);
    }
        else if(menuitem == 2 && frame == 2)
    {
      displayMenuItem(menuItem2, 15,false);
      displayMenuItem(menuItem3, 25,true);
      displayMenuItem(menuItem4, 35,true);
    }
    else if(menuitem == 4 && frame == 3)
    {
      displayMenuItem(menuItem3, 15,true);
      displayMenuItem(menuItem4, 25,false);
      displayMenuItem(menuItem5, 35,true);
    }   
    display.display();
  }
  else if (page==2 && menuitem == 1) 
  {    
   displayHomeScreen();
  }
  else if (page==2 && menuitem == 2) 
  {
   displayIntMenuPage(menuItem2, seaLevelPressure);
  }
   else if (page==2 && menuitem == 3) 
  {
   displayFloatMenuPage(menuItem3, siAltitudeHm);
  }
  else if (page==2 && menuitem == 4) 
  {
   displayFloatMenuPage(menuItem4, buff);
  }
 }

//----------------恢复默认值-------------------------------------------------
  void resetDefaults()
  {
    seaLevelPressure = 1013;
    siAltitude = 0;
    buff = 10.0;
    //selectedDifficulty = 0;

    vacuumPump = false;
    menuItem5 = "Pump: OFF";
    turnPumpOff();
  }

//-----------------泵启动------------------------------------
  void turnPumpOn()
  {
    float altiRead = bme.readAltitude(SEALEVELPRESSURE_HPA);
    if(altiRead < siAltitudeHm)
    {
      if(altiRead < siAltitudeHm - buff)
      {
        pump1Start();
        pump2Start();
        vacuumPump = true;
      }
      else if((altiRead >= siAltitudeHm - buff) && altiRead < siAltitudeHm)
      {
        pump1Stop();
        pump2Start();
        vacuumPump = true;
      }
    }
    else if(altiRead >= siAltitudeHm + 10)
    {
      pump1Stop();
      pump2Stop();
      vacuumPump = false;
    }
  }

//-----------------泵关闭---------------------------------------------
    void turnPumpOff()
  {
    pump1Stop();
    pump2Stop();
  }

//-----------------旋转编码器时间程序---------------------------------------------------------
  void timerIsr() 
  {
  encoder->service();
  }


//---------------------主界面-------------------------------------------
void displayHomeScreen()
{
    display.setTextSize(1);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print("JZ-APSD");
    display.drawFastHLine(0,10,128,WHITE);
    display.setCursor(5, 15);
    display.print("Altitude");
    display.setCursor(5+12, 25);
    display.print("(m)");
    display.setTextSize(2);
    display.setCursor(5, 35);
    display.print(bme.readAltitude(SEALEVELPRESSURE_HPA), 0);
    
    display.setTextSize(1);
    display.setCursor(65, 15);
    display.print("Pressure");
    display.setCursor(65+12, 25);
    display.print("(hpa)");
    display.setTextSize(2);
    display.setCursor(65, 35);
    display.print(bme.readPressure() / 100.0F, 0);

    display.display();
}

//---------------------子菜单显示程序（int型）-------------------------------------------
void displayIntMenuPage(String menuItem, int value)
{
    display.setTextSize(1);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(5, 0);
    display.print(menuItem);
    display.drawFastHLine(0,10,128,WHITE);
    display.setCursor(5, 15);
    display.print("Value");
    display.setTextSize(2);
    display.setCursor(5, 25);
    display.print(value);
    display.setTextSize(2);
    display.display();
}

//---------------------子菜单显示程序（float型）-------------------------------------------
void displayFloatMenuPage(String menuItem, float value)
{
    display.setTextSize(1);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(5, 0);
    display.print(menuItem);
    display.drawFastHLine(0,10,128,WHITE);
    display.setCursor(5, 15);
    display.print("Value");
    display.setTextSize(2);
    display.setCursor(5, 25);
    display.print(value);
    display.setTextSize(2);
    display.display();
}

//---------------------子菜单显示程序（string型）-------------------------------------------
void displayStringMenuPage(String menuItem, String value)
{
    display.setTextSize(1);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(15, 0);
    display.print(menuItem);
    display.drawFastHLine(0,10,128,WHITE);
    display.setCursor(5, 15);
    display.print("Value");
    display.setTextSize(2);
    display.setCursor(5, 25);
    display.print(value);
    display.setTextSize(2);
    display.display();
}

//----------------------------主菜单显示程序--------------------------------------------
void displayMenuItem(String item, int position, boolean selected)
{
    if(selected)
    {
      display.setTextColor(WHITE, BLACK);    //字体、填充颜色
    }
    else
    {
      display.setTextColor(BLACK, WHITE);
    }
    display.setCursor(0, position);
    display.print(">"+item);
}

//----------------------旋转编码器读取------------------------------------------
void readRotaryEncoder()
{
  value += encoder->getValue();
  
  if (value/2 > last) 
  {
    last = value/2;
    down = true;
    delay(150);
  }
  else if (value/2 < last) 
  {
    last = value/2;
    up = true;
    delay(150);
  }
}

//-------------------------真空泵程序------------------------------------------------
void pumpInit()  //初始化
{
  pinMode(pump1, OUTPUT);
  pinMode(pump2, OUTPUT);
  digitalWrite(pump1, HIGH);
  digitalWrite(pump2, HIGH);
}

void pump1Start()
{
  digitalWrite(pump1, LOW);
}

void pump2Start()
{
  digitalWrite(pump2, LOW);
}

void pump1Stop()
{
  digitalWrite(pump1, HIGH);
}

void pump2Stop()
{
  digitalWrite(pump2, HIGH);
}
