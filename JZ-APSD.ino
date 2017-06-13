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
 *      通信地址：0x78 (u8g库)
 *      
 *    u8g库网址：
 *    https://github.com/olikraus/u8glib
 *    
 *    字体网址：
 *    https://github.com/olikraus/u8glib/wiki/fontsize
 *    
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
 *    2017/6/13 14：00 南昌 海平面气压为：1007hpa
 *    
 *    标准海平面气压为：1013.25hpa
 */
 
#include "U8glib.h"   

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1007)

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);

Adafruit_BME280 bme; // I2C

void draw(void) 
{
  u8g.setFont(u8g_font_6x10);        // select font
  u8g.drawStr(1, 12, "JZ-APSD");
  u8g.setFont(u8g_font_fub14);        
  u8g.setPrintPos(35, 40);     
  u8g.println(bme.readAltitude(SEALEVELPRESSURE_HPA), 0);
  u8g.println("m");      
  u8g.drawRFrame(15, 15, 100, 30, 8);

  u8g.setFont(u8g_font_gdb12);
  u8g.setPrintPos(25, 63);
  u8g.println(bme.readPressure() / 100.0F, 0);
  u8g.println("hpa");
  
}

void setup(void) 
{
  bme.begin();
}

void loop(void) 
{
  bme.readTemperature();   //必须读取这些值才能计算出准确的大气压
  bme.readHumidity();
  
  u8g.firstPage();  
  do 
    {
     draw();      
    }
  while( u8g.nextPage() );
  
  delay(200);  // Delay of 0.2sec 
}
