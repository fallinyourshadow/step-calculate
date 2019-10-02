#include <REG52.H>
#include <math.h>    //Keil library
#include <stdio.h>   //Keil library
#include <INTRINS.H>
//重定义关键字
typedef unsigned char  uchar;
typedef unsigned char  uint;

// 定义51单片机端口
#define DataPort P0	        //LCD1602数据端口
sbit    SCL=P1^0;			//IIC时钟引脚定义
sbit    SDA=P1^1;			//IIC数据引脚定义
sbit    LCM_RS=P2^6;		//LCD1602命令端口
sbit    LCM_RW=P2^5;		//LCD1602命令端口
sbit    LCM_EN=P2^7;		//LCD1602命令端口
// 定义MPU6050内部地址
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	SlaveAddress	0xD0	//IIC写入时的地址字节数据，+1为读取
//定义类型及变量
uchar dis[5];						//显示数字的字符数组
int	dis_data;						//变量
//函数声明
void  delay(uint k);		//延时
//LCD相关函数
void  InitLcd();					//初始化lcd1602
void  lcd_printf(uchar *s,int temp_data);               //字符转换
void  WriteDataLCM(uchar dataW);						//LCD数据
void  WriteCommandLCM(uchar CMD,uchar Attribc);			//LCD指令
void  DisplayOneChar(uchar X,uchar Y,uchar DData);		//显示一个字符
void  DisplayListChar(uchar X,uchar Y,uchar *DData,L);	//显示字符串
//MPU6050操作函数
void  InitMPU6050();									    //初始化MPU6050
void  Delay5us();                                           //延时5微秒
void  I2C_Start();                                          //I2C总线起始信号
void  I2C_Stop();                                           //I2C总线终止信号
void  I2C_SendACK(bit ack);                                 //发送应答信号
bit   I2C_RecvACK();                                        //接收应答信号
void  I2C_SendByte(uchar dat);                              //发送数据
uchar I2C_RecvByte();                                       //接收数据
uchar Single_ReadI2C(uchar REG_Address);				    //读取I2C数据
void  Single_WriteI2C(uchar REG_Address,uchar REG_data);	//向I2C写入数据

//延时
void delay(uint k)
{
    uint i,j;
    for(i=0; i<k; i++)
    {
        for(j=0; j<121; j++)
        {
            ;
        }
    }
}

//LCD1602相关子函数
//LCD1602初始化
void InitLcd()
{
    WriteCommandLCM(0x38,1);
    WriteCommandLCM(0x08,1);
    WriteCommandLCM(0x01,1);
    WriteCommandLCM(0x06,1);
    WriteCommandLCM(0x0c,1);
    DisplayOneChar(0,0,' ');
    DisplayOneChar(1,0,'S');
    DisplayOneChar(2,0,'t');
    DisplayOneChar(3,0,'e');
    DisplayOneChar(4,0,'p');
    DisplayOneChar(5,0,':');
}
//LCD1602写允许
void WaitForEnable(void)
{
    DataPort=0xff;
    LCM_RS=0;
    LCM_RW=1;
    _nop_();
    LCM_EN=1;
    _nop_();
    _nop_();
    while(DataPort&0x80);
    LCM_EN=0;
}
//LCD1602写入命令
void WriteCommandLCM(uchar CMD,uchar Attribc)
{
    if(Attribc)WaitForEnable();
    LCM_RS=0;
    LCM_RW=0;
    _nop_();
    DataPort=CMD;
    _nop_();
    LCM_EN=1;
    _nop_();
    _nop_();
    LCM_EN=0;
}
//LCD1602写入数据
void WriteDataLCM(uchar dataW)
{
    WaitForEnable();
    LCM_RS=1;
    LCM_RW=0;
    _nop_();
    DataPort=dataW;
    _nop_();
    LCM_EN=1;
    _nop_();
    _nop_();
    LCM_EN=0;
}
//LCD1602写入一个字符
void DisplayOneChar(uchar X,uchar Y,uchar DData)
{
    Y&=1;
    X&=15;
    if(Y)X|=0x40;
    X|=0x80;
    WriteCommandLCM(X,0);
    WriteDataLCM(DData);
}
//LCD1602显示字符串
void DisplayListChar(uchar X,uchar Y,uchar *DData,L)
{
    uchar ListLength=0;
    Y&=0x1;
    X&=0xF;
    while(L--)
    {
        DisplayOneChar(X,Y,DData[ListLength]);
        ListLength++;
        X++;
    }
}

//延时5微秒(STC90C52RC@12M)
void Delay5us()
{
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
}
//MPU6050相关子函数
//I2C起始信号
void I2C_Start()
{
    SDA = 1;                    //拉高数据线
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SDA = 0;                    //产生下降沿
    Delay5us();                 //延时
    SCL = 0;                    //拉低时钟线
}
//I2C停止信号
void I2C_Stop()
{
    SDA = 0;                    //拉低数据线
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SDA = 1;                    //产生上升沿
    Delay5us();                 //延时
}
//I2C发送应答信号
//入口参数:ack (0:ACK 1:NAK)
void I2C_SendACK(bit ack)
{
    SDA = ack;                  //写应答信号
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SCL = 0;                    //拉低时钟线
    Delay5us();                 //延时
}
//I2C接收应答信号
bit I2C_RecvACK()
{
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    CY = SDA;                   //读应答信号
    SCL = 0;                    //拉低时钟线
    Delay5us();                 //延时
    return CY;
}
//向I2C总线发送一个字节数据
void I2C_SendByte(uchar dat)
{
    uchar i;
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;              //移出数据的最高位
        SDA = CY;               //送数据口
        SCL = 1;                //拉高时钟线
        Delay5us();             //延时
        SCL = 0;                //拉低时钟线
        Delay5us();             //延时
    }
    I2C_RecvACK();
}
//从I2C总线接收一个字节数据
uchar I2C_RecvByte()
{
    uchar i;
    uchar dat = 0;
    SDA = 1;                    //使能内部上拉,准备读取数据,
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        SCL = 1;                //拉高时钟线
        Delay5us();             //延时
        dat |= SDA;             //读数据
        SCL = 0;                //拉低时钟线
        Delay5us();             //延时
    }
    return dat;
}
//向I2C设备写入一个字节数据
void Single_WriteI2C(uchar REG_Address,uchar REG_data)
{
    I2C_Start();                  //起始信号
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号
    I2C_SendByte(REG_Address);    //内部寄存器地址，
    I2C_SendByte(REG_data);       //内部寄存器数据，
    I2C_Stop();                   //发送停止信号
}
//从I2C设备读取一个字节数据
uchar Single_ReadI2C(uchar REG_Address)
{
    uchar REG_data;
    I2C_Start();                   //起始信号
    I2C_SendByte(SlaveAddress);    //发送设备地址+写信号
    I2C_SendByte(REG_Address);     //发送存储单元地址，从0开始
    I2C_Start();                   //起始信号
    I2C_SendByte(SlaveAddress+1);  //发送设备地址+读信号
    REG_data=I2C_RecvByte();       //读出寄存器数据
    I2C_SendACK(1);                //接收应答信号
    I2C_Stop();                    //停止信号
    return REG_data;
}
//初始化MPU6050
void InitMPU6050()
{
    Single_WriteI2C(PWR_MGMT_1, 0x00);	//解除休眠状态
    Single_WriteI2C(SMPLRT_DIV, 0x07);
    Single_WriteI2C(CONFIG, 0x06);
    Single_WriteI2C(GYRO_CONFIG, 0x18);
    Single_WriteI2C(ACCEL_CONFIG, 0x01);
}
//合成数据
int GetData(uchar REG_Address)
{
	int v;
    char H,L;
    H=Single_ReadI2C(REG_Address);
    L=Single_ReadI2C(REG_Address+1);
	v = (H<<8)+L;
    return v/128;  //合成数据
}
//显示数据子函数
//在1602上显示步数
void DisplayStep(int step,uchar x,uchar y)
{
    lcd_printf(dis,step);		//转换数据显示
    DisplayListChar(x,y,dis,5);	//启始列，行，显示数组，显示长度
}
//整数转字符串
void lcd_printf(uchar *s,int step)
{
	*s =step/10000+0x30;//万
    step=step%10000;    //取余运算
	*++s =step/1000+0x30;//千
    step=step%1000;    //取余运算
    *++s =step/100+0x30; //百
    step=step%100;     //取余运算
    *++s =step/10+0x30;//十
    step=step%10;      //取余运算
    *++s =step+0x30;   //个
}
bit TitterJudge(int frist,int second,int tvalue1,int tvalue2)//干扰判断
{
    int dvalue;
    dvalue = frist - second;//做差
    while(dvalue<0)dvalue = -dvalue;//取正
    if(dvalue>=tvalue1||dvalue<=tvalue2)return 0;	//抖动
    else  return 1;
}
long int G_Accel()
{
    int x,y,z;
    x = GetData(ACCEL_XOUT_H);
    y = GetData(ACCEL_YOUT_H);
    z = GetData(ACCEL_ZOUT_H);
    return sqrt(x*x+y*y+z*z);
}
long int Y_Z_Accel()
{
	int y,z;
    y = GetData(ACCEL_YOUT_H);
    z = GetData(ACCEL_ZOUT_H);
    return sqrt(y*y+z*z);
}
bit begin()//峰值检测
{
    float F,L;
	float d=6;//初值，使流判结果为0
	F = G_Accel();//首次取值
	if(F>126||F<118)//峰值产生
	{
		delay(200);//延时
    	L = G_Accel();//二次取值
		d=F-L;//做差
	}
	while(d<0)d=-d;//取正
    if(d<6)return 1;//波动过滤
    else return 0;
}
//主程序
void main()
{	int	x;//水平加速度变量
   	bit g = 0;//竖直
	bit threshold = 0;
    int step = 0;
    delay(500);		//上电延时
    InitLcd();		//液晶初始化
    InitMPU6050();	//初始化MPU6050
    delay(150);
    while(1)
    {
		DisplayStep(step,6,0);//显示步数
		//delay(1000);
		//delay(1000);
		//delay(1000);
		x = Y_Z_Accel();
    	g = begin();
    	while(g)
    	{
			threshold=TitterJudge(x,Y_Z_Accel(),40,12);//抖动判断
			while(threshold)
       	 	{
				g = 0;
				while(!g)
        	 	{
            		g = begin();
        		}//加速度升降延时
				g = 0;
				while(g)
        	 	{
            		g = begin();
        		}//加速度升降延时
      	      	step++;
      	      	break;
        	break;
    	  }
		}
    }
}
