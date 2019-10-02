#include <REG52.H>
#include <math.h>    //Keil library
#include <stdio.h>   //Keil library
#include <INTRINS.H>
//�ض���ؼ���
typedef unsigned char  uchar;
typedef unsigned char  uint;

// ����51��Ƭ���˿�
#define DataPort P0	        //LCD1602���ݶ˿�
sbit    SCL=P1^0;			//IICʱ�����Ŷ���
sbit    SDA=P1^1;			//IIC�������Ŷ���
sbit    LCM_RS=P2^6;		//LCD1602����˿�
sbit    LCM_RW=P2^5;		//LCD1602����˿�
sbit    LCM_EN=P2^7;		//LCD1602����˿�
// ����MPU6050�ڲ���ַ
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define	SlaveAddress	0xD0	//IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ
//�������ͼ�����
uchar dis[5];						//��ʾ���ֵ��ַ�����
int	dis_data;						//����
//��������
void  delay(uint k);		//��ʱ
//LCD��غ���
void  InitLcd();					//��ʼ��lcd1602
void  lcd_printf(uchar *s,int temp_data);               //�ַ�ת��
void  WriteDataLCM(uchar dataW);						//LCD����
void  WriteCommandLCM(uchar CMD,uchar Attribc);			//LCDָ��
void  DisplayOneChar(uchar X,uchar Y,uchar DData);		//��ʾһ���ַ�
void  DisplayListChar(uchar X,uchar Y,uchar *DData,L);	//��ʾ�ַ���
//MPU6050��������
void  InitMPU6050();									    //��ʼ��MPU6050
void  Delay5us();                                           //��ʱ5΢��
void  I2C_Start();                                          //I2C������ʼ�ź�
void  I2C_Stop();                                           //I2C������ֹ�ź�
void  I2C_SendACK(bit ack);                                 //����Ӧ���ź�
bit   I2C_RecvACK();                                        //����Ӧ���ź�
void  I2C_SendByte(uchar dat);                              //��������
uchar I2C_RecvByte();                                       //��������
uchar Single_ReadI2C(uchar REG_Address);				    //��ȡI2C����
void  Single_WriteI2C(uchar REG_Address,uchar REG_data);	//��I2Cд������

//��ʱ
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

//LCD1602����Ӻ���
//LCD1602��ʼ��
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
//LCD1602д����
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
//LCD1602д������
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
//LCD1602д������
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
//LCD1602д��һ���ַ�
void DisplayOneChar(uchar X,uchar Y,uchar DData)
{
    Y&=1;
    X&=15;
    if(Y)X|=0x40;
    X|=0x80;
    WriteCommandLCM(X,0);
    WriteDataLCM(DData);
}
//LCD1602��ʾ�ַ���
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

//��ʱ5΢��(STC90C52RC@12M)
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
//MPU6050����Ӻ���
//I2C��ʼ�ź�
void I2C_Start()
{
    SDA = 1;                    //����������
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    SDA = 0;                    //�����½���
    Delay5us();                 //��ʱ
    SCL = 0;                    //����ʱ����
}
//I2Cֹͣ�ź�
void I2C_Stop()
{
    SDA = 0;                    //����������
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    SDA = 1;                    //����������
    Delay5us();                 //��ʱ
}
//I2C����Ӧ���ź�
//��ڲ���:ack (0:ACK 1:NAK)
void I2C_SendACK(bit ack)
{
    SDA = ack;                  //дӦ���ź�
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    SCL = 0;                    //����ʱ����
    Delay5us();                 //��ʱ
}
//I2C����Ӧ���ź�
bit I2C_RecvACK()
{
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    CY = SDA;                   //��Ӧ���ź�
    SCL = 0;                    //����ʱ����
    Delay5us();                 //��ʱ
    return CY;
}
//��I2C���߷���һ���ֽ�����
void I2C_SendByte(uchar dat)
{
    uchar i;
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;              //�Ƴ����ݵ����λ
        SDA = CY;               //�����ݿ�
        SCL = 1;                //����ʱ����
        Delay5us();             //��ʱ
        SCL = 0;                //����ʱ����
        Delay5us();             //��ʱ
    }
    I2C_RecvACK();
}
//��I2C���߽���һ���ֽ�����
uchar I2C_RecvByte()
{
    uchar i;
    uchar dat = 0;
    SDA = 1;                    //ʹ���ڲ�����,׼����ȡ����,
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        SCL = 1;                //����ʱ����
        Delay5us();             //��ʱ
        dat |= SDA;             //������
        SCL = 0;                //����ʱ����
        Delay5us();             //��ʱ
    }
    return dat;
}
//��I2C�豸д��һ���ֽ�����
void Single_WriteI2C(uchar REG_Address,uchar REG_data)
{
    I2C_Start();                  //��ʼ�ź�
    I2C_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
    I2C_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ��
    I2C_SendByte(REG_data);       //�ڲ��Ĵ������ݣ�
    I2C_Stop();                   //����ֹͣ�ź�
}
//��I2C�豸��ȡһ���ֽ�����
uchar Single_ReadI2C(uchar REG_Address)
{
    uchar REG_data;
    I2C_Start();                   //��ʼ�ź�
    I2C_SendByte(SlaveAddress);    //�����豸��ַ+д�ź�
    I2C_SendByte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ
    I2C_Start();                   //��ʼ�ź�
    I2C_SendByte(SlaveAddress+1);  //�����豸��ַ+���ź�
    REG_data=I2C_RecvByte();       //�����Ĵ�������
    I2C_SendACK(1);                //����Ӧ���ź�
    I2C_Stop();                    //ֹͣ�ź�
    return REG_data;
}
//��ʼ��MPU6050
void InitMPU6050()
{
    Single_WriteI2C(PWR_MGMT_1, 0x00);	//�������״̬
    Single_WriteI2C(SMPLRT_DIV, 0x07);
    Single_WriteI2C(CONFIG, 0x06);
    Single_WriteI2C(GYRO_CONFIG, 0x18);
    Single_WriteI2C(ACCEL_CONFIG, 0x01);
}
//�ϳ�����
int GetData(uchar REG_Address)
{
	int v;
    char H,L;
    H=Single_ReadI2C(REG_Address);
    L=Single_ReadI2C(REG_Address+1);
	v = (H<<8)+L;
    return v/128;  //�ϳ�����
}
//��ʾ�����Ӻ���
//��1602����ʾ����
void DisplayStep(int step,uchar x,uchar y)
{
    lcd_printf(dis,step);		//ת��������ʾ
    DisplayListChar(x,y,dis,5);	//��ʼ�У��У���ʾ���飬��ʾ����
}
//����ת�ַ���
void lcd_printf(uchar *s,int step)
{
	*s =step/10000+0x30;//��
    step=step%10000;    //ȡ������
	*++s =step/1000+0x30;//ǧ
    step=step%1000;    //ȡ������
    *++s =step/100+0x30; //��
    step=step%100;     //ȡ������
    *++s =step/10+0x30;//ʮ
    step=step%10;      //ȡ������
    *++s =step+0x30;   //��
}
bit TitterJudge(int frist,int second,int tvalue1,int tvalue2)//�����ж�
{
    int dvalue;
    dvalue = frist - second;//����
    while(dvalue<0)dvalue = -dvalue;//ȡ��
    if(dvalue>=tvalue1||dvalue<=tvalue2)return 0;	//����
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
bit begin()//��ֵ���
{
    float F,L;
	float d=6;//��ֵ��ʹ���н��Ϊ0
	F = G_Accel();//�״�ȡֵ
	if(F>126||F<118)//��ֵ����
	{
		delay(200);//��ʱ
    	L = G_Accel();//����ȡֵ
		d=F-L;//����
	}
	while(d<0)d=-d;//ȡ��
    if(d<6)return 1;//��������
    else return 0;
}
//������
void main()
{	int	x;//ˮƽ���ٶȱ���
   	bit g = 0;//��ֱ
	bit threshold = 0;
    int step = 0;
    delay(500);		//�ϵ���ʱ
    InitLcd();		//Һ����ʼ��
    InitMPU6050();	//��ʼ��MPU6050
    delay(150);
    while(1)
    {
		DisplayStep(step,6,0);//��ʾ����
		//delay(1000);
		//delay(1000);
		//delay(1000);
		x = Y_Z_Accel();
    	g = begin();
    	while(g)
    	{
			threshold=TitterJudge(x,Y_Z_Accel(),40,12);//�����ж�
			while(threshold)
       	 	{
				g = 0;
				while(!g)
        	 	{
            		g = begin();
        		}//���ٶ�������ʱ
				g = 0;
				while(g)
        	 	{
            		g = begin();
        		}//���ٶ�������ʱ
      	      	step++;
      	      	break;
        	break;
    	  }
		}
    }
}
