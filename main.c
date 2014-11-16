/*
 * main.c
 *
 *  Created on: 2014-7-29
 *      Author: Yuqin Zhu
 */
#include"DSP281x_Device.h"
#include"DSP281x_Examples.h"
//#include"string.h"
//#include"IQmathLib.h"	//全局为Q24
//#include"pid_grando.h"
//#include"pid.h"

#define FlyType 0		//飞行模式选择0-----+型飞行
						//飞行模式选择1-----x型飞行

//MemoryCopy变量声明
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

/****************/
//把这些函数搬移至RAM中运行
#pragma	CODE_SECTION(renew_PWM_isr,"ramfuncs");
#pragma CODE_SECTION(scia_receive_isr,"ramfuncs");
#pragma CODE_SECTION(scia_send_isr,"ramfuncs");
#pragma CODE_SECTION(scib_receive_isr,"ramfuncs");
#pragma CODE_SECTION(scib_send_isr,"ramfuncs");
//#pragma CODE_SECTION(measure_height_isr,"ramfuncs");
#pragma CODE_SECTION(PID,"ramfuncs");
#pragma CODE_SECTION(DataTrans,"ramfuncs");
#pragma CODE_SECTION(adc_isr,"ramfuncs");
/******************/

//中断函数原型声明
interrupt void renew_PWM_isr();
interrupt void scia_receive_isr();
interrupt void scia_send_isr();
interrupt void scib_receive_isr();
interrupt void scib_send_isr();
//interrupt void measure_height_isr();
interrupt void cpu_timer0_isr(void);
//interrupt void adc_isr(void);


void DataTrans();
void PID(signed char X_Ref, signed char Y_Ref, signed char Z_Ref, unsigned int H_Ref);
void HPID(float Height,float Heightset);
void TakeoffSoilgate(Uint16 a);

// 全局变量声明
Uint16 bufferA[11];
Uint32 bufferB[11];						//用于存放接收到的姿态数据
char sign=0 ;							//用于标识是否正确接收到姿态数据
float height=0;							//用于存放当前高度值
Uint16 c_isrPwm=0;						//记录renew_PWM_isr中断次数,作为计算超声波测距的时间用
char   c_isrCap=0;						//用于measure_height_isr中断计数
Uint16  when=0;							//存放当前T3CNT的值
Uint16  when_1=0;						//存放当前c_isrPWM的值
Uint16 last=0,last_1=0;					//用于存放上一次的T3CNT和上一次的c_isrPwm值
char ScibSendFlag=0;					//Scib发送标志位
char j=0,k=0;
char trig=0;							//用renew_PWM中断次数来定时100ms(=trig*40)

int32 Ux , Uy , Uz;						//存放x，y，z控制量
float AngleSet[3]={0,0,0};				//x,y,z上的角度设定值
float w[3];								//用于存放当前角速度和角度

float angle[3]={0,0,0};

char sign_PID=0;						//1----指示数据解析完成准备进入PID计算
char n=0;								//用于Evb发送数据
int  SciaRcou=0;
char sign_reload=0;

//float control[];					//存放控制量的数组
int16 num_control=0;

Uint16 soilgate=0x500E;					//////////////////////////////////////////////油门

float angleX,angleY,angleZ;
unsigned int Height;
float cX=0,cY=0,cZ=0,cH=0;
float KPx=0.002,KIx=0,KDx=0.00065;//0.000005;   			//PID参数	p=0.000375	 0.004055	0.00047	   0.000010;   i=0.000003;
															//KP=0.001,KI=0,KD=0.0004;神数据
float Yi;													//KP=0.001,KI=0,KD=0.0006;
float a=0,b=0;	//y,x轴常值误差补偿
float KPy=0.0020,KIy=0,KDy=0.00067;					//i=0.00005		0.0001		//D

float KPz=0.002,KIz=0,KDz=0.0008;
float KDh=0,KPh=0;
char FlyFlag=0;

//ADC
Uint16 LoopCount=0;
Uint16 ConversionCount=0;
Uint16 Voltage1[10]={0, 0 ,0 ,0, 0 ,0 ,0 ,0, 0 ,0  };
Uint16 Voltage2[10]={0, 0 ,0 ,0, 0 ,0 ,0 ,0, 0 ,0 };
float HeightRed=0,average=0;



void main()
{
	int i;
	InitSysCtrl();
	InitGpio();
	DINT;					//禁止全局中断
	InitPieCtrl();			//将PIE控制寄存器初始化为默认状态
	IER=0x0000;				//禁止CPU中断（IER：CPU级中断使能寄存器）
	IFR=0x0000;				//清除CPU中断标志位（IFR：CPU级中断标志寄存器）
	InitPieVectTable(); 	//初始化中断向量表
	EALLOW;					//确认中断函数入口地址

	/***********车哥的是把MemoryCopy()放在这里************/

	PieVectTable.T1PINT = &renew_PWM_isr;
	PieVectTable.RXAINT = &scia_receive_isr;
	PieVectTable.TXAINT = &scia_send_isr;
	PieVectTable.RXBINT = &scib_receive_isr;
	PieVectTable.TXBINT = &scib_send_isr;
	//PieVectTable.CAPINT5= &measure_height_isr;
	PieVectTable.TINT0  = &cpu_timer0_isr;
//	PieVectTable.ADCINT = &adc_isr;

	EDIS;
	InitPeripherals();
	ConfigCpuTimer(&CpuTimer0, 150, 1000000);
	StartCpuTimer0();

	// Configure ADC
//	AdcRegs.ADCMAXCONV.all = 0x0001;       // Setup 2 conv's on SEQ1
//	AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x3; // Setup ADCINA3 as 1st SEQ1 conv.
//	AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x2; // Setup ADCINA2 as 2nd SEQ1 conv.
//	AdcRegs.ADCTRL2.bit.EVA_SOC_SEQ1 = 1;  // Enable EVASOC to start SEQ1
//	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  // Enable SEQ1 interrupt (every EOS)

//	EvaRegs.GPTCONA.bit.T1TOADC = 1;       // Enable EVASOC in EVA
	for(i=0;i<11;i++)  	//初始化数据变量
	{
	    bufferA[i] = 0;
	    bufferB[i] = 0;
    }

	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitFlash();

	PieCtrlRegs.PIEIER2.bit.INTx4=1;	//使能INT2 INTx4 T1PINT
	PieCtrlRegs.PIEIER9.bit.INTx1=1;  	//使能PIE模块中的SCIA接收中断
	PieCtrlRegs.PIEIER9.bit.INTx2=1;  	//使能PIE模块中的SCIA发送中断
	PieCtrlRegs.PIEIER9.bit.INTx3=1;	//使能PIE模块中的SCIB接收中断
	PieCtrlRegs.PIEIER9.bit.INTx4=1;	//使能PIE模块中的SCIB发送中断
//	PieCtrlRegs.PIEIER5.bit.INTx6=1;	//使能PIE模块中的CAP5捕获中断
	PieCtrlRegs.PIEIER1.bit.INTx7=1;	//使能PIE模块中的CPU定时器中断
	PieCtrlRegs.PIEIER1.bit.INTx6=1;	//使能PIE模块中的ADC中断
	IER|=M_INT9 | M_INT2  | M_INT1;		//开CPU中断

	EINT;  								//开全局中断
	ERTM;  								//开全局实时中断


	/*初始化pid_z模块	全局Q24
	pid_z.param.Kp  =_IQ(0.5);
	pid_z.param.Ki  =_IQ(0.0);
	pid_z.param.Kd  =_IQ(0);
	pid_z.param.Kr  =_IQ(1.0);
	pid_z.param.Km  =_IQ(1.0);
	pid_z.param.Umax=_IQ(1.0);
	pid_x.param.Umin=_IQ(1.0);

	//初始化pid_y模块	全局Q24
	pid_y.param.Kp  =_IQ(0.01);
	pid_y.param.Ki  =_IQ(0);
	pid_y.param.Kd  =_IQ(0);
	pid_y.param.Kr  =_IQ(1.0);
	pid_y.param.Km  =_IQ(1.0);
	pid_y.param.Umax=_IQ(10.0);
	pid_y.param.Umin=_IQ(10.0);

	//初始化pid_x模块	全局Q24
	pid_x.param.Kp  =_IQ(0.01);
	pid_x.param.Ki  =_IQ(0);
	pid_x.param.Kd  =_IQ(0);
	pid_x.param.Kr  =_IQ(1.0);
	pid_x.param.Km  =_IQ(1.0);
	pid_x.param.Umax=_IQ(10.0);
	pid_x.param.Umin=_IQ(10.0);
	*/
	/**************看门狗******************
	EALLOW;
	SysCtrlRegs.SCSR=1;
	EDIS;
	KickDog();
	EALLOW;
	SysCtrlRegs.WDCR =0x2F;				//使能看门狗
	EDIS;
	************************************/

	//DELAY_US(2000000);					//安全起见延时10s，现在延时2s做实验

	DELAY_US(5000000);				//delay 3s


//	EvaRegs.T1CMPR=0x5B8D;				//50%
//	EvaRegs.T2CMPR=0x5B8D;
//	EvbRegs.T3CMPR=0x5B8D;
//	EvbRegs.T4CMPR=0x5B8D;

//	EvaRegs.T1CMPR=soilgate;			//60%
//	EvaRegs.T2CMPR=soilgate;
//	EvbRegs.T3CMPR=soilgate;
//	EvbRegs.T4CMPR=soilgate;
//	DELAY_US(2000000);
/*
	EvaRegs.T1CMPR=0x7704;				//65%
	EvaRegs.T2CMPR=0x7704;
	EvbRegs.T3CMPR=0x7704;
	EvbRegs.T4CMPR=0x7704;
*/
	/**********************油门设置*************************
	EvaRegs.T1CMPR=0x927C;
	EvaRegs.T2CMPR=0x927C;
	EvbRegs.T3CMPR=0x927C;
	EvbRegs.T4CMPR=0x927C;
	DELAY_US(3000000);					//delay 2s
	EvaRegs.T1CMPR=0x493E;
	EvaRegs.T2CMPR=0x493E;
	EvbRegs.T3CMPR=0x493E;
	EvbRegs.T4CMPR=0x493E;
	****************************************************/



	//进入循环，等待中断
	for(;;)
	{
		DELAY_US(20000000);
		TakeoffSoilgate(23437);		//50%
		DELAY_US(2000000);
		TakeoffSoilgate(26906);		//起飞57.3%
		DELAY_US(10000000);
		TakeoffSoilgate(26250);		//软着陆
		DELAY_US(1000000);
		TakeoffSoilgate(25312);
		DELAY_US(1000000);
		TakeoffSoilgate(0x500E);
		DELAY_US(15000000);
		TakeoffSoilgate(26906);		//起飞
		b=-3.5;
		DELAY_US(6000000);
		TakeoffSoilgate(26250);		//软着陆
		DELAY_US(1000000);
		TakeoffSoilgate(25312);
		DELAY_US(1000000);
		TakeoffSoilgate(0x500E);
		DELAY_US(15000000);
//		TakeoffSoilgate(26906);		//起飞
		for(;;)
		{

		}
		/*
		switch(FlyFlag)
		{
			case 1:						//起飞

				 TakeoffSoilgate(809);
				 DELAY_US(2000000);
				 TakeoffSoilgate(23437);
				 break;
			case 2:						//开启定高PID，飞行20s
				 ;
				 break;
			case 3:						//降落
				 break;
			case 4:						//起飞
				 break;
			case 5:						//往回飞,改大点飞行角度
				 break;
			case 6:						//降落
				 break;
			case 7:						//起飞
				 break;
			case 8:						//飞行，此过程中有风扇
				 break;
			case 9:						//降落
				 break;
		}
		*/

	}//end for loop
}//end main

interrupt void renew_PWM_isr()
{
	//FlashRegs.FPWR.bit.PWR = FLASH_SLEEP;				//将FLASH置为睡眠状态


	c_isrPwm++;
	if(c_isrPwm==60000)
	{
		c_isrPwm=0;
	}

	trig++;				//定时2.5*40=100ms
	if(trig==40)
	{
		trig=0;
		GpioDataRegs.GPBDAT.bit.GPIOB8=1;
		DELAY_US(20);
		GpioDataRegs.GPBDAT.bit.GPIOB8=0;
	}

	EvbRegs.T3CNT=EvbRegs.T4CNT=EvaRegs.T1CNT;			//同步定时器

	#if(FlyType==0)										//+型飞行
		//EvaRegs.T1CMPR=EvaRegs.T1CMPR+(Uint16)_IQtoF(_IQmpyI32(Ux,0x1));				//0xB71B
		//EvbRegs.T3CMPR=EvbRegs.T3CMPR-(Uint16)_IQtoF(_IQmpyI32(Ux,0x1));
		//EvaRegs.T2CMPR=EvaRegs.T2CMPR+(Uint16)_IQtoF(_IQmpyI32(Uy,0x1));
		//EvbRegs.T4CMPR=EvbRegs.T4CMPR-(Uint16)_IQtoF(_IQmpyI32(Uy,0x1));
		/******************PID饱和控制**********************
		if(cX>0.2)cX=0.2;
		if(cX<-0.2)cX=-0.2;
		if(cY>0.2)cY=0.2;
		if(cY<-0.2)cY=-0.2;
		*************************************************/
	if(sign_PID==1)
	{
		sign_PID=0;
		PID(-1.6+b,-2.76+a,0,0);
		sign_reload=1;

	}//计算PID end
		if(sign_reload==1)
		{
			sign_reload=0;

			/************融合Z轴和高度PID控制量输出***********/
			if(soilgate+(int16)(0xB71B*(cX-cZ+cH))>0x900C)				//M1
			{
				EvaRegs.T1CMPR=0x900C;
			}
			else if(soilgate+(int16)(0xB71B*(cX-cZ+cH))<0x500E)
			{
				EvaRegs.T1CMPR=0x500E;
			}
			else
			{
				EvaRegs.T1CMPR=soilgate+(int16)(0xB71B*(cX-cZ+cH));
			}

			if(soilgate-(int16)(0xB71B*(cX+cZ-cH))>0x900C)				//M3
			{
				EvbRegs.T3CMPR=0x900C;
			}
			else if(soilgate-(int16)(0xB71B*(cX+cZ-cH))<0x500E)
			{
				EvbRegs.T3CMPR=0x500E;
			}
			else
			{
				EvbRegs.T3CMPR=soilgate-(int16)(0xB71B*(cX+cZ-cH));
			}

			if(soilgate-(int16)(0xB71B*(cY-cZ-cH))>0x900C)				//M2
			{
				EvaRegs.T2CMPR=0x900C;
			}
			else if(soilgate-(int16)(0xB71B*(cY-cZ-cH))<0x500E)
			{
				EvaRegs.T2CMPR=0x500E;
			}
			else
			{
				EvaRegs.T2CMPR=soilgate-(int16)(0xB71B*(cY-cZ-cH));
			}

			if(soilgate+(int16)(0xB71B*(cY+cZ+cH))>0x927C)				//M4
			{
				EvbRegs.T4CMPR=0x900C;
			}
			else if(soilgate+(int16)(0xB71B*(cY+cZ+cH))<0x500E)
			{
				EvbRegs.T4CMPR=0x500E;
			}
			else
			{
				EvbRegs.T4CMPR=soilgate+(int16)(0xB71B*(cY+cZ+cH));
			}
		}
	#endif



	#if(FlyType==1)									//x型飞行

	#endif


	// 注意退出中断函数时需要先释放PIE，使得PIE能够响应同组其他中断
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
	EvaRegs.EVAIFRA.bit.T1PINT=1;	//清除中断标志位
	EINT;
}


interrupt void scia_receive_isr()										//SCIA接收中断函数
{
    int m=0,n=0;
    /****************************11个数据产生中断*********************************/
    for(m=0;m<11;m++)
    {
    	bufferA[m] = SciaRegs.SCIRXBUF.all; 							//接收数据
    	if(bufferA[0]!=0x55)
    	{
    		SciaRegs.SCIFFRX.bit.RXFIFORESET=0;
    		SciaRegs.SCIFFRX.bit.RXFIFORESET=1;
    		SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;
    		PieCtrlRegs.PIEACK.all=0x0100;  							//使得同组其他中断能够得到响应
			EINT;  														//开全局中断
    		return;
    	}

	}
	/////
    ScibSendFlag=1;
    if(bufferA[0]==0x55)
    {
    	sign = 1;
    }
    else
    {
    	SciaRegs.SCIFFRX.bit.RXFIFORESET=0;
		SciaRegs.SCIFFRX.bit.RXFIFORESET=1;
		SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;
		PieCtrlRegs.PIEACK.all=0x0100;  	//使得同组其他中断能够得到响应
		EINT;  								//开全局中断
    	return;
    }
    /****************************************11个数据收到后产生中断end****************************/

    /*if(i==11)
    {
    	sign=1;
    }
	if(strncmp(bufferA,"hellodsp",8)==0)
	{
       SciaRegs.SCIFFTX.bit.TXINTCLR=1; //清除发送中断标志位，使其响应新的中断
	}
	*/
    /***********************收到一个数据产生中断****************************
    bufferA[SciaRcou] = SciaRegs.SCIRXBUF.all;
    SciaRcou++;
    if(bufferA[0]!=0x55)
    {
    	SciaRcou=0;
    	SciaRegs.SCIFFRX.bit.RXFIFORESET=0;
		SciaRegs.SCIFFRX.bit.RXFIFORESET=1;
		SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;
		PieCtrlRegs.PIEACK.all=0x0100;  	//使得同组其他中断能够得到响应
		EINT;  								//开全局中断
		return;
    }
    else
    {
		if(SciaRcou==11)
		{
			SciaRcou=0;
			sign = 1;
		}

    }
    **********************************************************收到一个数据产生中断end*/
//    for(n=0;n<11;n++)
//    {
//    	ScibRegs.SCITXBUF=bufferA[n];
//    }
	ScibRegs.SCIFFTX.bit.TXINTCLR=1; 								//清除发送中断标志位，使其响应新的中断
    DataTrans();
    SciaRegs.SCIFFRX.bit.RXFIFORESET=0;
    SciaRegs.SCIFFRX.bit.RXFIFORESET=1;
	SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;
    PieCtrlRegs.PIEACK.all=0x0100;  	//使得同组其他中断能够得到响应
    EINT;  								//开全局中断

}

interrupt void scia_send_isr()			//SCIA发送中断
{
    int i;

	//for(i=0;i<11;i++)
	//{
	//   SciaRegs.SCITXBUF=bufferA[i]; 	//发送数据
	//}

    PieCtrlRegs.PIEACK.all=0x0100;  	//使得同组其他中断能够得到响应
    EINT;  								//开全局中断
}


interrupt void scib_receive_isr()
{

	int16 NumScib=0;
	for(NumScib=0;NumScib<1;NumScib++)
	{
		bufferB[0] = ScibRegs.SCIRXBUF.all;				//接收数据
	}
	ScibRegs.SCITXBUF=bufferB[0];
	switch(bufferB[0])
	{
	case 0x00:
				EvaRegs.T1CMPR=19687;
				EvaRegs.T2CMPR=19687;
				EvbRegs.T3CMPR=19687;
				EvbRegs.T4CMPR=19687;
				DELAY_US(10000);
				EvaRegs.T1CMPR=0x500E;
				EvaRegs.T2CMPR=0x500E;
				EvbRegs.T3CMPR=0x500E;
				EvbRegs.T4CMPR=0x500E;
				EALLOW;
				GpioMuxRegs.GPAMUX.bit.T1PWM_GPIOA6=0;		//置成GPIO口
				GpioMuxRegs.GPADIR.bit.GPIOA6=1;			//设置为输出
				GpioMuxRegs.GPAMUX.bit.T2PWM_GPIOA7=0;		//置成GPIO口
				GpioMuxRegs.GPADIR.bit.GPIOA7=1;			//设置为输出
				GpioMuxRegs.GPBMUX.bit.T3PWM_GPIOB6=0;		//置成GPIO口
				GpioMuxRegs.GPBDIR.bit.GPIOB6=1;			//设置输出
				GpioMuxRegs.GPBMUX.bit.T4PWM_GPIOB7=0;		//置成GPIO口
				GpioMuxRegs.GPBDIR.bit.GPIOB7=1;			//设置为输出
				EDIS;
				GpioDataRegs.GPADAT.bit.GPIOA6=0;			//置成低电平
				GpioDataRegs.GPADAT.bit.GPIOA7=0;			//置成低电平
				GpioDataRegs.GPBDAT.bit.GPIOB6=0;			//置成低电平
				GpioDataRegs.GPBDAT.bit.GPIOB7=0;			//置成低电平

			break;

			case 0x50:										//4个电机置成50%输出油门
				soilgate=23437;
			break;

			case 0x40:										//4个电机置成60.5%输出油门
				soilgate=28359;
			break;

			case 0x51:	//61.1%
				soilgate=28640;
			break;

			case 0x41:
				soilgate=28828;								//4个电机置成61.5%输出油门
			break;

			case 0x52:	//61.2%
				soilgate=28687;
			break;

			case 0x42:
				soilgate=29296;								//4个电机置成62.5%输出油门
			break;

			case 0x53:	//61.3%
				soilgate=28734;
			break;

			case 0x43:										//4个电机置成63.5%输出油门
				soilgate=29765;
			break;

			case 0x54:	//61.4%
				soilgate=28781;
			break;

			case 0x44:
				soilgate=30234;								//4个电机置成64.5%输出油门
			break;

			case 0x55:	//61.5%									//4个电机置成50%输出油门
				soilgate=28828;
			break;

			case 0x45:	//55%									//4个电机置成45%输出油门
				soilgate=26015;
			break;

			case 0x56:	//61.6%									//4个电机置成50%输出油门
				soilgate=28875;
			break;

			case 0x46:	//56%									//4个电机置成50%输出油门
				soilgate=26484;
			break;

			case 0x57:	//61.7									//4个电机置成50%输出油门
				soilgate=28921;
			break;

			case 0x47:	//57%									//4个电机置成50%输出油门
				soilgate=26953;
			break;

			case 0x58:	//61.8%									//4个电机置成50%输出油门
				soilgate=28968;
			break;

			case 0x48:	//58%									//4个电机置成50%输出油门
					soilgate=27421;
			break;

			case 0x59:	//61.9%								//4个电机置成50%输出油门
					soilgate=29015;
			break;

			case 0x49:	//59%									//4个电机置成50%输出油门
					soilgate=27890;
			break;

			case 0x60:	//60%									//4个电机置成60%输出油门
				soilgate=28125;
			break;

			case 0x61:										//4个电机置成61%输出油门
				soilgate=28593;
			break;

			case 0x62:										//4个电机置成62%输出油门
				soilgate=29062;
					break;
			case 0x63:										//4个电机置成63%输出油门
				soilgate=29531;
					break;
			case 0x64:										//4个电机置成64%输出油门
				soilgate=30000;
					break;
			case 0x65:										//4个电机置成65%输出油门
				soilgate=30469;
			break;

			case 0x66:		//58%								//4个电机置成70%输出油门
				soilgate=27187;
			break;
			case 0x67:		//57.1%
				soilgate=26765;
			break;
			case 0x68:		//57.2%
				soilgate=26812;
			break;
			case 0x69:		//57.3%
				soilgate=26859;
			break;
			case 0x70:		//57.4%
				soilgate=26906;
			break;
			case 0x71:		//57.5%
				soilgate=26953;
			break;
			case 0x72:		//57.6%
				soilgate=27000;
			break;
			case 0x73:		//57.7%
				soilgate=27046;
			break;
			case 0x74:		//57.8%
				soilgate=27093;
			break;
			case 0x75:		//57.9%
				soilgate=27140;
			break;
			case 0x76:		//58.0%
				soilgate=27187;
			break;

			case 0x77:		//58.1%
				soilgate=27234;
			break;
			case 0x78:		//58.2%
				soilgate=27281;
				break;
			case 0x79:		//58.3%
				soilgate=27328;
				break;
			case 0x80:		//58.4%
				soilgate=27375;
				break;
			case 0x81:		//58.5%
				soilgate=27421;
				break;
			case 0x82:		//58.6%
				soilgate=27468;
				break;
			case 0x83:		//58.7%
				soilgate=27515;
				break;
			case 0x84:		//58.8%
				soilgate=27560;
				break;
			case 0x85:		//58.9%
				soilgate=27607;
				break;
			case 0x86:		//59.0%
				soilgate=27653;
				break;
			case 0x87:		//59.1%
				soilgate=27699;
				break;
			case 0x88:		//59.2%
				soilgate=27746;
				break;
			case 0x89:		//59.3%
				soilgate=27792;
				break;
			case 0x90:		//59.4%
				soilgate=27838;
				break;
			case 0x91:		//59.5%
				soilgate=27884;
				break;
			case 0x92:		//59.6%
				soilgate=27937;
				break;
			case 0x93:		//59.7%
				soilgate=27983;
				break;
			case 0x94:		//59.8%
				soilgate=28076;
				break;
			case 0x95:		//59.9%
				soilgate=28122;
				break;
			case 0x96:		//60.0%
				soilgate=28171;
				break;
			case 0x97:		//60.1%
				soilgate=28218;
				break;
			case 0x98:		//60.2%
				soilgate=28264;
				break;
			case 0x99:		//60.3%
				soilgate=28311;
				break;
			case 0xA0:		//60.4%
				soilgate=28311;
				break;
			case 0xA1:		//60.5%
				soilgate=28367;
				break;
			case 0xA2:		//60.6%
				soilgate=28406;
				break;
			case 0xA3:		//60.7%
				soilgate=28453;
				break;
			case 0xA4:		//60.8%
				soilgate=28500;
				break;
			case 0xA5:		//60.9%
				soilgate=28547;
				break;
			case 0xA6:		//61.0%
				soilgate=28593;
				break;
			case 0xA7:		//61.1%
				soilgate=28640;
				break;
			case 0xA8:		//61.2%
				soilgate=28687;
				break;
			case 0xA9:		//61.3%
				soilgate=28734;
				break;
			case 0xB0:		//61.4%
				soilgate=28781;
				break;
			case 0xB1:		//61.5%
				soilgate=28827;
				break;
			case 0xB2:		//61.6%
				soilgate=28875;
				break;
			case 0xB3:		//61.7%
				soilgate=28922;
				break;
			case 0xB4:		//61.8%
				soilgate=28969;
				break;
			case 0xB5:		//61.9%
				soilgate=28916;
				break;
			case 0xB6:		//62.0%
				soilgate=29062;
				break;


			case 0x11:
				EALLOW;
				GpioMuxRegs.GPAMUX.bit.T1PWM_GPIOA6=1;		//置成T1PWM
				GpioMuxRegs.GPAMUX.bit.T2PWM_GPIOA7=1;		//置成T2PWM
				GpioMuxRegs.GPBMUX.bit.T3PWM_GPIOB6=1;		//置成T3PWM
				GpioMuxRegs.GPBMUX.bit.T4PWM_GPIOB7=1;		//置成T4PWM
				EDIS;
				soilgate=0x494E;
			break;
	/***********y轴常值误差角度补偿**************/

			case 0x01:
				a=-2.5;
					break;
			case 0x02:
				a=-2;
					break;
			case 0x03:
				a=-1.5;
					break;
			case 0x04:
				a=-1;
					break;
			case 0x05:
				a=-0.5;
					break;
			case 0x06:
				a=0.5;
					break;
			case 0x07:
				a=1;
					break;
			case 0x08:
				a=1.5;
					break;
			case 0x09:
				a=2;
					break;
			case 0x10:
				a=2.5;
					break;
	/************x轴常值误差角度补偿*************/
			case 0x12:
				b=-2.5;
					break;
			case 0x13:
				b=-2;
					break;
			case 0x14:
				b=-1.5;
					break;
			case 0x15:
				b=-1;
					break;
			case 0x16:
				b=-0.5;
					break;
			case 0x17:
				b=0.5;
					break;
			case 0x18:
				b=1;
					break;
			case 0x19:
				b=1.5;
					break;
			case 0x20:
				b=2;
					break;
			case 0x21:
				KPh=0.2;
					break;
			case 0x22:
				KPh=0.02;
					break;
			case 0x23:
				KPh=0.01;
					break;
			case 0x24:
				KPh=0.006;
				break;
			case 0x25:
				KPh=0.002;
				break;
			case 0x26:
				KPh=0.001;
				break;
			case 0x27:
				KPh=0.0005;
				break;
			case 0x28:
				KPh=0.0001;
				break;
			case 0x31:
				KDh=0.005;
				break;
			case 0x32:
				KDh=0.001;
				break;
			case 0x33:
				KDh=0.0008;
				break;
			case 0x34:
				KDh=0.0006;
				break;
			case 0x35:
				KDh=0.0002;
				break;
			case 0x36:
				KDh=0.0001;
				break;
			case 0x37:
				KDh=0.00005;
				break;
			case 0x38:
				KDh=0.00001;
				break;
	}
	/*k++;
	if(k==7)
	{
		ScibRegs.SCITXBUF=bufferB[k];
		k=0;
		ScibSendFlag=1;
	}
	*/

	ScibRegs.SCIFFRX.bit.RXFIFORESET=0;
	ScibRegs.SCIFFRX.bit.RXFIFORESET=1;
	ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;
	PieCtrlRegs.PIEACK.all=0x0100;  	//使得同组其他中断能够得到响应
	EINT;  								//开全局中断

}
interrupt void scib_send_isr()
{
	//ScibRegs.SCITXBUF=1;
	//if(ScibSendFlag==1)
	//{
		//ScibRegs.SCITXBUF=bufferA[j];
		//ScibRegs.SCITXBUF=bufferB[j];
	//	j++;
	//	if(j==11)
	//	{
	//		j=0;
	//		ScibSendFlag=0;
	//	}
	//}
/*

	int n;
	for(n=0;n<3;n++)
	{
		ScibRegs.SCITXBUF = angle[n];
	}
 */
	PieCtrlRegs.PIEACK.all=0x0100;
	EINT;
}

interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;
   if( CpuTimer0.InterruptCount==30)
   {
	   FlyFlag=1;
	   CpuTimer0.InterruptCount=0;
   }
  // if(FlyFlag==1)
   //{
	   GpioDataRegs.GPBTOGGLE.bit.GPIOB0=1;		//每隔1s闪烁LED
   //}
   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void  adc_isr(void)
{
	static Uint32 sum=0;
	float HeightRed_1=0;
	Voltage1[ConversionCount] = AdcRegs.ADCRESULT0 >>4;
	Voltage2[ConversionCount] = AdcRegs.ADCRESULT1 >>4;

	sum=sum+ Voltage1[ConversionCount];



	// If 40 conversions have been logged, start over
	if(ConversionCount == 9)
	{
		ConversionCount = 0;
		average=sum/10.0;
		average=average*3/4095;
		sum=0;
		HeightRed_1=375.9*average*average-631.9*average+353.3;
	}
	else ConversionCount++;

	if(HeightRed_1>150 || HeightRed_1<100)
	{
		AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
		AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
		PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
		return;
	}
	else
	{
		HeightRed = HeightRed_1;
		HPID(HeightRed, 1.3);
	}
	HPID(HeightRed, 1.3);
	// Reinitialize for next ADC sequence
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

	return;
}


/*
interrupt void measure_height_isr()
{

	if(c_isrCap==0)
	{
		last_1 = c_isrPwm;
	}
	else if(c_isrCap==1)
	{
		when_1 = c_isrPwm;
		c_isrPwm;
	}

//	EvbRegs.CAPFIFOB.bit.CAP5FIFO;
	if(EvbRegs.CAPFIFOB.bit.CAP5FIFO== 0x01)
	{
		last_1 = c_isrPwm;
	}
	if(EvbRegs.CAPFIFOB.bit.CAP5FIFO== 2)
	{
		when_1 = c_isrPwm;
		last	= EvbRegs.CAP5FIFO;
		when 	= EvbRegs.CAP5FIFO;


//		when_1  = c_isrPwm;

//		if(c_isrCap==2)
//		{
		if(when_1<last_1)
		{
			height=((float)(46875+when-last)*2.5/46875+2.5*(when_1-last_1+60000))*0.17;
		}
		else
		{
			//height=((float)(46875+when-last)*2.5/46875+2.5*(when_1-last_1))*0.17;
			height=(float)(2.5*(when_1-last_1))*0.17;
		}
//		}

//		last_1	=when_1;
//		last	=when;
	}
	c_isrCap++;
	if(c_isrCap==2)
	{
		c_isrCap=0;
		//c_isrPwm=0;
	}
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
	EvbRegs.EVBIFRC.bit.CAP5INT = 1;
	EINT;
}
*/

void DataTrans()
{
	char flag_W_a;
	if(sign==1)
	{
		sign=0;
		switch(bufferA[1])
		{
		//	case 0x51:
		//	a[0] = (short(bufferA [3]<<8| bufferA [2]))/32768.0*16;			//x轴上的
		//	a[1] = (short(bufferA [5]<<8| bufferA [4]))/32768.0*16;			//y轴上的
		//	a[2] = (short(bufferA [7]<<8| bufferA [6]))/32768.0*16;			//z轴上的
		//	T = (short(bufferA [9]<<8| bufferA [8]))/340.0+36.25;
		//	break;

			case 0x52:
			w[0] = ((short)(bufferA [3]<<8| bufferA [2]))/32768.00*2000.0;		//x轴上的
			w[1] = ((short)(bufferA [5]<<8| bufferA [4]))/32768.00*2000.0;		//y轴上的
			w[2] = ((short)(bufferA [7]<<8| bufferA [6]))/32768.0*2000;			//z轴上的
			//T = ((short)(bufferA [9]<<8| bufferA [8]))/340.0+36.25;
			flag_W_a=1;
			break;
			case 0x53:
			if(flag_W_a==1)
			{
				flag_W_a=0;
				angle[0] = ((short)(bufferA [3]<<8| bufferA [2]))/32768.00*180.0;	//x轴上的
				angle[1] = ((short)(bufferA [5]<<8| bufferA [4]))/32768.00*180.0;	//y轴上的
				angle[2] = ((short)(bufferA [7]<<8| bufferA [6]))/32768.0*180;	//z轴上的
				//T = ((short)(bufferA [9]<<8| bufferA [8]))/340.0+36.25;

				if(angle[0]>90||angle[0]<-90)return;
				/*****将PID计算放入中******/
				//angleX=angle[0];
				//angleY=angle[1];
				//angleZ=angle[2];
				sign_PID = 1;		//设置标志位，准备进入PID计算
				for(n=2;n<3;n++)
				{
					ScibRegs.SCITXBUF =angle[n];
				}
			}
			break;
		}//end switch

	}//end if
}


void PID(signed char X_Ref, signed char Y_Ref, signed char Z_Ref, unsigned int H_Ref)
{
	float ws[3],as[3];

		float eX=0,eY=0,eZ=0,eH=0;  		//X轴误差、Y轴误差、Z轴误差、高度误差
		static float eX_i=0,eY_i=0,eZ_i=0;	//累计误差

	//	static float eX_1,eY_1,eZ_1,eH_1;   //X、Y、Z轴，以及高度 H 的前 1 次误差
	//	static float eX_2,eY_2,eZ_2,eH_2;   //X、Y、Z轴，以及高度 H 的前 2 次误差

		float pX=0,pY=0,pZ=0,pH=0;         //当前PID计算值
	//	static float pX_1=0,pY_1=0,pZ_1=0,pH_1=0;   //上一次PID计算值


		ws[0]=w[0];		ws[1]=w[1];		ws[2]=w[2];
		as[0]=angle[0]; as[1]=angle[1]; as[2]=angle[2];
		//误差值计算
		/*顺序反了，负反馈*
		eX=angleX-X_Ref;
		eY=angleY-Y_Ref;
		eZ=angleZ-Z_Ref;
		eH=Height-H_Ref;
		*/

		/*误差计算
		eX=X_Ref-angleX;
		eY=Y_Ref-angleY;
		eZ=Z_Ref-angleZ;
		eH=H_Ref-Height;
		*/
		eX=X_Ref-as[0];
		eY=Y_Ref-as[1];
		eZ=Z_Ref-as[2];

		eX_i=eX_i+eX;
		eY_i=eY_i+eY;
		if(eY_i>7)eY_i=7;
		else if(eY_i<-7)eY_i=-7;

	//	PID计算u(t) = Kp*[e(t)-e(t-1)] + Ki*e(t) + Kd*[e(t)-2*e(t-1)+e(t-2)];
	//	pX=KP*(eX-eX_1)+KI*eX+KD*(eX-2*eX_1+eX_2);
	//	pY=KP*(eY-eY_1)+KI*eY+KD*(eY-2*eY_1+eY_2);
	//	pZ=KP*(eZ-eX_1)+KI*eZ+KD*(eZ-2*eZ_1+eZ_2);
	//	pH=KP*(eH-eH_1)+KI*eH+KD*(eH-2*eH_1+eH_2);

		/******************用角速度代替角度的D 位置式*****************/
		pX=KPx*eX+KIx*eX_i-KDx*ws[0];
		pY=KPy*eY+KIy*eY_i-KDy*ws[1];
		pZ=KPz*eZ+KIz*eZ_i-KDz*ws[2];
		Yi=eY_i;

		//pX=KP*eX+KI*eX-KD*w[0];
		//pY=KP*eY+KI*eY-KD*w[1];

		//PID控制量输出，直接用于寄存器的值
	//	cX=(pX_1+pX);
	//	cY=(pY_1+pY);
	//	cZ=(pZ_1+pZ);
	//	cH=(pH_1+pH);
		cX=pX;
		cY=pY;
		cZ=pZ;

		//存储变量值
	//	eX_2=eX_1;
	//	eY_2=eY_1;
	//	eZ_2=eZ_1;
	//	eH_2=eH_1;

	//	eX_1=eX;
	//	eY_1=eY;
	//	eZ_1=eZ;
	//	eH_1=eH;
		/*
		pX_1=pX;
		pY_1=pY;
		pZ_1=pZ;
		pH_1=pH;
		*/
}

void HPID(float Height,float Heightset)
{
	float error;
	static float error_1;
	error=Heightset-Height;
	cH=KPh*error+KDh*(error-error_1);
	error_1=error;
}


void TakeoffSoilgate(Uint16 a)
{
	EvaRegs.T1CMPR=a;
	EvaRegs.T2CMPR=a;
	EvbRegs.T3CMPR=a;
	EvbRegs.T4CMPR=a;
}

