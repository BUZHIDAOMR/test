#include "sys.h"
#include "io.h"
#include "laser_ranging.h"
#include "string.h"
#include "usart3.h"
#include "timer.h"
#include "delay.h"
#include "usart.h"
#include "stdio.h"
#include "stop.h"
#include "moter.h"
#include "math.h"
#include "stdlib.h"
#include "feel.h"
#include "key.h"
#include "stop_control.h"
int ifh=0;
int fangxiang;
int value[]=//加速数组651——9
//d1左前角/激光;d2左后角/激光;d3右前角/超声波;d4右后角/超声波;
{
220,219,217,216,215,214,212,211,210,209,
207,206,205,204,203,201,200,199,198,197,
196,194,193,192,191,190,189,188,187,186,
185,184,182,181,180,179,178,177,176,175,
174,173,172,171,170,169,168,167,166,165,
164,163,163,161,161,160,159,158,157,156,
155,154,153,152,152,151,150,149,148,147,
147,146,145,144,143,142,142,141,140,139,
138,138,137,136,135,135,134,133,132,132,
131,130,129,129,128,127,127,126,125,124,//
124,123,122,122,121,120,120,119,118,118,
117,116,116,115,115,114,113,113,112,112,
111,110,110,109,109,108,107,107,106,106,
105,105,104,103,103,102,102,101,101,100,
100,99,99,98,98,97,97,96,96,95,
95,94,94,93,93,92,92,91,91,91,
90,90,89,89,88,88,87,87,87,86,
86,85,85,85,84,84,83,83,83,82,
82,81,81,81,80,80,79,79,79,78,
78,78,77,77,77,76,76,76,75,75,//
74,74,74,73,73,73,73,72,72,72,
71,71,71,70,70,70,69,69,69,69,
68,68,68,67,67,67,67,66,66,66,
65,65,65,65,64,64,64,64,63,63,
63,63,62,62,62,62,61,61,61,61,
61,60,60,60,60,59,59,59,59,58,
58,58,58,58,57,57,57,57,57,56,
56,56,56,56,55,55,55,55,55,55,
54,54,54,54,54,53,53,53,53,53,
53,52,52,52,52,52,52,51,51,51,//
51,51,51,50,50,50,50,50,50,50,
49,49,49,49,49,49,49,48,48,48,
48,48,48,48,47,47,47,47,47,47,
47,47,46,46,46,46,46,46,46,46,
46,45,45,45,45,45,45,45,45,45,
44,44,44,44,44,44,44,44,44,44,
43,43,43,43,43,43,43,43,43,43,
42,42,42,42,42,42,42,42,42,42,
42,42,41,41,41,41,41,41,41,41,
41,41,41,41,40,40,40,40,40,40,//
40,40,40,40,40,40,40,40,39,39,
39,39,39,39,39,39,39,39,39,39,
39,39,39,39,38,38,38,38,38,38,
38,38,38,38,38,38,38,38,38,38,
38,38,37,37,37,37,37,37,37,37,
37,37,37,37,37,37,37,37,37,37,
37,37,37,36,36,36,36,36,36,36,
36,36,36,36,36,36,36,36,36,36,
36,36,36,36,36,36,36,36,35,35,
35,35,35,35,35,35,35,35,35,35,//
};

int value1[]=//加速数组
{   
1157,1157,1147,1147,1147,1147,1147,1147,1136,1136,
1136,1136,1136,1126,1126,1126,1126,1116,1116,1116,
1116,1116,1106,1106,1106,1096,1096,1096,1096,1087,
1087,1087,1078,1078,1078,1078,1068,1068,1068,1059,
1059,1050,1050,1050,1042,1042,1042,1033,1033,1025,
1025,1016,1016,1016,1008,1008,1000,1000,992,992,
984,984,977,969,969,962,962,954,954,947,
940,940,933,926,926,919,912,912,906,899,
893,893,887,880,874,868,862,862,856,850,
845,839,833,828,822,817,812,806,801,796,
791,786,776,772,767,762,758,749,744,740,
731,727,723,714,710,706,698,694,687,683,
676,668,665,658,651,648,641,635,628,625,
619,613,607,601,595,590,584,579,573,566,
561,556,551,546,539,534,527,523,517,512,
506,502,496,490,486,481,475,470,465,460,
456,451,445,440,436,431,427,421,417,413,
407,403,398,394,389,386,381,377,372,369,
364,360,356,352,348,344,340,336,332,329,
325,321,317,314,310,307,303,300,297,293,
290,287,283,280,278,274,271,268,265,263,
260,257,254,252,249,246,243,241,238,236,
233,231,229,226,224,222,219,217,215,213,
210,208,207,205,203,201,199,197,195,193,
191,189,188,186,184,183,181,179,178,176,
175,173,172,171,169,168,166,165,164,162,
161,160,159,157,156,155,154,153,152,151,
150,148,147,146,145,144,144,143,142,141,
140,139,138,137,136,136,135,134,133,133,
132,131,130,130,129,128,128,127,126,126,
125,125,124,123,123,122,122,121,121,120,
120,119,119,118,118,117,117,116,116,115,
115,115,114,114,113,113,113,112,112,112,
111,111,111,110,110,110,109,109,109,108,
108,108,108,107,107,107,107,106,106,106,
106,105,105,105,105,105,104,104,104,104,
104,103,103,103,103,103,102,102,102,102,
102,102,102,101,101,101,101,101,101,101,
100,100,100,100,100,100,100,100,99,99,
99,99,99,99,99,99,99,99,99,98,
98,98,98,98,98,98,98,98,98,98,
98,97,97,97,97,97,97,97,97,97,
97,97,97,97,97,97,97,97,96,96,
96,96,96,96,96,96,96,96,96,96,
96,96,96,96,96,96,96,96,96,96,
96,96,95,95,95,95,95,95,95,95,
95,95,95,95,95,95,95,95,95,95,
95,95,95,95,95,95,95,95,95,95,
95,95,95,95,95,95,95,95,95,95,
95,95,95,95,95,95,95,95,95,95,
};


void speed_up_CNT_ms(int ia,int CNT,u16 nms,s32 step)//毫秒级加速改动过 1，2号取反
{//可控加速的方式、速度峰值、加速ms时基、加速步数
    int Count=0;//加速标志数
    int Speed;//设定加速速度变量
    switch(ia)//设定四个步进电机目标步数
    {
    case 0:
        SetpMotor_SetStep(0,step);	//后退
        SetpMotor_SetStep(1,step);
        SetpMotor_SetStep(2,-step);
        SetpMotor_SetStep(3,-step);
        break;
    case 1:
        SetpMotor_SetStep(0,-step);	//前进
        SetpMotor_SetStep(1,-step);
        SetpMotor_SetStep(2,step);
        SetpMotor_SetStep(3,step);
        break;
    case 2:
        SetpMotor_SetStep(0,-step);	//左移
        SetpMotor_SetStep(1,step);
        SetpMotor_SetStep(2,-step);
        SetpMotor_SetStep(3,step);
        break;
    case 3:
        SetpMotor_SetStep(0,step);	//右移
        SetpMotor_SetStep(1,-step);
        SetpMotor_SetStep(2,step);
        SetpMotor_SetStep(3,-step);
        break;
    }


    for(Count = 0; Count<CNT; Count++)//每隔nms重复设定速度值，越小越快
    {
        Speed=value[Count];//设定加速速度
        delay_ms(nms);//加速时基传递参数
        SetpMotor_SetSpeed(0,Speed);//设定4个电机步数
        SetpMotor_SetSpeed(1,Speed);
        SetpMotor_SetSpeed(2,Speed);
        SetpMotor_SetSpeed(3,Speed);
        
    }

}

void speed_up_CNT_ms1(int ia,int CNT,u16 nms,s32 step)//毫秒级加速第二个数组
{//可控加速的方式、速度峰值、加速ms时基、加速步数
    int Count=0;//加速标志数
    int Speed;//设定加速速度变量
    switch(ia)//设定四个步进电机目标步数
    {
    case 0:
        SetpMotor_SetStep(0,step);	//后退
        SetpMotor_SetStep(1,step);
        SetpMotor_SetStep(2,-step);
        SetpMotor_SetStep(3,-step);
        break;
    case 1:
        SetpMotor_SetStep(0,-step);	//前进
        SetpMotor_SetStep(1,-step);
        SetpMotor_SetStep(2,step);
        SetpMotor_SetStep(3,step);
        break;
    case 2:
        SetpMotor_SetStep(0,-step);	//左移
        SetpMotor_SetStep(1,step);
        SetpMotor_SetStep(2,-step);
        SetpMotor_SetStep(3,step);
        break;
    case 3:
        SetpMotor_SetStep(0,step);	//右移
        SetpMotor_SetStep(1,-step);
        SetpMotor_SetStep(2,step);
        SetpMotor_SetStep(3,-step);
        break;
    }


    for(Count = 0; Count<CNT; Count++)//每隔nms重复设定速度值，越小越快
    {
        Speed=value1[Count];//设定加速速度
        delay_ms(nms);//加速时基传递参数
        SetpMotor_SetSpeed(0,Speed);//设定4个电机步数
        SetpMotor_SetSpeed(1,Speed);
        SetpMotor_SetSpeed(2,Speed);
        SetpMotor_SetSpeed(3,Speed);
        
    }

}

void speed_up_CNT_ms2(int ia,int CNT,u16 nms,s32 step)//毫秒级加速出赛道
{//可控加速的方式、速度峰值、加速ms时基、加速步数
    int Count=0;//加速标志数
    int Speed;//设定加速速度变量
    switch(ia)//设定四个步进电机目标步数
    {
    case 2:
        SetpMotor_SetStep(0,-step);	//左移
        SetpMotor_SetStep(1,step);
        SetpMotor_SetStep(2,-step);
        SetpMotor_SetStep(3,step);
        break;
    }


    for(Count = 0; Count<CNT; Count++)//每隔nms重复设定速度值，越小越快
    {
        Speed=value[Count];//设定加速速度
        delay_ms(nms);//加速时基传递参数
        SetpMotor_SetSpeed(0,Speed);//设定4个电机步数
        SetpMotor_SetSpeed(1,Speed);
        SetpMotor_SetSpeed(2,(int)Speed*0.9);
        SetpMotor_SetSpeed(3,Speed);
        
    }

}

void run(int ia,s32 step,s32 Speed)//匀速行驶
{   switch(ia)
    {
    case 0:
        SetpMotor_SetStep(0,step);	//后退
        SetpMotor_SetStep(1,step);
        SetpMotor_SetStep(2,-step);
        SetpMotor_SetStep(3,-step);
        break;
    case 1:
        SetpMotor_SetStep(0,-step);	//前进
        SetpMotor_SetStep(1,-step);
        SetpMotor_SetStep(2,step);
        SetpMotor_SetStep(3,step);
        break;
    case 2:
        SetpMotor_SetStep(0,-step);	//左移
        SetpMotor_SetStep(1,step);
        SetpMotor_SetStep(2,-step);
        SetpMotor_SetStep(3,step);
        break;
    case 3:
        SetpMotor_SetStep(0,step);	//右移
        SetpMotor_SetStep(1,-step);
        SetpMotor_SetStep(2,step);
        SetpMotor_SetStep(3,-step);
        break;
		case 6://右侧靠近栏板
		    SetpMotor_SetStep(0,step);	
        SetpMotor_SetStep(1,-step);
        SetpMotor_SetStep(2,-step);
        SetpMotor_SetStep(3,-step);
				break;
		case 7://右侧远离栏板
				SetpMotor_SetStep(0,step);	
        SetpMotor_SetStep(1,-step);
        SetpMotor_SetStep(2,step);
        SetpMotor_SetStep(3,step);
		    break;
		case 8://左侧靠近栏板
				SetpMotor_SetStep(0,step);	
        SetpMotor_SetStep(1,step);
        SetpMotor_SetStep(2,step);
        SetpMotor_SetStep(3,-step);
		    break;
		case 9://左侧远离栏板
				SetpMotor_SetStep(0,-step);	
        SetpMotor_SetStep(1,-step);
        SetpMotor_SetStep(2,step);
        SetpMotor_SetStep(3,-step);
		    break;
    }
    SetpMotor_SetSpeed(0,Speed);
    SetpMotor_SetSpeed(1,Speed);
    SetpMotor_SetSpeed(2,Speed);
    SetpMotor_SetSpeed(3,Speed);
}

//void run1(int ia,s32 step,s32 Speed)//出赛道用
//{   switch(ia)
//    {
//    case 6:
//        SetpMotor_SetStep(0,step);	
//        SetpMotor_SetStep(1,-step);
//        SetpMotor_SetStep(2,step);
//        SetpMotor_SetStep(3,-step);
//		    SetpMotor_SetSpeed(0,Speed);
//        SetpMotor_SetSpeed(1,Speed);
//        SetpMotor_SetSpeed(2,Speed);
//        SetpMotor_SetSpeed(3,Speed*0.8);//
//        break;
//    }
//}

//void run2(int ia,s32 step,s32 Speed)//出垄道调正
//{   switch(ia)
//    {
//    case 0:
//        SetpMotor_SetStep(0,-step);	//左侧前进
//        SetpMotor_SetStep(1,-step);
//        SetpMotor_SetStep(2,0);
//        SetpMotor_SetStep(3,0);
//        break;
//    case 1:
//        SetpMotor_SetStep(0,0);	//右侧前进
//        SetpMotor_SetStep(1,0);
//        SetpMotor_SetStep(2,step);
//        SetpMotor_SetStep(3,step);
//        break;
//    case 2:
//        SetpMotor_SetStep(0,step);	//左侧后退
//        SetpMotor_SetStep(1,step);
//        SetpMotor_SetStep(2,0);
//        SetpMotor_SetStep(3,0);
//        break;
//    case 3:
//        SetpMotor_SetStep(0,0);	//右侧后退
//        SetpMotor_SetStep(1,0);
//        SetpMotor_SetStep(2,-step);
//        SetpMotor_SetStep(3,-step);
//        break;
//    }
//    SetpMotor_SetSpeed(0,Speed);
//    SetpMotor_SetSpeed(1,Speed);
//    SetpMotor_SetSpeed(2,Speed);
//    SetpMotor_SetSpeed(3,Speed);
//}


void speed_up_CNT_us(int ia,int CNT,u32 nus,s32 step)
{//可控加速的方式、速度峰值、加速us时基、加速步数
    int Count=0;//加速标志数
    int Speed;//设定加速速度变量
    switch(ia)//设定四个步进电机目标步数
    {
    case 2:
        SetpMotor_SetStep(0,-step);	//左移
        SetpMotor_SetStep(1,step);
        SetpMotor_SetStep(2,-step);
        SetpMotor_SetStep(3,step);
        break;
    case 3:
        SetpMotor_SetStep(0,step);	//右移
        SetpMotor_SetStep(1,-step);
        SetpMotor_SetStep(2,step);
        SetpMotor_SetStep(3,-step);
        break;
    case 4:
        SetpMotor_SetStep(0,step);	//正旋
        SetpMotor_SetStep(1,step);
        SetpMotor_SetStep(2,step);
        SetpMotor_SetStep(3,step);
        break;
    case 5:
        SetpMotor_SetStep(0,-step);	//逆旋
        SetpMotor_SetStep(1,-step);
        SetpMotor_SetStep(2,-step);
        SetpMotor_SetStep(3,-step);
        break;
    }


    for(Count = 0; Count<CNT; Count++)//每隔2ms重复设定速度值，越小越快
    {
        Speed=value[Count];//设定加速速度
        delay_us(nus);//加速时基传递参数
        SetpMotor_SetSpeed(0,Speed);//设定4个电机步数
        SetpMotor_SetSpeed(1,Speed);
        SetpMotor_SetSpeed(2,Speed);
        SetpMotor_SetSpeed(3,Speed);
    }
}

//微秒级自调加速
void speed_up_CNT_us_selfcontrol(int ia,int CNT,u32 nus,s32 step)//改动过
{//可控加速的方式、速度峰值、加速us时基、加速步数
    int Count=0;//加速标志数
    int Speed;//设定加速速度变量
    switch(ia)//设定四个步进电机目标步数
    {
			
		//case0,case1 前进系列	
    case 0://左转
        SetpMotor_SetStep(0,-step);	
        SetpMotor_SetStep(1,-step);
        SetpMotor_SetStep(2,step);
        SetpMotor_SetStep(3,step);
		for(Count = 0; Count<CNT; Count++)
    {
        Speed=value1[Count];//设定加速速度
        delay_us(nus);//加速时基传递参数
        SetpMotor_SetSpeed(0,40);//设定4个电机步数
        SetpMotor_SetSpeed(1,40);
        SetpMotor_SetSpeed(2,25);
        SetpMotor_SetSpeed(3,25);
    }
        break;
    case 1://右转
        SetpMotor_SetStep(0,-step);	
        SetpMotor_SetStep(1,-step);
        SetpMotor_SetStep(2,step);
        SetpMotor_SetStep(3,step);
		for(Count = 0; Count<CNT; Count++)
    {
        Speed=value1[Count];//设定加速速度
        delay_us(nus);//加速时基传递参数
        SetpMotor_SetSpeed(0,25);//设定4个电机步数
        SetpMotor_SetSpeed(1,25);
        SetpMotor_SetSpeed(2,40);
        SetpMotor_SetSpeed(3,40);
    }
        break;
		
		//case2,case3 后退系列
    case 2://左转
        SetpMotor_SetStep(0,step);	
        SetpMotor_SetStep(1,step);
        SetpMotor_SetStep(2,-step);
        SetpMotor_SetStep(3,-step);
		for(Count = 0; Count<CNT; Count++)
    {
        Speed=value1[Count];//设定加速速度
        delay_us(nus);//加速时基传递参数
        SetpMotor_SetSpeed(0,25);//设定4个电机步数
        SetpMotor_SetSpeed(1,25);
        SetpMotor_SetSpeed(2,40);
        SetpMotor_SetSpeed(3,40);
    }
        break;
    case 3://右转
        SetpMotor_SetStep(0,step);	
        SetpMotor_SetStep(1,step);
        SetpMotor_SetStep(2,-step);
        SetpMotor_SetStep(3,-step);
		for(Count = 0; Count<CNT; Count++)
    {
        Speed=value1[Count];//设定加速速度
        delay_us(nus);//加速时基传递参数
        SetpMotor_SetSpeed(0,40);//设定4个电机步数
        SetpMotor_SetSpeed(1,40);
        SetpMotor_SetSpeed(2,25);
        SetpMotor_SetSpeed(3,25);
    }
        break;
		
		//同一侧同时碰到
    case 4://左移
        SetpMotor_SetStep(0,-step);
        SetpMotor_SetStep(1,step);
        SetpMotor_SetStep(2,-step);
        SetpMotor_SetStep(3,step);
		for(Count = 0; Count<CNT; Count++)
    {
        Speed=value1[Count];//设定加速速度
        delay_us(nus);//加速时基传递参数
        SetpMotor_SetSpeed(0,Speed);//设定4个电机步数
        SetpMotor_SetSpeed(1,Speed);
        SetpMotor_SetSpeed(2,Speed);
        SetpMotor_SetSpeed(3,Speed);
    }
        break;
    case 5://右移
        SetpMotor_SetStep(0,step);
        SetpMotor_SetStep(1,-step);
        SetpMotor_SetStep(2,step);
        SetpMotor_SetStep(3,-step);
		for(Count = 0; Count<CNT; Count++)
    {
        Speed=value1[Count];//设定加速速度
        delay_us(nus);//加速时基传递参数
        SetpMotor_SetSpeed(0,Speed);//设定4个电机步数
        SetpMotor_SetSpeed(1,Speed);
        SetpMotor_SetSpeed(2,Speed);
        SetpMotor_SetSpeed(3,Speed);
    }
        break;
    }
}

void reset_step(void)
{
  motor[0].step=motor[0].target=0;//清零步数，停止
  motor[1].step=motor[1].target=0;
  motor[2].step=motor[2].target=0;
  motor[3].step=motor[3].target=0;
}


void step_wait(void)//等待当前步数达到目标步数，防止未达到目标步数就执行下一步程序
{   int wait=0;
    if(motor[0].target!=motor[0].step)//如果当前步数！=当前步数
    wait=1;//启动步数等待标志位
    while(wait)
    {
        if(motor[0].target==motor[0].step&&motor[2].target==motor[2].step&&motor[1].target==motor[1].step&&motor[3].target==motor[3].step)//如果 0号电机当前步数=当前步数
        {
            motor[0].step=motor[0].target=0;//清零步数
            motor[1].step=motor[1].target=0;
            motor[2].step=motor[2].target=0;
            motor[3].step=motor[3].target=0;
            wait=0;//结束循环
        }
    }

}



void GO(int ib)//行进函数，分别由后退、前进、左移、右移
{

    int going_wait=0;//初始化步数等待标志位
	  int d1,d2,d3,d4;//d1左前角/激光;d2左后角/激光;d3右前角/超声波;d4右后角/超声波;
	  
    switch(ib)
		{
		case 0://后退
		fangxiang=0;
		self_control2();
		speed_up_CNT_ms(0,500,1,1200);//35
		while(1)
		{
			d2= Senor_Using(3);
			if(d2<300)
			{
				self_control2();
				reset_step();
				run(0,1200,70);
				if(L_rear==0&&R_rear==0)delay_ms(10);
				if(L_rear==0&&R_rear==0)break;
			}
			else if(GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_1) == 1)
			{
				reset_step();
				zhongduan();                          // 直接开始补苗
			}
      else
			{
				self_control2();
				reset_step();
				run(0,1200,35);
			}	
		}
		reset_step();
		delay_ms(1000);
        break;
		
		case 7://特殊后退
		fangxiang=0;
		self_control2();
		speed_up_CNT_ms(0,500,1,1200);//35
		while(1)
		{
			d2= Senor_Using(3);
			if(d2<300)
			{
				self_control2();
				reset_step();
				run(0,1200,70);
				if(L_rear==0&&R_rear==0)delay_ms(10);
				if(L_rear==0&&R_rear==0)break;
			}
      else
			{
				self_control2();
				reset_step();
				run(0,1200,35);
			}	
		}
		reset_step();
		delay_ms(1000);
        break;
			
		case 1://前进
		fangxiang=1;
		self_control();
		speed_up_CNT_ms(1,500,1,1200);//35
		while(1)
		{
			d3=Senor_Using(1);//右前
			if(d3<300)
			{	
				self_control();
				reset_step();
				run(1,1200,70);
				if(L_front==0&&R_front==0)delay_ms(10);
				if(L_front==0&&R_front==0)break;
			}
			else if(GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_1) == 1)
			{
				reset_step();
				zhongduan();                          // 直接开始补苗
			}
			else
			{
				self_control();
				reset_step();
				run(1,1200,35);
			}
		}
		reset_step();
		delay_ms(1000);
        break;
		case 8:
			d1=Measure();
		if(d1>100)
		{
			speed_up_CNT_us(5,303,1,100);//旋
			step_wait();
		}
		reset_step();
		break;
		
		case 9:
      d2= Senor_Using(3);
		if(d2>100)
		{
			speed_up_CNT_us(4,303,1,100);//正旋
			step_wait();
		}
		reset_step();
		break;
		
		case 3: //循环时垄道后端左移
		speed_up_CNT_ms(2,400,2,50000);
		while(1)
		{
			d2= Senor_Using(3);
			d4= Senor_Using(2);
			if(d2<150&&d4<150)break;
		}
    going_wait=1;//启动步数等待标志位
    while(going_wait)
    {
	    if(L_front==0)
			  delay_ms(10);
			if(L_front==0)//当左前角光电检测到笼道后端面
      {
              going_wait=0;//结束步数等待
      }		
    }
		 while(1)
		 {
	     if(L_front==1)
			  delay_ms(10);
			 if(L_front==1)
			 {
				 break;
			 }				 
     }
		 delay_ms(200);
		 reset_step();
     break;
			
		 case 4://循环时垄道前端左移
		 speed_up_CNT_ms(2,400,2,50000);
     going_wait=1;//启动步数等待标志位
     while(going_wait)
     {
	     if(L_rear==0)
				 delay_ms(10);
			 if(L_rear==0)//当左上角光电检测到笼道后端面
       {
         going_wait=0;//结束步数等待
       }		
     }
		 while(1)
		 {
			 if(L_rear==1)
				 delay_ms(10);
			 if(L_rear==1)
			 {
				 break;
			 }				 
     }
		 delay_ms(200);
		 reset_step();	
        break;
		
		  case 5: //循环时垄道后端右移
		 speed_up_CNT_ms2(3,400,2,50000);
		 
		 case 6: //循环时垄道后端右移
		 going_wait=1;//启动步数等待标志位
     while(going_wait)
     {
       d4=Senor_Using(2);//右后
			 d2= Senor_Using(3);//左后
			 d3=Senor_Using(1);//右前
			 if(d3>100&&d4>100)//左后方光电检测不到垄道，即走出垄道
        {  
     
                  while(Senor_Using(3)<100);
									delay_ms(4500);
									reset_step();
									going_wait=0;
                
        }
				else if(going_wait==1&&d4>60)
            {  
//                    reset_step();
//                    while(d4>50)
//										{
//											d4=Senor_Using(2);
											run(6,50,40);
							        step_wait();
							        run(3,50000,40);
//										}
                    GO(6);//重新起步
                    going_wait=0;//结束循环
                
            }
//				else if(going_wait==1&&d2>60)
//            {  
////                    reset_step();
////                    while(d2>50)
////										{
////											d2=Senor_Using(2);
//											run(7,50,40);
//							        step_wait();
//							        run(3,50000,40);
////										}
//                    GO(6);//重新起步
//                    going_wait=0;//结束循环
//                
//            }
				 
//				
//        else if(going_wait==1&&(R_rear==1||d4>80))//右前方光电检测到意味将要撞垄，去掉了R_front==0
//            {   delay_ms(10);
//                if(going_wait==1&&(R_rear==1||d4>80))
//                {
//                    reset_step();
//                    while(d4>80)
//										{
//											run(6,30,100);
//										}
//                    GO(5);//重新起步
//                    going_wait=0;//结束循环
//                }
//            }
//				else if(going_wait==1&&(R_rear==0||d2>80))//左前方光电检测到意味将要撞垄
//						{ 
//							  delay_ms(10);
//                if(going_wait==1&&(R_rear==0||d2>80))
//                {
//                    reset_step();
//                    while(d2>80)
//										{
//											run(7,30,100);
//										}
//                    GO(5);//重新起步
//                    going_wait=0;//结束循环
//                }
//				
//						}
//						
						
						
						
        }
		 
	
		}
	}
