/**********************************************************************
*Copyright (c),2002-2012,Sifang Rolling Stock Research Institute Ltd.
*File Name:    main.c
*Author:       Xiameng   
*Date:         2012.11.16
*Version:      0.0.1
*HardWare:     TMS320F28335
*Description: shuyang SIV prog 
*Modification history:
    1.xiameng, 2012.11.16, first issue;
**********************************************************************/
#include "SY_Global.h"      //added by xiameng@20121115
#include "math.h"
#include "DSP2833x_Project.h"  // Device Headerfile and Examples Include File


#pragma CODE_SECTION(cpu_timer0_isr,"ramfuncs");


#define ADC_START()     AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1

#define ADC_CS0    		GpioDataRegs.GPADAT.bit.GPIO19
#define ADC_CS1    		GpioDataRegs.GPADAT.bit.GPIO16
#define ADC_CS2        	GpioDataRegs.GPBDAT.bit.GPIO56	 //PCB-V1.6 由GPIO22改为GPIO56

/******************************/
#define KM_STA          GpioDataRegs.GPADAT.bit.GPIO26   //KM1 state
#define HB_STA          GpioDataRegs.GPADAT.bit.GPIO24   //HB state
#define DC_1500         GpioDataRegs.GPADAT.bit.GPIO25   //DI2 DC_1500V
#define DC_750          GpioDataRegs.GPBDAT.bit.GPIO51   //DI3 DC_750V
#define ResetCmd        GpioDataRegs.GPADAT.bit.GPIO27   //DI4 reset button
#define Discharge       GpioDataRegs.GPBDAT.bit.GPIO50   //DI5 Discharge button
#define IGBTSTA         GpioDataRegs.GPADAT.bit.GPIO13   //DI6 PWM state
#define FBSTA           GpioDataRegs.GPBDAT.bit.GPIO49   //DI7 Res state
/******************************/
#define HB_CLOSE        GpioDataRegs.GPBDAT.bit.GPIO58   //DO1 K2_CLOSE 合高断
#define HB_OPEN         GpioDataRegs.GPBDAT.bit.GPIO59   //DO2 K3_CLOSE 分高断
#define KM_CLOSE        GpioDataRegs.GPBDAT.bit.GPIO60   //DO3 K1_CLOSE 合接触器
#define FaultLED        GpioDataRegs.GPBDAT.bit.GPIO61   //DO4 Fault led
#define FAN_START       GpioDataRegs.GPBDAT.bit.GPIO62   //FAN RUN CMD
#define PWMEnable       GpioDataRegs.GPADAT.bit.GPIO15   //PWM enable
/******************************/
int PWMcount=0;

unsigned int PWM_stop=0;
unsigned int RX_count=0;

unsigned int OVTCNT=0;                        /*ovt counter*/
unsigned int ChargeCNT = 0;                   /*precharge counter*/
unsigned int HBLatchCNT = 0;                  /*HB latch pulse counter*/
unsigned int temp_count = 0;                  /*over temperature counter*/
unsigned int HB_CLOSE_COUNT = 0;              /*HB cannot close fault counter*/
unsigned int HB_OPEN_COUNT = 0;               /*HB cannot open fault counter*/
unsigned int KM_OPEN_COUNT=0;                 /*KM cannot close fault counter*/
unsigned int KM_CLOSE_COUNT=0;                /*KM cannot open fault counter*/
unsigned int NetVoltage_1500_COUNT=0;
unsigned int NetVoltage_750_COUNT=0;
unsigned int TA_1500_COUNT=0;
unsigned int TA_750_COUNT=0;
unsigned int FANCNT0 = 0;
unsigned int FANCNT1 = 0;
unsigned int FBSTACNT = 0;
unsigned int HBopnCnt = 0;

unsigned int HB_STATE=0;                        //1: K2 close state; 0: K2 open state
unsigned int HB_CLOSE_STA=0;                    //1: HB self close ;0: HB self open
unsigned int FaultSTA=0;                        //1: fault flag ;0: normal flag
unsigned int ChargeSTA=0;                       //1: precharging ;0: precharge over or undone
unsigned int IGBTSTA_Back = 0;
unsigned int FanRunFlag = 0;                    //1: fan run cmd ;0: fan silent
unsigned int HBopnFlag = 0;
unsigned char KMCLSFautl = 0;
unsigned char KMCPNFault = 0;


unsigned int TimerCnt0=0;
unsigned int TimerCnt1=0;
unsigned int Test=0;


unsigned int LOWCNT=0;
/******************************/
float32 ADFilterDataOutput[12] = {0};
double NetVoltage=0;
double FCVoltage=0;
double OVTCurrent=0;
double PUTemp=0;
double NetVoltageFilter = 0;
double FCVoltageFilter = 0;
double OVTCurrentFilter = 0;
double PUTempFilter = 0;
double Raito;
/******************************/



Uint16 pwmpercent1=0;

// Prototype statements for functions found within this file.
interrupt void cpu_timer0_isr(void);
void Init_timer0(void);
void start_timer0(void);
void stop_timer0(void);
void enabledog(void);


void ADC_initial(void);
Uint16 ADC_FILTER(Uint16 ad_channel_select);
Uint16 ADC_RESULT(Uint16 ad_result_get);
Uint16 ADC_spi(Uint16 b);



void Gpio_select(void);
void InitPwm(void);
void RXD_Comm(void);
void TXD_Comm(void);
void DataRealFill(void);
void delay(Uint16 count);
int32  move_average_filter(int32 data_buf[], int count);
void ControlData_Measure(void);

void start_timer0(void)
    {
	    CpuTimer0Regs.PRD.all = 30000;                //7500  50us
	    CpuTimer0Regs.TCR.bit.TSS = 0;
    }

void stop_timer0(void)
    {
	    CpuTimer0Regs.TCR.bit.TSS = 1;
    }

void Init_timer0(void)
    {
	    CpuTimer0Regs.TPR.all = 0;
	    CpuTimer0Regs.TIM.all = 0;
	    CpuTimer0Regs.TPRH.all = 0;
	    CpuTimer0Regs.TCR.bit.TSS = 1;
	    CpuTimer0Regs.TCR.bit.SOFT = 1;
	    CpuTimer0Regs.TCR.bit.FREE = 1;
	    CpuTimer0Regs.TCR.bit.TRB = 1;       //TIMH:TIM寄存器重新装载PRDH:PRD中的周期值
	    CpuTimer0Regs.TCR.bit.TIE = 1;
	    CpuTimer0.InterruptCount = 0;
    }

interrupt void cpu_timer0_isr(void)
    {
        ControlData_Measure();

        if(ResetCmd==1)/*ResetButton*/
	  	     {
                FaultLED = 0;
                FaultSTA = 0;
                EALLOW;
                SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
                EDIS;
                PWMEnable = 1;
                GpioDataRegs.GPBCLEAR.bit.GPIO59=1;
                GpioDataRegs.GPBCLEAR.bit.GPIO60=1;
                GpioDataRegs.GPBCLEAR.bit.GPIO58=1;
                ChargeSTA = 0;
                FanRunFlag = 0;
                HB_CLOSE_STA = 0;
                temp_count = 0;
                HB_CLOSE_COUNT = 0;
                HB_OPEN_COUNT = 0;
                KM_OPEN_COUNT = 0;
                KM_CLOSE_COUNT = 0;
                NetVoltage_1500_COUNT = 0;
                NetVoltage_750_COUNT = 0;
                TA_1500_COUNT = 0;
                TA_750_COUNT = 0;
                FANCNT0 = 0;
                FBSTACNT = 0;
                HBopnCnt = 0;
                HBopnFlag = 0;
	  	     }

        //if((Discharge==1)&&(HB_CLOSE_STA==0))/*DischargeButton*/
        if(Discharge==1)//&&(HB_CLOSE_STA==0))/*DischargeButton*/
		     {
        		Test = 2345;
                /****************Resistor Fault*****************/
                FAN_START = 1;
                FANCNT0++;
                if((FANCNT0>=15000)&(FBSTA==0))
                     {
                         FaultLED = 1;
                         FaultSTA = 1;
                         HB_CLOSE_STA = 0;
                         EALLOW;
                         SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
                         EDIS;
                         PWMEnable = 0;
                         GpioDataRegs.GPBSET.bit.GPIO59 = 1;
                         GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
                         Test =100;
                     }
                 if(FBSTA==1)
                     {
                	 	 Test = 2346;
                         FANCNT0 = 0;
                         FBSTACNT++;
                         if((FBSTACNT>=2500)&&(FanRunFlag==0))
                              {
                                   FanRunFlag = 1;
                                   FBSTACNT = 0;
                              }
                     }
                 if((FanRunFlag)&&(FBSTA==0))
                     {
                         FaultLED=1;
                         FaultSTA=1;
                         HB_CLOSE_STA = 0;
                         EALLOW;
                         SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
                         EDIS;
                         PWMEnable = 0;
                         GpioDataRegs.GPBSET.bit.GPIO59=1;
                         GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
                         Test =200;
                     }
                  else{;}
                 /****************Resistor Fault END*****************/
		         EALLOW;
		         SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
		         EDIS;
		   
		         PWMEnable = 1;
		         EPwm1Regs.AQCSFRC.bit.CSFA = 0x00;	     // 软件强制禁止
		         EPwm1Regs.CMPA.half.CMPA = 8000;       //8000*NetVoltage/2987;
		     }
        /******************KM1 FAULT*******************/
         if((ChargeSTA==1)&&(KM_STA==0))  //KM1 cannot close delay 1s
	          {
	              KM_CLOSE_COUNT++;
	              if(KM_CLOSE_COUNT > 5000)
	                  {
	                      FaultSTA = 1;
	                      HB_CLOSE_STA = 0;
	                      FaultLED = 1;

	                      EALLOW;
	                      SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	                      EDIS;
	                      PWMEnable = 0;
	                      GpioDataRegs.GPBSET.bit.GPIO59=1;
	                      GpioDataRegs.GPBCLEAR.bit.GPIO60=1;
	                      Test=1;
	                  }
	          }
         else
              {
                 KM_CLOSE_COUNT=0;
              }
         if((ChargeSTA==0)&&(KM_STA==1))  //KM1 cannot open delay 1s
	  	      {
	             KM_OPEN_COUNT++;
	  	
	             if(KM_OPEN_COUNT>5000)
	                 {
	                     FaultSTA=1;
	                     HB_CLOSE_STA=0;
	                     FaultLED=1;
	                     EALLOW;
	                     SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	                     EDIS;
	                     PWMEnable = 0;
	                     GpioDataRegs.GPBSET.bit.GPIO59=1;
	                     GpioDataRegs.GPBCLEAR.bit.GPIO60=1;
	                     Test=2;
	                 }
	  	      }
         else
		      {
	              KM_OPEN_COUNT=0;
		      }
         /******************KM1 FAULT END*******************/


         /******************NO CHOPPER YES CURRENT*******************/

         if((OVTCurrentFilter > 50)&&(HB_STA==0))  //50A
              {
                    OVTCNT++;
                    if (OVTCNT > 5000)
                         {
                              FaultSTA = 1;
                              HB_CLOSE_STA = 0;
                              FaultLED = 1;
                              EALLOW;
                              SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
                              EDIS;
                              PWMEnable = 0;
                              GpioDataRegs.GPBSET.bit.GPIO59=1;
                              GpioDataRegs.GPBCLEAR.bit.GPIO60=1;
                              Test =3;
                          }
                      }
                  else
                      {
                          OVTCNT = 0;
                      }

         /******************NO CHOPPER YES CURRENT*******************/

         /******************HB FAULT*******************/
	 
		 if((HB_CLOSE_STA==1)&&(HB_STA==0))//HB cannont close delay 1s
			  {
			      HB_CLOSE_COUNT++;
		   	
			      if(HB_CLOSE_COUNT>5000)
			          {
			              FaultSTA=1;
			              HB_CLOSE_STA=0;
			              FaultLED=1;
			              EALLOW;
			              SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
			              EDIS;
			              PWMEnable = 0;
			              GpioDataRegs.GPBSET.bit.GPIO59=1;
			              GpioDataRegs.GPBCLEAR.bit.GPIO60=1;
			              Test =400;
			          }
			  }
		 else
		   	  {
                   HB_CLOSE_COUNT=0;
		   	  }


		 if((HB_CLOSE_STA==0)&&(HB_STA==1))  //HB cannot open delay 1s
	         {	
			     HB_OPEN_COUNT++;
		   	
			     if(HB_OPEN_COUNT>10000)
			         {
			             FaultSTA = 1;
			             HB_CLOSE_STA = 0;
			             FaultLED = 1;
			             EALLOW;
			             SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
			             EDIS;
			             PWMEnable = 0;
			             GpioDataRegs.GPBSET.bit.GPIO59 = 1;
			             GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
			             Test =500;
			         }
	         }
		 else
		   	 {
                 HB_OPEN_COUNT=0;
		   	 }
		 /******************HB FAULT END*******************/

		 /******************OVER TEMPERATURE FAULT*******************/
		 if(PUTempFilter > 1785) //85度
		     {
		         temp_count++;
		         if(temp_count> 50)
		             {
		                 FaultSTA = 1;
		                 HB_CLOSE_STA = 0;
		                 FaultLED = 1;
		                 EALLOW;
		                 SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
		                 EDIS;
		                 PWMEnable = 0;
		                 GpioDataRegs.GPBSET.bit.GPIO59=1;
		                 GpioDataRegs.GPBCLEAR.bit.GPIO60=1;
		                 Test=600;
		             }
		     }
		 else
		     {
		         temp_count=0;
		     }
		 /******************Low Voltage Protect*******************/
		 if((NetVoltageFilter <= 100)&&(HBopnCnt<=12500)&&(HBopnFlag==0))
		     {
		          HBopnCnt++;
		          HB_CLOSE_STA = 0;
                  EALLOW;
                  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
                  EDIS;
                  PWMEnable = 0;
                  if(LOWCNT<=10000)
                  {
                	  LOWCNT++;
                  }
                  if(LOWCNT>10000)
                  {
                	  GpioDataRegs.GPBSET.bit.GPIO59 = 1;
                	  LOWCNT=0;
                  }

                  Test=2100;
		     }
		  if(HBopnCnt > 12500)
		      {
		          HBopnFlag = 1;
		          HBopnCnt = 0;
		          GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
		     }
		/******************Low Voltage Protect*******************/


		 /******************IGBT FAULT*******************/
		  if(((IGBTSTA==0)&(IGBTSTA_Back==1))||(IGBTSTA==0))
		      {
		           FaultSTA=1;
		           HB_CLOSE_STA=0;
		           FaultLED=1;
		           EALLOW;
		           SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
		           EDIS;
		           PWMEnable = 0;
		           GpioDataRegs.GPBSET.bit.GPIO59 = 1;
		           GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
		           Test=700;
		      }
		  else{;}
		 /******************IGBT FAULT END*******************/



		 /*********************1500V **********************/
		 if((DC_1500==1)&&(DC_750==0)&&(ResetCmd==0))
		     {

		        TimerCnt0++;
		        if(TimerCnt0 >= 5000)
		                       {
		                           TimerCnt1++;
		                           TimerCnt0 = 0;
		                       }

		         /****************Resistor Fault*****************/
		         FAN_START = 1;
		         FANCNT0++;
		         if((FANCNT0>=15000)&(FBSTA==0))
		             {
		                 FaultLED=1;
		                 FaultSTA=1;
		                 HB_CLOSE_STA=0;
		                 EALLOW;
		                 SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
		                 EDIS;
		                 PWMEnable = 0;
		                 GpioDataRegs.GPBSET.bit.GPIO59=1;
		                 GpioDataRegs.GPBCLEAR.bit.GPIO60=1;
		                 Test=800;
		             }
		         if(FBSTA==1)
		             {
		                 FANCNT0 = 0;
		                 FBSTACNT++;
		                 if((FBSTACNT>=2500)&&(FanRunFlag==0))
		                     {
		                         FanRunFlag = 1;
		                         FBSTACNT = 0;
		                     }
		             }
		         if((FanRunFlag)&&(FBSTA==0))
		             {
		                 FaultLED=1;
		                 FaultSTA=1;
		                 HB_CLOSE_STA=0;
		                 EALLOW;
		                 SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
		                 EDIS;
		                 PWMEnable = 0;
		                 GpioDataRegs.GPBSET.bit.GPIO59=1;
		                 GpioDataRegs.GPBCLEAR.bit.GPIO60=1;
		                 Test=900;
		             }
		          else{;}
		        /****************Resistor Fault END*****************/
		         /****************OverVoltage Fault*****************/
		         if(NetVoltageFilter>=2200) //1500V系统电压超过2200V默认过压故障
		             {
		                 NetVoltage_1500_COUNT++;
		                 if(NetVoltage_1500_COUNT>100)
		                     {
		                         FaultLED=1;
		                         FaultSTA=1;
		                         HB_CLOSE_STA=0;
		                         EALLOW;
		                         SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
		                         EDIS;
		                         PWMEnable = 0;
		                         GpioDataRegs.GPBSET.bit.GPIO59=1;
		                         GpioDataRegs.GPBCLEAR.bit.GPIO60=1;
		                         Test=1100;
		                     }
		             }
		         else
		             {
		                 NetVoltage_1500_COUNT=0;
		             }
		         /****************OverVoltage Fault End*****************/

		         /****************OverCurrent Fault*****************/
		         if(OVTCurrentFilter > 600) //600A 1365
		             {
		                 TA_1500_COUNT++;
		                 if(TA_1500_COUNT>20)
		                     {
		                         FaultSTA=1;
		                         HB_CLOSE_STA=0;
		                         FaultLED = 1;
		                         EALLOW;
		                         SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
		                         EDIS;
		                         PWMEnable = 0;
		                         GpioDataRegs.GPBSET.bit.GPIO59=1;
		                         GpioDataRegs.GPBCLEAR.bit.GPIO60=1;
		                         Test=1200;
		                     }
		             }
		         else
		             {
		                 TA_1500_COUNT=0;
		             }
		         /****************OverCurrent Fault End*****************/

		         /****************PreCharge Start*****************/
		         if (((NetVoltageFilter-FCVoltageFilter) >= 1200)&&(ChargeSTA==0)&&(FaultSTA==0))
		             {
		                 KM_CLOSE=1;
		                 ChargeSTA=1;
		                 HBopnFlag = 0;
		             }
		         else{;}

		         if(((NetVoltageFilter-FCVoltageFilter) <= 50)&&(ChargeSTA==1)&&(FaultSTA==0))
		             {
		                 ChargeCNT++;
		                 if((HBLatchCNT < 2500)&&(HB_STATE==0))
		                     {
		                         HB_CLOSE = 1;    // close K2
		                         HBLatchCNT++;    // latch counter
		                         HB_CLOSE_STA=1;  // HB self close
		                     }
		                 else{;}
		                 if(HBLatchCNT >= 2500)   //HB 0.5s latch pulse
		                     {
		                         HB_CLOSE = 0;    // open K2
		                         HBLatchCNT=0;    // latch clear
		                         HB_STATE = 1;    // k2 open state
		                     }
		                 else{;}
		                 if(ChargeCNT >= 5000)    // precharge over
		                     {
		                         KM_CLOSE = 0;    // open K1
		                         ChargeSTA = 0;
		                         ChargeCNT = 0;
		                         HB_STATE = 0;
		                     }
		                 else{;}
		             }
		         /****************PreCharge Over*****************/

		         /******************PWM脉宽限制200ms 关断25ms*******************/
		         if((FCVoltageFilter > 1900)&&(FaultSTA==0)&&(HB_STA==1))  //1750V  2948//1900
		             {
		                 EALLOW;
		                 SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
		                 EDIS;
		                 PWMEnable = 1;    // PWM enable ok

		                 EPwm1Regs.AQCSFRC.bit.CSFA = 0x00;
		                 Raito = FCVoltageFilter/1900;
		                 if(Raito>=1.56) {Raito = 1.56;}
		                 EPwm1Regs.CMPA.half.CMPA = 8000*Raito;
		                 Test=2000;
		             }
		         else{;}
	
		         if(FCVoltageFilter < 1850)
		             {
		                 EPwm1Regs.AQCSFRC.bit.CSFA = 0x01;
		                 PWMEnable = 0;    // PWM enable close
		             }
		         else{;}
		         /************************PWM OVER*************************/
		     }
		 /******************1500V END*******************/






		 /******************750v*******************/
		 if((DC_1500==0)&&(DC_750==1)&&(ResetCmd==0))
		     {
		         /****************Resistor Fault*****************/
		         FAN_START = 1;
		         FANCNT0++;
		         if((FANCNT0>=15000)&(FBSTA==0))
		             {
		                  FaultLED = 1;
		                  FaultSTA = 1;
		                  HB_CLOSE_STA = 0;
		                  EALLOW;
		                  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
		                  EDIS;
		                  PWMEnable = 0;
		                  GpioDataRegs.GPBSET.bit.GPIO59 = 1;
		                  GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
		             }
		          if(FBSTA==1)
		             {
		                  FANCNT0 = 0;
		                  FBSTACNT++;
		                  if((FBSTACNT>=2500)&&(FanRunFlag==0))
		                      {
		                          FanRunFlag = 1;
		                          FBSTACNT = 0;
		                      }
		             }
		          if((FanRunFlag)&&(FBSTA==0))
		             {
		                  FaultLED=1;
		                  FaultSTA=1;
		                  HB_CLOSE_STA=0;
		                  EALLOW;
		                  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
		                  EDIS;
		                  PWMEnable = 0;
		                  GpioDataRegs.GPBSET.bit.GPIO59=1;
		                  GpioDataRegs.GPBCLEAR.bit.GPIO60=1;
		             }
		          else{;}
		         /****************Resistor Fault END*****************/

		        /****************OverVoltage Fault*****************/
		         if(NetVoltageFilter > 1050) //750V系统电压超过1050V默认过压故障
		             {
		                 NetVoltage_750_COUNT++;
        
		                 if(NetVoltage_750_COUNT > 100)
		                     {
		                         FaultLED = 1;
		                         FaultSTA = 1;
		                         HB_CLOSE_STA = 0;
		                         EALLOW;
		                         SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
		                         EDIS;
		                         PWMEnable = 0;
		                         GpioDataRegs.GPBSET.bit.GPIO59 = 1;
		                         GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
		                         Test=800;
		                     }
		             }
		         else
		             {
		                 NetVoltage_750_COUNT=0;
		             }
		         /****************OverVoltage Fault END*****************/

		         /****************OverCurrent Fault*****************/
		         if(OVTCurrentFilter > 300) //300A  680
		             {
		                 TA_750_COUNT++;
		                 if(TA_750_COUNT>5)
		                     {
		                         FaultLED = 1;
		                         FaultSTA = 1;
		                         HB_CLOSE_STA = 0;
		                         EALLOW;
		                         SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
		                         EDIS;
		                         PWMEnable = 0;
		                         GpioDataRegs.GPBSET.bit.GPIO59=1;
		                         GpioDataRegs.GPBCLEAR.bit.GPIO60=1;
		                         Test=900;
		                     }
		             }
		         else
		             {
		                 TA_750_COUNT = 0;
		             }
		         /****************OverCurrent Fault End*****************/
	   
		         /****************PreCharge Start*****************/
		         if (((NetVoltageFilter-FCVoltageFilter) > 500)&&(ChargeSTA==0)&&(FaultSTA==0))   //大于 500伏
		             {
		                 KM_CLOSE = 1;
		                 ChargeSTA = 1;
		                 HBopnFlag = 0;
		             }
		         if(((NetVoltageFilter - FCVoltageFilter) < 50)&&(ChargeSTA==1)&&(FaultSTA==0))  //小于50V
		             {
		                 ChargeCNT++;
		                 if(( HBLatchCNT < 2500)&&(HB_STATE==0))
		                     {
		                         HB_CLOSE = 1;    // close K2
		                         HBLatchCNT++;    // latch counter
		                         HB_CLOSE_STA = 1;  // HB self close
		                     }
		                 if(HBLatchCNT>=2500) //HB 0.5s脉冲
		                     {
		                         HB_CLOSE = 0;    // open K2
		                         HBLatchCNT=0;    // latch clear
		                         HB_STATE = 1;    // k2 open state
		                     }
		                 if(ChargeCNT > 5000)
		                    {
		                         KM_CLOSE = 0;    // open K1
		                         ChargeSTA = 0;
		                         ChargeCNT = 0;
		                         HB_STATE = 0;
		                    }
		             }
		         /****************PreCharge end*****************/

		         /******************PWM start*******************/
		         if((FCVoltageFilter >= 900)&&(FaultSTA==0)&&(HB_STA==1))	//850V 1431//900v
		             {
		                 EALLOW;
		                 SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
		                 EDIS;
		                 PWMEnable = 1;

		                 EPwm1Regs.AQCSFRC.bit.CSFA = 0x00;	   // 软件强制禁止
		                 Raito = FCVoltageFilter/900;
		                 if(Raito>=1.56) {Raito = 1.56;}
		                 EPwm1Regs.CMPA.half.CMPA = 8000*Raito;
		                 Test = 1000;
		             }
		
		         if(FCVoltageFilter < 850)	//1420//850v
		             {
		                 EPwm1Regs.AQCSFRC.bit.CSFA = 0x01;
		                 PWMEnable = 0;
		             }
		         /******************PWM end*******************/
		     }
		 /******************750V END*******************/

		 /******************FAN STOP*******************/

		 if((DC_1500==0)&&(DC_750==0)&&(ResetCmd==0)&&(Discharge==0))
		     {
		         FAN_START = 0;
		         FANCNT0 = 0;
		         FBSTACNT = 0;
		         FanRunFlag = 0;
		         ChargeSTA = 0;
		         temp_count = 0;
		         HB_CLOSE_COUNT = 0;
		         HB_OPEN_COUNT = 0;
		         KM_OPEN_COUNT = 0;
		         KM_CLOSE_COUNT = 0;
		         NetVoltage_1500_COUNT = 0;
		         NetVoltage_750_COUNT = 0;
		         TA_1500_COUNT = 0;
		         TA_750_COUNT = 0;
		         EALLOW;
		         SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
		         EDIS;
		         PWMEnable = 0;
		     }
		 else{;}
		 /******************FAN STOP*******************/

		 IGBTSTA_Back = IGBTSTA;
		 CpuTimer0.InterruptCount++;
   
		 PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
		 CpuTimer0Regs.TCR.bit.TIF = 1;
//   CpuTimer0Regs.TCR.bit.TRB = 1;    //deleted by xiameng@20130107
    }

Uint16 ADC_spi(Uint16 b)
{
    Uint16 DataConvSpi;
	Uint16 i;
    SpiaRegs.SPITXBUF = SpiaRegs.SPITXBUF; //???question by liuchao
     // Check against sent data
	 // Wait until data is received
	DataConvSpi = SpiaRegs.SPIRXBUF;	
    while(SpiaRegs.SPIFFRX.bit.RXFFST != 1){}//???question无限循环 liuchao!!!
    switch(b)						//chose the relevent ADC chip according to b
        {
		    case 1:
		        {
		            ADC_CS0 = 1; break;
		        }
		    case 2:
		        {
		            ADC_CS1 = 1; break;
		        }
		    case 3:
		        {
		             ADC_CS2 = 1; break;
		        }
		    default:
			break;
        }
   for(i=0;i<10;i++);
   switch(b)						//disable the relevent ADC chip according to b
        {
		    case 1:
		        {
		            ADC_CS0 = 0; break;
		        }
		    case 2:
		        {
		            ADC_CS1 = 0; break;
		        }
		    case 3:
		        {
		            ADC_CS2 = 0; break;
		        }
		    default:
			break;
    }
	return DataConvSpi;			
} 

Uint16 ADC_RESULT(Uint16 ad_channel_get)
{ 
	static Uint16 Charger_ain_bf_filter;

	if(ad_channel_get == 0)
	{
		Charger_ain_bf_filter = ((AdcRegs.ADCRESULT0)>>4);
		return Charger_ain_bf_filter;
	}
	else if(ad_channel_get == 1)
	{
		Charger_ain_bf_filter = ((AdcRegs.ADCRESULT1)>>4);
		return Charger_ain_bf_filter;
	}
	else if(ad_channel_get == 2)
	{
		Charger_ain_bf_filter = ((AdcRegs.ADCRESULT2)>>4);
		return Charger_ain_bf_filter;
	}
	else if(ad_channel_get == 3)
	{
		Charger_ain_bf_filter = ((AdcRegs.ADCRESULT3)>>4);
		return Charger_ain_bf_filter;
	}
	else if(ad_channel_get == 4)
	{
		Charger_ain_bf_filter = ((AdcRegs.ADCRESULT4)>>4);
		return Charger_ain_bf_filter;
	}
	else if(ad_channel_get == 5)
	{
		Charger_ain_bf_filter = ((AdcRegs.ADCRESULT5)>>4);
		return Charger_ain_bf_filter;
	}
	else if(ad_channel_get == 6)
	{
		Charger_ain_bf_filter = ((AdcRegs.ADCRESULT6)>>4);
		return Charger_ain_bf_filter;
	}
	else if(ad_channel_get == 7)
	{
		Charger_ain_bf_filter = ((AdcRegs.ADCRESULT7)>>4);
		return Charger_ain_bf_filter;
	}
	else if(ad_channel_get == 8)
	{
		Charger_ain_bf_filter = ((AdcRegs.ADCRESULT8)>>4);
		return Charger_ain_bf_filter;
	}
	else if(ad_channel_get == 9)
	{
		Charger_ain_bf_filter = ((AdcRegs.ADCRESULT9)>>4);
		return Charger_ain_bf_filter;
	}
	else if(ad_channel_get == 10)
	{
		Charger_ain_bf_filter = ((AdcRegs.ADCRESULT10)>>4);
		return Charger_ain_bf_filter;
	}
	else if(ad_channel_get == 11)
	{
		Charger_ain_bf_filter = ((AdcRegs.ADCRESULT11)>>4);
		return Charger_ain_bf_filter;
	}
	else if(ad_channel_get == 12)
	{
		Charger_ain_bf_filter = ADC_spi(3);
		return Charger_ain_bf_filter;
	}
	else return 0;	  
}

Uint16 ADC_FILTER(Uint16 ad_channel_select)
{
	static float32 RCfilter_Para = 0.2439;      //RCfilter_Para = t/(Ts+t),Ts为采样周期200us，t=RC。RC = 5us
//	static Uint16 DataOutput = 0;
	static float32  DataInput = 0;

	DataInput = ADC_RESULT(ad_channel_select);
    ADFilterDataOutput[ad_channel_select] = RCfilter_Para*(ADFilterDataOutput[ad_channel_select]-DataInput)+DataInput;
    return (Uint16)ADFilterDataOutput[ad_channel_select];		
}

int32  move_average_filter(int32 data_buf[], int count)
    {
        int32 sampe_value=0;
        int i;
        for(i=0;i<count;i++)
            sampe_value+= data_buf[i]/count;
        return(sampe_value);
    }
/***********************************************/
/*数据采集函数                                 */
/***********************************************/
void ControlData_Measure(void)
    {
        OVTCurrent = (float32)(ADC_FILTER(0)&0x0FFF)*0.441176470;
		FCVoltage = (float32)(ADC_FILTER(1)&0x0FFF)*0.59362279511;
		NetVoltage= (float32)(ADC_FILTER(2)&0x0FFF)*0.59362279511;
		PUTemp= (float32)(ADC_FILTER(6)&0x0FFF);

		NetVoltageFilter = 0.8*(NetVoltageFilter-NetVoltage)+NetVoltage;
		FCVoltageFilter = 0.8*(FCVoltageFilter-FCVoltage)+FCVoltage;
		OVTCurrentFilter = 0.8*(OVTCurrentFilter-OVTCurrent)+OVTCurrent;
		PUTempFilter = 0.8*(PUTempFilter-PUTemp)+PUTemp;
    }

void delay(Uint16 count)       	/*count=1 about 0.1us */
    {
        while(count>0)
            {
                count--;
                asm(" NOP");
                asm(" NOP");
                asm(" NOP");
                asm(" NOP");
                asm(" NOP");
                asm(" NOP");
            }
    }


void DataRealFill()
    {
        DataReal[0] =(Uint16)(ADC_FILTER(1)&0x0FFF)&0x0ff;
        DataReal[1] =(((Uint16)(ADC_FILTER(1)&0x0FFF))>>8)&0x0ff;

        DataReal[2] =(Uint16)(EPwm1Regs.CMPA.half.CMPA)&0x0ff;
        DataReal[3] =(Uint16)(EPwm1Regs.CMPA.half.CMPA)&0x0ff;

        DataReal[4] =(Uint16)(EPwm1Regs.CMPA.half.CMPA)&0x0ff;/*1-3*///Test
        DataReal[5] =(((Uint16)(EPwm1Regs.CMPA.half.CMPA))>>8)&0x0ff;

        DataReal[6] =(Uint16)(ADC_FILTER(2)&0x0FFF)&0x0ff;
        DataReal[7] =(((Uint16)(ADC_FILTER(2)&0x0FFF))>>8)&0x0ff;

        DataReal[8] =(Uint16)(PUTempFilter)&0x0ff;
        DataReal[9] =(((Uint16)(PUTempFilter))>>8)&0x0ff;

        DataReal[10] =(Uint16)(ADC_FILTER(0)&0x0FFF)&0x0ff;
        DataReal[11] =(((Uint16)(ADC_FILTER(0)&0x0FFF))>>8)&0x0ff;

        DataReal[12] =(Uint16)(Test)&0x0ff;
        DataReal[13] =(((Uint16)(Test))>>8)&0x0ff;

        DataReal[14] =(Uint16)(800)&0x0ff;
        DataReal[15] =(((Uint16)(800))>>8)&0x0ff;

        DataReal[18] = (KM_CLOSE&0x01);

        DataReal[19] = (HB_CLOSE&0x01);

        DataReal[27] = (KM_STA&0x01)+((FBSTA<<1)&0x02)+((DC_1500<<2)&0x04)+((DC_750<<3)&0x08)+\
                           ((ResetCmd<<4)&0x10)+((Discharge<<5)&0x20)+((HB_STA<<6)&0x40)+((0<<7)&0x80);

        DataReal[29] = (FAN_START&0x01);

        DataReal[39] = (IGBTSTA&0x01);

        DataReal[40] = (FaultSTA&0x01);
    }

void RXD_Comm()
{   
    Uint16 i=1;  
	Uint16 Rxd_check1=0;
	Uint16 Rxd_check2=0;
	Uint16 Rxd_counter=0;
	if(scia_rxd(Rxd_Data)) 
      {
	    DataRecvFromSerial[0] = *Rxd_Data;
	    if(DataRecvFromSerial[0]==0xa5)  
	      { 
	       for(i=1;i<8;i++)
	          {
	            do{	
	 	           Rxd_flag=scia_rxd(Rxd_Data);
	               Rxd_counter++;
		           if(Rxd_counter>=1000) return;
                   }
		        while (Rxd_flag!=1);  
		       Rxd_counter=0;
		       DataRecvFromSerial[i]=*Rxd_Data;    			          		            
		       } 
		  Rxd_check1 = DataRecvFromSerial[0]^DataRecvFromSerial[1]^DataRecvFromSerial[2]^DataRecvFromSerial[3]\
	                   ^DataRecvFromSerial[4]^DataRecvFromSerial[5]^DataRecvFromSerial[6];
	      Rxd_check2 = DataRecvFromSerial[7]; 
	       if(Rxd_check1 == Rxd_check2)
	         {
			  switch (DataRecvFromSerial[1])
	  		  {
	  	 	       case 0:           /* 建立通信*/
			  		{  
			  			TXD_Flag=0x01;
			  			break;  
			  		 }
			  		case 1:           /* data */
			  		  {  
				  		TXD_Flag=0x02;
				  		break;
			  		   }
			  		case 2:           /* 事件记录 */
			  		  { 	               
		                 EventReadCmd=1;
		                 break;   
			  		  }  
			  	    case 3:
			  	      {  	                 
		                EventStateCmd=1; /*事件记录状态发送*/
		                break; 
		              }   
		            case 4:
		              { 
		                EventErasCmd=1; /*事件记录擦除*/
		                break;
		               }    
		            case 5:
		              { 

		               		switch (DataRecvFromSerial[2])
		               		{
			               	case 1:  {
							pwmpercent1 =(DataRecvFromSerial[3]&0xFF)+((DataRecvFromSerial[4]&0xFF)<<8);
				  	        TXD_Flag=0x02;
							break;
							}
			               		case 2:  {; 	break; }
			               		default:
			               		{
									break;
			               		}
					}
		               			
		               		break;              		
		               }    
		             case 7:
		               { 
		                 TXD_Flag=0x07;
						 break;
		               }              	               
		            case 8:
		               { 		            
			            EventSaveCmd=1;               /*记录事件 */
			            break;
		               }
		            case 10:				/*事件管理器复位命令*/
			            {
			            	EventRstCmd=1;
							break;
			            } 
		            case 16:
		               {
		               		TDsave=1;	
		               		break;              		
		               } 
		               			            
		            case 33:       /*FaultSTA enable*/
		               {
		               		switch (DataRecvFromSerial[2])
							{
			               		case 1:  {  	break; }
			               		default:
			               		{
									break;
			               		}
							}
		               			
		               		break;              		
		               } 			            

		            case 34:      /*FaultSTA disable*/
		               {
						
			               		switch (DataRecvFromSerial[2])
								{
				               		case 1:	 {   	break; }
				               		default:
				               		{
										break;
				               		}
								}
							}
	   		  	default: 
	  		    {  
					TXD_Flag=0;
					break;
	  		    }       	             		
		               } 			             
		
	  		}
	  } else DataRecvFromSerial[0]=0;
   }else DataRecvFromSerial[0]=0;   
  } 	         

void TXD_Comm()
{
	Uint16 i,temp_var1,temp_var2,temp_var3,temp_var4;
	switch(TXD_Flag)
	{
		case 1:               /* 建立通信*/
		{
			scia_txd(0xa5); 
			scia_txd(0x00); 
			scia_txd(0xa5);
			TXD_Flag = 0;
			break;
		}
		case 2:               /* 实时监控数据发送*/
		{
			  TXDCheck=0xa5^0x01;   
 	       	       	      
	 	      scia_txd(0xa5);
	 	      scia_txd(0x01);  	      
	 	           
	 	      for(i=0;i<REALDATALENGTH;i++)
	 	      { 
	 	      	TXDCheck^=DataReal[i];   
	 	      	scia_txd(DataReal[i]);
	 	      }  
	 	      scia_txd(TXDCheck&0xff);
	 	      TXD_Flag=0;
			  break;
		}
        case 4:		/* 事件记录发送 */
        {
          if(DataEventReadOver==1) 	/*事件记录寥⊥晗*/
          {  
              DataEventReadOver=0;
              scia_txd(0xa5);
	 	      scia_txd(0x02); 
	 	      scia_txd(0xaa);
	 	      scia_txd(0xa7);
          }
          else if(DataEventReadDone==1)
          {	
           	  TXDCheck=0xa5^0x02^0x55;   
	 	  
	 	      scia_txd(0xa5);
	 	      scia_txd(0x02); 	 	       	      
	 	      scia_txd(0x55);
	 	      	 	           
	 	      for(i=0;i<EVENTDATALENGTH;i++)
	 	      { 
	 	      	TXDCheck^=DataEvent[i];     	      
	 	      	scia_txd(DataEvent[i]);
	 	      }   
	 	        scia_txd(TXDCheck&0xff);        	  
	      	  TXD_Flag=0;		 	 
	      	  DataEventReadDone=0;
      	  }
      	  break;	             	                             
        }                    
        case 7:
        {
       
        	{        	  	     
		   	    scia_txd(0xa5);
 	            scia_txd(0x07);  
 	            scia_txd(0xa2); 		   	  
			}
			break;			   	   					
        }
            	
        case 8:  		/* 事件记录状态发送*/
        {  	 	     
		     temp_var1=EventStateData&0x0ff;
		     temp_var2=(EventStateData&0x0ff00)>>8;
		     temp_var3=(EventStateData&0x0f0000)>>16;  
		     temp_var4=0x0a5^0x03^temp_var1^temp_var2^temp_var3;  		     

		     scia_txd(0x0a5);
	 	     scia_txd(0x003);  	      
	 	     scia_txd(temp_var1);
	 	     scia_txd(temp_var2);
	 	     scia_txd(temp_var3);
	 	     scia_txd(temp_var4);      
		     TXD_Flag=0; 
		     break;	        	                             
        } 
        
        case 16:  		/* 事件记录擦除完成*/
        { 
            TXD_Flag=0; 
	        scia_txd(0x0a5);
 	        scia_txd(0x004);
 	        scia_txd((0x0a5^0x04)); 
 	        break; 	           	                          	                             
        }   
        
        case 32:  		/*记际录瓿�*/
        {
 	        TXD_Flag=0; 
	        scia_txd(0x0a5);
 	        scia_txd(0x008);
 	        scia_txd((0x0a5^0x08)); 
 	        break;  	          	                          	                             
        }         
        case 64:    /*事件管理器复位完成*/
        {   
        	TXD_Flag=0;
        	scia_txd(0x0a5);
 	        scia_txd(0x0a);
 	        scia_txd((0x0a5^0x0a));  
 	        break;     	
        }
               
        case 128:     /* 设置模式完毕*/
        {
          	TXD_Flag=0; 
	        scia_txd(0x0a5);
 	        scia_txd(0x080);
 	  	
			break;	
        }	       
        default:  
        {
        	 TXD_Flag=0;
        	 break; 	
        } 
    }  /*end switch*/      
}     	       
void StartPwm()
{
   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;
   EPwm1Regs.AQCSFRC.bit.CSFA = 0;
   EPwm1Regs.AQCSFRC.bit.CSFB = 0;
   EPwm2Regs.AQCSFRC.bit.CSFA = 0;
   EPwm2Regs.AQCSFRC.bit.CSFB = 0;
   EPwm3Regs.AQCSFRC.bit.CSFA = 0;
   EPwm3Regs.AQCSFRC.bit.CSFB = 0;
}

void StopPwm()
{
/*********************FORCE LOW**********************/
 
//   EPwm1Regs.AQSFRC.all = 0xc0;
//   EPwm2Regs.AQSFRC.all = 0xc0;
//   EPwm3Regs.AQSFRC.all = 0xc0;

   EPwm1Regs.AQCSFRC.bit.CSFA = 1;
   EPwm1Regs.AQCSFRC.bit.CSFB = 1;
   EPwm2Regs.AQCSFRC.bit.CSFA = 1;
   EPwm2Regs.AQCSFRC.bit.CSFB = 1;
   EPwm3Regs.AQCSFRC.bit.CSFA = 1;
   EPwm3Regs.AQCSFRC.bit.CSFB = 1;
   
}


 void InitPwm(void)
 {
 
	// Setup TBCLK
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_DOWN; // Count up
	EPwm1Regs.TBPRD = 0x3d09;		             // Set timer period 15625  800Hz
	EPwm1Regs.TBCTL.bit.PHSEN =  TB_DISABLE;     // Disable phase loading
	EPwm1Regs.TBPHS.half.TBPHS = 0x0000;	     // Phase is 0
	EPwm1Regs.TBCTR = 0x0000;				     // Clear counter
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0x03;	     // Clock ratio to SYSCLKOUT
	EPwm1Regs.TBCTL.bit.CLKDIV = 0x01;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	// Setup shadow register load on ZERO
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
 
	// Set Compare values
	EPwm1Regs.CMPA.half.CMPA = 0x1f40;//0x3b95;	  // Set compare A value2950 15253
	EPwm1Regs.CMPB = 0x1f40;//0x3b95;			  // Set Compare B value


   EPwm1Regs.AQSFRC.bit.ACTSFA = 0x1;  
   EPwm1Regs.AQSFRC.bit.RLDCSF = 0x3; 
	// Set actions
	EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;			// Set PWM1A on Zero
	EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR;		  // Clear PWM1A on event A, up count
	
 }
	
void enabledog()
    {
		EALLOW;
		SysCtrlRegs.WDCR= 0x2A;
		EDIS;
    }

 /***************MAIN FUNCTION *******************************/
void main(void)
{

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2833x_SysCtrl.c file.
   InitSysCtrl();
   
// Step 2. Initalize GPIO: 
// This example function is found in the DSP2833x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example
 
// For this example use the following configuration:

   InitEPwm1Gpio();
   InitEPwm2Gpio();
   InitEPwm3Gpio();
   InitSciaGpio();
   InitSpiGpio();
   	     
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts 
   DINT;
// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.  
// This function is found in the DSP2833x_PieCtrl.c file.
   InitPieCtrl();
// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;
// Initialize the PIE vector table with pointers to the shell Interrupt 
// Service Routines (ISR).  
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
// This function is found in DSP2833x_PieVect.c.
   InitPieVectTable();
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.TINT0 = &cpu_timer0_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize the Device Peripheral. This function can be
//         found in DSP2833x_CpuTimers.c
   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
   InitFlash();
   InitCpuTimers();   // For this example, only initialize the Cpu Timers
   InitAdc();
   InitSci();
   spi_fifo_init();	  // Initialize the Spi FIFO
   spi_init();		  // init SPI 
   

#if (CPU_FRQ_150MHZ)
// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
// 150MHz CPU Freq, 1 second Period (in uSeconds)
   ConfigCpuTimer(&CpuTimer0, 150, 1000000);
#endif
   Init_timer0();


   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
   EDIS;
   
   InitPwm();
        
   IER |= M_INT1;       //使能CPU中断1,即第一组PIE中断，共12组PIE中断
   IER |= M_INT3;
   //IER |= M_INT13;
   //IER |= M_INT14;

// Enable TINT0 in the PIE: Group 1 interrupt 7
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;     //timer0 interrupt
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;     //ePWM1  interrupt
//   PieCtrlRegs.PIEIER3.bit.INTx2 = 1;   //ePWM2  interrupt
//  PieCtrlRegs.PIEIER3.bit.INTx3 = 1;    //ePWM3  interrupt

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM
   
   ADC_initial();
   ADC_START();
   
   delay(1000);
   
   InitScibGpio();
   Gpio_select();

   StartPwm();
  
  
   GpioDataRegs.GPBDAT.bit.GPIO53=0;	

   start_timer0();
   enabledog();
   for(;;)
       {
           ServiceDog();
           if(CpuTimer0.InterruptCount >= 400)   //定时20ms
               {
                   CpuTimer0.InterruptCount = 0;
                   DataRealFill();
                   RXD_Comm();
                   TXD_Comm();
		
						
                   /*for test step time*/
                   TimerCounterFlag++;
                   if(TimerCounterFlag >= 50)
                       {
                           TimerCounter++;
                           TimerCounterFlag = 0;
                       }
               }
       }
}
