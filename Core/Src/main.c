
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#define Rsense 0.02
#define amplifier 16
volatile uint32_t PgaGain=OPAMP_PGA_GAIN_16_OR_MINUS_15*(amplifier==16)+OPAMP_PGA_GAIN_8_OR_MINUS_7*(amplifier==8)+OPAMP_PGA_GAIN_4_OR_MINUS_3*(amplifier==4)+OPAMP_PGA_GAIN_2_OR_MINUS_1*(amplifier==2);

volatile uint16_t AD[Lad*2];volatile uint16_t bias[Lad*2];volatile uint32_t biast[Lad*2];
volatile uint16_t tx_buf1[tx_buf_size];volatile uint16_t tx_buf[tx_buf_size];volatile uint16_t PWMON=15*clk;
volatile int32_t R[NR*3];volatile int16_t Ibuff[IbuffL];volatile int16_t Icnt=0;volatile int32_t If=0;
volatile int32_t COMMAND[NR];
volatile int32_t BUF_SIZE=sizeof(R)/4;
volatile uint32_t CURR_TEST=0;volatile uint8_t tosend=0;volatile uint32_t lay_i=0;
POSITION_STRUCT P={0};;CURRENT_STRUCT I={0};;MOTOR_STRUCT M={0};;BALAM_STRUCT BALAM={0};COMMUNICATION_STRUCT COMM={0};;BUTTON_STRUCT BUT={0};;PROFILE_STRUCT PRF={0};;
volatile int32_t CAN[LCAN/4];
volatile uint16_t CANi=0;
volatile int32_t rxCAN[LCAN/4+1];
volatile uint16_t PhasePWM[3];
FDCAN_RxHeaderTypeDef rxHeader1;
int16_t CAN_RCV_DATA=0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp4;

DAC_HandleTypeDef hdac1;

FDCAN_HandleTypeDef hfdcan1;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim6_up;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_ADC1_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_COMP4_Init(void);
static void MX_DAC1_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_ADC1_Init();
  MX_OPAMP1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_COMP4_Init();
  MX_DAC1_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  hadc2.Instance->SMPR2|=(1<<24);
  NVIC_DisableIRQ(DMA2_Channel1_IRQn);
  I.PWMAX=(PWMON-dt_delay);I.PWMAX2=I.PWMAX/2;I.PWM=-100;
  I.int_max=I.PWMAX/I.ki; I.max=15000;I.l_hd=sizeof(I.meas_hd)/2;
  I.CCERN=((1<<3)+(1<<7)+(1<<11)+(1<<1)+(1<<5)+(1<<9));
  I.factor=curr_f/Rsense/amplifier;
  M.busv=1000;
  HAL_Delay(1000);  tx_buf[tx_buf_size-1]=0xAA44;  htim1.Instance->ARR=PWMON;
  htim1.Instance->CR2|=1;htim1.Instance->CCR1=0;htim1.Instance->CCR2=0; htim1.Instance->CCR3=0;
      htim1.Instance->CCR4=PWMON-20;
      HAL_TIM_Base_Start(&htim8);HAL_TIM_Base_Start(&htim1);HAL_TIM_Base_Start(&htim2);HAL_TIM_Base_Start(&htim17);
      HAL_TIM_Base_Start(&htim16);HAL_TIM_Base_Start(&htim4);HAL_TIM_Base_Start(&htim7);HAL_TIM_Base_Start(&htim15);
      HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); htim1.Instance->EGR|=(1<<5);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_5);htim1.Instance->CCR5=PWMON-20;
      HAL_OPAMP_Start(&hopamp1);HAL_OPAMP_Start(&hopamp2);HAL_OPAMP_Start(&hopamp3);HAL_Delay(500);
      HAL_ADC_Start_DMA(&hadc1,&AD[0],Lad);HAL_ADC_Start_DMA(&hadc2,&AD[Lad],1); HAL_Delay(100);
      volatile uint32_t LDON=250; htim3.Instance->ARR=LDON;htim3.Instance->CR1=0;
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);for(int i=0;i<20;i++){
      {biast[0]=biast[0]+AD[0];biast[1]=biast[1]+AD[Lad];
      hadc2.Instance->CR|=(1<<4);while((hadc2.Instance->CR&(1<<4))>0){};
      hadc2.Instance->SQR1&=~((0b11111)<<6);hadc2.Instance->SQR1|=(18<<6);  hadc2.Instance->CR|=(1<<2);
      HAL_Delay(1); biast[2]=biast[2]+AD[Lad];
      hadc2.Instance->CR|=(1<<4);while((hadc2.Instance->CR&(1<<4))>0){};
      hadc2.Instance->SQR1&=~((0b11111)<<6); hadc2.Instance->SQR1|=(16<<6);hadc2.Instance->CR|=(1<<2);
      HAL_Delay(1);}}
      for (int j=0;j<(Lad*2);j++){bias[j]=(biast[j])/20;}
      volatile int8_t HP0=0;volatile int8_t HP=0;volatile int8_t HS=0;volatile int8_t HS0=0;
      volatile uint16_t CHECKSUM=0;volatile uint32_t POS32=0;
      volatile float dir_mas=0;volatile float dir_mas0=0;volatile float POS=0;
      HAL_UART_Transmit_DMA(&huart2,&tx_buf,tx_buf_size*2);
      htim16.Instance->CNT=0;
      volatile float POS_ERR=0;
    volatile uint16_t PW[3]={0};volatile uint16_t PWM16=0;
volatile uint8_t CP=0;;volatile uint8_t CP0=0;;


volatile float POS_Kp=5;volatile float POS_Ki=1;volatile float POS_Kd=0;
volatile float tar_curr=0;volatile float curr_tot=0;
volatile float pwm_dir=1;volatile float Ip_max=0;
volatile float VPerr=0;volatile float VIerr=0;
htim1.Instance->CCMR1&=~((1<<3)+(1<<11));
htim1.Instance->CCMR2&=~((1<<3));
volatile float T20=0;volatile float DT20=30;
volatile float DT2=0;volatile float CURMAX=1500;




volatile float MOTION_COMP=0;volatile float IN_WIN=0;

volatile float WIN_T=10;volatile float POS_ERR_LIM=1000;
volatile float TO_HOLD=0;

volatile uint16_t  BIT_COM=0;
volatile uint32_t t0_WIN=0;
volatile uint8_t CLR_TX=0;volatile uint16_t COM_NUM=0;
volatile int32_t CST=0;volatile float v_prof=0;
volatile int32_t dir_prof=0;volatile float dpos=0;
float FDFWD=0;
volatile uint8_t Faulhaber=0;
float vel2=0;volatile float PWMONf=PWMON;
volatile float atemp;
volatile float COMP_CURR=15000;
volatile float COMP_V=COMP_CURR*COMP_Factor*Rsense+COMP_Bias;
HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,COMP_V);
HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,COMP_V);
HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
HAL_COMP_Start(&hcomp1);HAL_COMP_Start(&hcomp2);HAL_COMP_Start(&hcomp4);
int16_t dbg1=0;uint16_t THALL=0;


volatile uint16_t HPV2[11]={0};
volatile uint16_t HPV2L=10;
memset(HPV2,0,(HPV2L+1)*2);
volatile uint16_t HPV2LIM=HPV2L*0.7;
volatile uint8_t HPT[7]={0};volatile uint16_t HPtemp=0;
__HAL_TIM_ENABLE_DMA(&htim6,TIM_DMA_UPDATE);
HAL_DMA_Start(&hdma_tim6_up,&GPIOB->IDR,HPV2,HPV2L);
HAL_TIM_Base_Start(&htim6);
HS0=((GPIOB->IDR)&448)/64;htim15.Instance->CNT=0;
volatile uint32_t tsin0;volatile float sinfreq=0;
volatile float CUR_A=0;volatile float CUR_B=0;
volatile float pot=0;volatile uint8_t button_pressed=0;volatile uint32_t button_time=0;volatile uint8_t pot_mode=0;
//volatile int32_t COM_STR_OPEN_TEMP[]={200,2700,1400,10000,1000,10000,2000,3,5000,1000,100000,200000,100,0,0,54,98177,201};
//volatile int32_t COM_STR_CLOSE_TEMP[]={200,2700,1400,-100000,1000,10000,500,3,5000,1000,100000,200000,100,0,0,54,98177,201};
//volatile int32_t COM_STR_ZERO_TEMP[]={200,2700,1400,0,1000,10000,500,3,5000,1000,100000,200000,100,0,2,54,98177,201};
//volatile int32_t COM_STR_OPEN[]={200,18000,3800,65500,2000,1000,400,10,5000,100,100000,40000,200,0,0,54,98177,201};
//volatile int32_t COM_STR_CLOSE[]={200,18000,3800,-200000,1300,1000,250,10,5000,100,100000,40000,200,0,0,54,98177,201};
//volatile int32_t COM_STR_CLOSE_Z[]={200,18000,3800,0,1300,1000,250,202,100000,100,100000,40000,200,0,0,54,98177,201};
//volatile int32_t COM_STR_ZERO[]={200,18000,3800,0,1300,1000,250,10,5000,300,100000,40000,200,0,2,54,98177,201};
volatile int8_t CS_PASS=0;volatile int8_t POT_ZERO=0;
volatile float vfilt=0;volatile float vfilt2=0;volatile uint8_t vfilt_end=0;
volatile uint32_t dbg2=0;
volatile uint32_t music_t0=0;
uint16_t CAN_FULL=0;
NVIC_EnableIRQ(DMA2_Channel1_IRQn);
HAL_GPIO_WritePin(CAN_TERM_GPIO_Port,CAN_TERM_Pin,1);
int32_t torq24;float torq=0;;float torq0=65536;GPIO_InitTypeDef GPIO7_InitStruct = {0};GPIO7_InitStruct.Pin = 1<<7;
__HAL_UART_CLEAR_FLAG(&huart2,UART_FLAG_ORE|UART_FLAG_NE|UART_FLAG_FE);
memset(R,0,sizeof(R));HAL_UART_Receive_DMA(&huart2,&R[0],sizeof(R));

//volatile int32_t AVIV[]={200,200,200,100000,1,969696,100,0,5000,1000000,4000,0,1000,0,2,54,0,201};
BUT.time0=htim2.Instance->CNT;
BUT.step_time0=htim2.Instance->CNT;

BUT.step=1000;
volatile int32_t AVIV[]={200,200,200,100000,1,777777,100,0,0,200000000,0,0,1000,0,2,54,0,201};
memcpy(COMMAND,AVIV,NR*4);COMM.com_recievd=POT_RCV;

htim3.Instance->CCR1=LDON*0.5*(BUT.value+1);

  /* USER CODE END 2 */



  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

  while (1)

  {

//       BUT.pot=(float)AD[2]*60/4095;

	  BUT.time=htim2.Instance->CNT-BUT.time0;
	  BUT.step_time=htim2.Instance->CNT-BUT.step_time0;
	 M.stand=1;M.curr_mode=PWM3;HP0=0;

       if((HAL_GPIO_ReadPin(BUTTON_GPIO_Port,BUTTON_Pin)==0)&&(BUT.pressed==0)&&(BUT.time>200000)){
    	   BUT.pressed=1;BUT.time0=htim2.Instance->CNT;BUT.value=1-BUT.value;BUT.step=0;BUT.step_time=0;
    	   BUT.step_time0=htim2.Instance->CNT;BUT.rep=0;
    	   htim3.Instance->CCR1=LDON*0.5*(BUT.value+1);
       }

       if (BUT.pressed==1){BUT.pressed=0;}

       if (BUT.value==1){
    	   M.on=1;
    	   if (BUT.step==0){PhasePWM[1]=(float)I.PWMAX*(float)10/(float)12;
    	   PhasePWM[0]=0;
    	   PhasePWM[2]=(float)I.PWMAX*(float)0/(float)12;BUT.step++;
    	   BUT.step_time0=htim2.Instance->CNT;BUT.step_time=0;}
    	   if ((BUT.step_time>5000)&&(BUT.step==1)){PhasePWM[1]=0;BUT.step++;}
    	   if ((BUT.step_time>7500)&&(BUT.step==2)){BUT.rep++;BUT.step_time0=htim2.Instance->CNT;
    	   	   if (BUT.rep<6){BUT.step=0;}
    	   	   else{BUT.step=10;BUT.rep=0;}
    	   }

		   if (BUT.step==10){PhasePWM[1]=(float)I.PWMAX*(float)7/(float)12;BUT.step++;
		   BUT.step_time0=htim2.Instance->CNT;BUT.step_time=0;}
		   if ((BUT.step_time>4000)&&(BUT.step==11)){PhasePWM[1]=0;BUT.step++;}
		   if ((BUT.step_time>6000)&&(BUT.step==12)){BUT.rep++;BUT.step_time0=htim2.Instance->CNT;
			   if (BUT.rep<6){BUT.step=10;}
			   else{BUT.step=1000; M.on=0;}
		   }
       }

       if (BUT.value==0){
    	   M.on=1;

    	   if (BUT.step==0){PhasePWM[0]=(float)I.PWMAX*(float)7/(float)12;
    	   PhasePWM[1]=0;
    	   PhasePWM[2]=(float)I.PWMAX*(float)7/(float)12;BUT.step++;
    	   BUT.step_time0=htim2.Instance->CNT;BUT.step_time=0;}
    	   if ((BUT.step_time>10000)&&(BUT.step==1)){PhasePWM[0]=0;PhasePWM[2]=0;BUT.step=1000;M.on=0;}
       }




//    	   if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port,BUTTON_Pin)==1){
//
//             BUT.pressed=0;if(((htim2.Instance->CNT-BUT.time)>50000)){BUT.mode=(BUT.mode==0);BUT.zeroed=0;
//
//             if(BUT.mode==0){STOP_MOTOR();}}}}
//
////     if(BUT.mode==0){BUT.mode=1;};
//
//       if(BUT.mode>0){
//
//             if((BUT.mode==1)&&(BUT.pot<0.2)){BUT.mode=2;memcpy(COMMAND,AVIV,NR*4);COMM.com_recievd=POT_RCV;}
//
//             if(BUT.mode==2){M.win_cnt=BUT.pot;}



//           if((pot<-0.6)&&(pot_mode!=2)){pot_mode=2;

//           if (POT_ZERO==0){memcpy(R,COM_STR_CLOSE,NR*4);}else{memcpy(R,COM_STR_CLOSE_Z,NR*4);}COMM.com_recievd=1;CS_PASS=1;}

//           if((pot>0.6)&&(pot_mode!=3)&&(POT_ZERO==1)){pot_mode=3;memcpy(R,COM_STR_OPEN,NR*4);COMM.com_recievd=1;CS_PASS=1;}

//           if((pot<0.5)&&(pot>-0.5)&&(pot_mode!=99)){pot_mode=99;R[0]=255;COMM.com_recievd=1;}

//       }

//     if((pot_mode==2)&&(MOTION_COMP==2)&&(POT_ZERO==0)){MOTION_COMP=3;POT_ZERO=1;memcpy(R,COM_STR_ZERO,NR*4);COMM.com_recievd=1;CS_PASS=1;}

       UART_RX_Handler();



       if(COMM.com_recievd>NONE){

     if(COMM.com_recievd==UART_RCV){BUT.mode=0;BUT.zeroed=0;memcpy(CAN,COMMAND,LCAN);CAN[0]=0x40000000;CAN_SND();COMM.is_master=1;}

          if(COMM.com_recievd==CAN_RCV){BUT.mode=0;BUT.zeroed=0;COMM.is_master=0;}

           vfilt_end=0;P.motion_comp=0;P.in_win=0;



           I.kp=(float)(COMMAND[1])/1000*15*170; I.ki=(float)(COMMAND[2])/1000*15*170;

     PRF.tar_pos=COMMAND[3];PRF.tar_vel=COMMAND[4];PRF.tar_acc=COMMAND[5];I.int_max=(float)I.PWMAX/I.ki;

             CURMAX=((float)(COMMAND[6]));M.win_cnt=COMMAND[7];WIN_T=COMMAND[8];POS_ERR_LIM=COMMAND[9];

             POS_Kp=0.001*(float)(COMMAND[10]);POS_Ki=0.001*(float)(COMMAND[11]);  POS_Kd=0.001*(float)(COMMAND[12]);FDFWD=0.001*(float)(COMMAND[13]);

             M.curr_mode=NORMAL;M.stand=0;GPIO7_InitStruct.Mode = GPIO_MODE_INPUT;HAL_GPIO_Init(GPIOB, &GPIO7_InitStruct);

             HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);

             if (PRF.tar_acc==838383){M.stand=1;M.curr_mode=TORQUE;HAL_TIM_IC_Stop(&htim4, TIM_CHANNEL_1);

             GPIO7_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;HAL_GPIO_Init(GPIOB, &GPIO7_InitStruct); HAL_GPIO_WritePin(GPIOB, 1<<7,0);}

             if (PRF.tar_acc==939393){M.curr_mode=PWM;}

             if (PRF.tar_acc==949494){M.stand=1;M.curr_mode=CONST;}

             if (PRF.tar_acc==959595){M.curr_mode=CONST;}

             if (PRF.tar_acc==969696){M.stand=1;M.curr_mode=PWM;HP0=0;}

             if (PRF.tar_acc==979797){M.stand=1;M.curr_mode=SIN;BALAM.freq=M.win_cnt*0.00000001;BALAM.t0=htim2.Instance->CNT;BALAM.a=CURMAX*(1-0.005*WIN_T); BALAM.b=CURMAX*(0.005*WIN_T);}

             if (PRF.tar_acc==989898){M.stand=1;M.curr_mode=RAMP;BALAM.freq=M.win_cnt*0.00000001;BALAM.t0=htim2.Instance->CNT;BALAM.a=CURMAX*(1-0.01*WIN_T); BALAM.b=CURMAX*(0.01*WIN_T);}

             if (PRF.tar_acc==989899){M.stand=1;M.curr_mode=BIASED_RAMP;BALAM.freq=M.win_cnt*0.00000001;BALAM.t0=htim2.Instance->CNT;BALAM.a=CURMAX*(1-0.01*WIN_T);BALAM.b=CURMAX*(0.01*WIN_T);}

             if (PRF.tar_acc==777777){M.stand=1;M.curr_mode=PWM3;HP0=0;

             if ((COMMAND[10]<10001)&&(COMMAND[10]>=0)) {PhasePWM[0]=(float)I.PWMAX*(float)COMMAND[10]/(float)10001;}

             if ((COMMAND[11]<10001)&&(COMMAND[11]>=0)) {PhasePWM[1]=(float)I.PWMAX*(float)COMMAND[11]/(float)10001;}

             if ((COMMAND[12]<10001)&&(COMMAND[12]>=0)) {PhasePWM[2]=(float)I.PWMAX*(float)COMMAND[12]/(float)10001;}}





             M.type=COMMAND[NR-4]&1;TO_HOLD=COMMAND[NR-4]&4;

             if((COMMAND[NR-4]&2)>0){ POS=0;PRF.pos=0;PRF.pos_l=0;PRF.pos_h=0;P.int_l=0;P.int_h=0;}

             M.on=1;COM_NUM=COMMAND[NR-3];Ip_max=CURMAX/POS_Ki;PRF.pos_temp=PRF.pos;

             if(M.type==FAULHABER){HS0=~HS0;HS0=HS0&7;}

           htim1.Instance->BDTR|=(1<<15);}

       COMM.com_recievd=NONE;





if(((htim1.Instance->BDTR)&(1<<15))==0){M.on=0;}



if ((POS_ERR>POS_ERR_LIM)||(POS_ERR<-POS_ERR_LIM)){   M.on=0;P.motion_comp=2;}

if ((((float)POS-PRF.tar_pos)<M.win_cnt)&&(((float)POS-PRF.tar_pos)>-M.win_cnt)){if (P.in_win==1){

     if(htim2.Instance->CNT>(t0_WIN+WIN_T)){P.motion_comp=1;}}

else {t0_WIN=htim2.Instance->CNT;P.in_win=1;}}

else {P.in_win=0;P.motion_comp=0;}

if ((TO_HOLD==0)&&(P.motion_comp==1)){M.on=0;}





for(int i2=0;i2<HPV2L;i2++){HPtemp=(HPV2[i2]&448)/64;HPT[HPtemp]++;

if (HPT[HPtemp]>HPV2LIM){HS=HPtemp;if(M.type==FAULHABER){HS=~HS;HS=HS&7;}break;}}

memset(HPT,0,7);

HP=(HS==4)*1+(HS==5)*2+(HS==1)*3+(HS==3)*4+(HS==2)*5+(HS==6)*6;

if(M.stand==1){HP=6;}

if(HP!=HP0){

if((HP>0)&&(HP<7)){

     THALL=htim15.Instance->CNT;

     htim15.Instance->CNT=0;

     NVIC_DisableIRQ(DMA2_Channel1_IRQn);

     M.polar[0]=1*((HP==2)||(HP==3))+2*((HP==5)||(HP==6));

     M.polar[1]=1*((HP==4)||(HP==5))+2*((HP==1)||(HP==2));

     M.polar[2]=1*((HP==1)||(HP==6))+2*((HP==3)||(HP==4));

I.CCER=((1<<0)+(1<<2)+(1<<4)+(1<<6)+(1<<8)+(1<<10))+(1<<3)*(M.polar[0]==0)+(1<<7)*(M.polar[1]==0)+(1<<11)*(M.polar[2]==0);

     NVIC_EnableIRQ(DMA2_Channel1_IRQn);

     dir_mas=((float)(2*((HP>HP0)-(HP==6)*(HP0==1)+(HP==1)*(HP0==6)))-1);HP0=HP;

     POS=POS+dir_mas;PRF.pos_temp=PRF.pos;

     if(dir_mas!=dir_mas0){vel2=0;}

     else{vel2=100000/THALL*dir_mas;}

     if(vfilt_end==1){vfilt_end=0;}

     vfilt=0.9*vfilt+0.1*vel2;

     dir_mas0=dir_mas;

     HS0=HS;

     while(((htim4.Instance->SR)&TIM_SR_CC1IF_Msk)>0){(htim4.Instance->SR)&=~TIM_SR_CC1IF_Msk;}}}



if(M.curr_mode==TORQUE){if((HS&1)==0){

     uDelay(170);torq24=0;int jt=24;while (jt>-1){jt--;HAL_GPIO_WritePin(GPIOB, 1<<7,1);   uDelay(170);

     if(jt>=0){torq24+=HAL_GPIO_ReadPin(GPIOB, 1<<6)*(1<<jt);}HAL_GPIO_WritePin(GPIOB, 1<<7,0);uDelay(170);}}

     if(torq24>8388608){torq24=torq24-16777216;}torq=(float)(torq24)/8388608*10000;if(torq0==65536){torq0=torq;}torq-=torq0;}





if(((htim4.Instance->CR1)&1)==0){vel2=0;}

if(htim2.Instance->CNT>(T20+DT20)){   DT2=htim2.Instance->CNT-T20;T20=htim2.Instance->CNT;

M.busv=M.busv*0.9+0.1*(8.36995*((AD[4]-bias[4])));if(M.busv<1000){M.busv=1000;}

     if (M.on==1){   dir_prof=2*((PRF.tar_pos-PRF.pos)>0)-1;dpos=(PRF.tar_pos-PRF.pos);

             if(((v_prof*v_prof*0.5/PRF.tar_acc)>((2*(dpos>0)-1)*dpos))&&((v_prof*dir_prof)>0)){dir_prof=-dir_prof;}

             atemp=PRF.tar_acc*DT2*0.00001*dir_prof;

             if((v_prof+atemp)>PRF.tar_vel){if((v_prof-atemp)>PRF.tar_vel){v_prof=v_prof-atemp;}else{v_prof=PRF.tar_vel;}}

             else if((v_prof+atemp)<-PRF.tar_vel){if((v_prof-atemp)<-PRF.tar_vel){v_prof=v_prof-atemp;}else{v_prof=-PRF.tar_vel;}}

             else {v_prof=v_prof+PRF.tar_acc*DT2*0.00001*dir_prof;}

             PRF.pos_l=PRF.pos_l+v_prof*DT2*0.00001;

             while(PRF.pos_l>1){PRF.pos_l=PRF.pos_l-1;PRF.pos_h=PRF.pos_h+1;}

             while(PRF.pos_l<-1){PRF.pos_l=PRF.pos_l+1;PRF.pos_h=PRF.pos_h-1;}

             PRF.pos=PRF.pos_l+PRF.pos_h;

             PRF.check=PRF.pos-PRF.pos_temp;

             if(PRF.check<1){ if(PRF.check>-1){POS_ERR=PRF.pos-POS-PRF.check;}else{POS_ERR=PRF.pos-POS+1;}}else{POS_ERR=PRF.pos-POS-1;}//motion exterpulation inside hall

             P.int_l=P.int_l+POS_ERR*DT2*0.00001;

             while(P.int_l>1){P.int_l=P.int_l-1;P.int_h=P.int_h+1;}

             while(P.int_l<-1){P.int_l=P.int_l+1;P.int_h=P.int_h-1;}

             P.int_t=P.int_l+P.int_h;

             if(P.int_t>Ip_max){P.int_t=Ip_max;P.int_l=0;P.int_h=P.int_t;}

             if(P.int_t<-Ip_max){P.int_t=-Ip_max;P.int_l=0;P.int_h=P.int_t;}

             if(htim15.Instance->CNT>60000){vfilt_end=1;}

//           if(vfilt_end==1){vfilt=0;}

//           vfilt2=vfilt;

//           if(150000/((float)(htim15.Instance->CNT))<(vfilt*dir_mas)){vfilt2=100000/((float)(htim15.Instance->CNT));}

//           vfilt2=vfilt2*((vfilt2>50)||(vfilt2<-50));

             VPerr=v_prof-vfilt;VIerr=VIerr+VPerr*DT2*0.00001;

             tar_curr=POS_Kd*VPerr+POS_Kp*POS_ERR+POS_Ki*P.int_t+FDFWD*v_prof;

             if(M.curr_mode==CONST){tar_curr=CURMAX;}

             if(M.curr_mode==SIN){tar_curr=BALAM.a-BALAM.b*cos(pi2*(htim2.Instance->CNT-BALAM.t0)*BALAM.freq);}

             if(M.curr_mode==RAMP){tar_curr=BALAM.a+BALAM.b*((cos(pi2*(htim2.Instance->CNT-BALAM.t0)*BALAM.freq))<0);}

             if(M.curr_mode==BIASED_RAMP){tar_curr=BALAM.a+BALAM.b*((cos(pi2*(htim2.Instance->CNT-BALAM.t0)*BALAM.freq))>0.8);}

             if (tar_curr>CURMAX){tar_curr=CURMAX;}if (tar_curr<-CURMAX){tar_curr=-CURMAX;}}

     else{ tar_curr=0;P.int_t=0;PRF.tar_pos=POS;POS_ERR=0;PRF.pos=POS;v_prof=0;}

     I.tar=tar_curr;}





if (M.on==0) {P.int_t=0;P.int_l=0;P.int_h=0;PRF.tar_pos=POS;PRF.pos=POS;PRF.pos_h=POS;PRF.pos_l=0;


htim1.Instance->CCR1=0;htim1.Instance->CCR2=0;htim1.Instance->CCR3=0;

htim1.Instance->CCER|=(1<<3)+(1<<7)+(1<<11);htim1.Instance->EGR|=(1<<5);}



           //COMMUNICATION

if((huart2.hdmatx->Instance->CNDTR==0)&&(tosend==1)){tosend=0;

     tx_buf[0]=(htim2.Instance->CNT)&0xFFFF;

     tx_buf[1]=(htim2.Instance->CNT)>>16;

     POS32=(int32_t)POS+0x80000000;

     tx_buf[2]=(POS32)&0xFFFF;

     tx_buf[3]=(POS32)>>16;

     tx_buf[4]=0x7FFF+(int16_t)((bias[0]-AD[0])*I.factor);

     tx_buf[5]=0x7FFF+(int16_t)(0);

     tx_buf[6]=0x7FFF+(int16_t)(0);

     tx_buf[7]=0x7FFF+(int16_t)(0);

     tx_buf[8]=0x7FFF+(int16_t)((float)PhasePWM[0]/(float)I.PWMAX*10000);

     tx_buf[9]=0x7FFF+(int16_t)((float)PhasePWM[1]/(float)I.PWMAX*10000);

     tx_buf[10]=0x7FFF+(int16_t)((float)PhasePWM[2]/(float)I.PWMAX*10000);

     tx_buf[11]=0x7FFF+(int16_t)(0);



     tx_buf[12]=0x7FFF+(int16_t)(M.busv);

     tx_buf[13]=0x7FFF+(int16_t)(torq);

     tx_buf[14]=0x7FFF+(int16_t)((bias[1]-AD[Lad])*I.factor);

     tx_buf[15]=COM_NUM;

     for(int j1=0;j1<I.l_hd;j1++){tx_buf[16+j1]=I.meas_hd[j1];}

     CHECKSUM=0; for (int i=0;i<tx_buf_size-1;i++){CHECKSUM=CHECKSUM+((tx_buf[i])>>8)+((tx_buf[i])&0xFF);}

     tx_buf[tx_buf_size-1]=0xAA00+(CHECKSUM&0xFF);

     huart2.hdmatx->Instance->CCR&=~1;huart2.hdmatx->Instance->CNDTR=tx_buf_size*2;huart2.hdmatx->Instance->CCR|=1;

     if(COMM.is_master==0){if((hfdcan1.Instance->TXFQS&3)==3){CAN[0]=0;CAN[1]=(int32_t)(I.meas);CAN_SND();CAN_FULL=0;}else{CAN_FULL++;}}

     }





while (((hfdcan1.Instance->RXF0S)&15)>0){HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader1, rxCAN);

if(rxCAN[0]==0x40000000){COMM.com_recievd=CAN_RCV;memcpy(COMMAND,rxCAN,LCAN);R[NR-1]=201;R[0]=200;} //R עבר שינוי, צריך לוודא שעדיין תקין

     if(rxCAN[0]==0x40000001){M.on=0;P.int_l=0;P.int_h=0;}

     if(rxCAN[0]==0){CAN_RCV_DATA=rxCAN[1];}}

if(CAN_FULL>1000){CAN_RST();CAN_FULL=0;}



    /* USER CODE END WHILE */



    /* USER CODE BEGIN 3 */

  }}

void uDelay(uint16_t t){htim7.Instance->CNT=0;while(htim7.Instance->CNT<t){}}

void CAN_RST(void){HAL_FDCAN_Stop(&hfdcan1);

while ((hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0);

__HAL_RCC_FDCAN_FORCE_RESET();

__HAL_RCC_FDCAN_RELEASE_RESET();

HAL_FDCAN_Init(&hfdcan1);

HAL_FDCAN_Start(&hfdcan1);

HAL_FDCAN_ActivateNotification(

    &hfdcan1,

    FDCAN_IT_BUS_OFF |

    FDCAN_IT_ERROR_PASSIVE |

    FDCAN_IT_TX_COMPLETE |

    FDCAN_IT_RX_FIFO0_NEW_MESSAGE,

    0

);

}

void DMA2_Channel1_IRQHandler(void){CurrentCtrl();}



void CurrentCtrl(){

     static uint16_t j_curr=0;  static int8_t CP0=0;static uint32_t tim2=0;

     hdma_adc2.DmaBaseAddress->IFCR|=1;

     I.meas=(float)(AD[Lad*(I.CP>0)]-bias[I.CP])*I.factor;

     if(I.PWM<0){I.meas=-I.meas;}

     I.e=(I.tar-I.meas)/M.busv;

     I.int_t=I.int_l+I.e;I.int_t=I.int_t+I.int_h;

     if((I.meas>I.max)||(I.meas<-I.max)){M.on=0;}

     if(I.int_t>I.int_max){I.int_t=I.int_max;I.int_l=0;I.int_h=I.int_t;}

     if(I.int_t<-I.int_max){I.int_t=-I.int_max;I.int_l=0;I.int_h=I.int_t;}

     I.p=I.kp*I.e;I.i=I.ki*I.int_t;I.PWM=(I.p+I.i);

     if(M.curr_mode==PWM){I.PWM=(float)(I.PWMAX)*M.win_cnt/100;}

     if (M.on==0) {I.PWM=0;I.int_t=0;I.int_l=0;I.int_h=0;}

     if (I.PWM>I.PWMAX){I.PWM16=I.PWMAX2;if (I.p>I.PWMAX){I.int_l=0;I.int_h=0;}else{I.int_l=(I.PWMAX-I.p)/I.ki-I.int_h;}}

     else if (I.PWM>=0){I.PWM16=I.PWM*0.5;I.int_l=I.int_l+I.e;}

     if (I.PWM<-I.PWMAX){I.PWM16=I.PWMAX2;if (I.p<-I.PWMAX){I.int_l=0;I.int_h=0;}else{I.int_l=(-I.PWMAX-I.p)/I.ki-I.int_h;}}

     else if (I.PWM<0){I.PWM16=-I.PWM*0.5;I.int_l=I.int_l+I.e;}

I.PW[0]=I.PWMAX2+I.PWM16*((M.polar[0]==1)*(I.PWM>0)+(M.polar[0]==2)*(I.PWM<=0)-(M.polar[0]==2)*(I.PWM>0)-(M.polar[0]==1)*(I.PWM<=0)+(M.polar[0]==0));

I.PW[1]=I.PWMAX2+I.PWM16*((M.polar[1]==1)*(I.PWM>0)+(M.polar[1]==2)*(I.PWM<=0)-(M.polar[1]==2)*(I.PWM>0)-(M.polar[1]==1)*(I.PWM<=1)+(M.polar[1]==0));

I.PW[2]=I.PWMAX2+I.PWM16*((M.polar[2]==1)*(I.PWM>0)+(M.polar[2]==2)*(I.PWM<=0)-(M.polar[2]==2)*(I.PWM>0)-(M.polar[2]==1)*(I.PWM<=2)+(M.polar[2]==0));

     if (M.on>0){

     if (M.curr_mode==PWM3){
    	 htim1.Instance->CCR1=PhasePWM[0];htim1.Instance->CCR2=PhasePWM[1];htim1.Instance->CCR3=PhasePWM[2];

     I.CCER=((1<<0)+(1<<2)+(1<<4)+(1<<6)+(1<<8)+(1<<10));htim1.Instance->EGR|=(1<<5);

     I.CP=1;}

     else{htim1.Instance->CCR1=I.PW[0];htim1.Instance->CCR2=I.PW[1];htim1.Instance->CCR3=I.PW[2];}

     htim1.Instance->CCER&=~I.CCERN;htim1.Instance->CCER|=I.CCER; htim1.Instance->EGR|=(1<<5);}

     I.dir=(I.PWM>=0)-(I.PWM<0);

     I.CP=1*((I.dir==-1)*(M.polar[1]==1)+(I.dir==1)*(M.polar[1]==2))+2*((I.dir==-1)*(M.polar[2]==1)+(I.dir==1)*(M.polar[2]==2));

     if ((I.CP!=CP0)&&(M.curr_mode!=PWM3)){ hadc2.Instance->CR|=(1<<4);while((hadc2.Instance->CR&(1<<4))>0){};

     hadc2.Instance->SQR1&=~((0b11111)<<6); hadc2.Instance->SQR1|=(16<<6)*(I.CP!=2)+(18<<6)*(I.CP==2); hadc2.Instance->CR|=(1<<2); CP0=I.CP;}

     I.dbg=((htim1.Instance->CR1)&16)==16;

//   I.dbg=htim1.Instance->CNT;

     I.dbg=htim2.Instance->CNT-tim2;tim2=htim2.Instance->CNT;

     while(I.int_l>100){I.int_l=I.int_l-100;I.int_h=I.int_h+100;}while(I.int_l<-100){I.int_l=I.int_l+100;I.int_h=I.int_h-100;}

     I.filt=I.filt*0.95+I.meas*0.05;

     I.meas_hd[j_curr]=0x7FFF+(int16_t)(I.meas);

     j_curr++;if(j_curr>=I.l_hd){j_curr=0;tosend=1;}  }


void CAN_SND(void){
	FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = COMM.is_master;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_64;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // לא CAN-FD
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN);}

void UART_RX_Handler(void){
	static int32_t CS=0;static int32_t CS1=0;static int32_t CS2=0; static int32_t CST=0;
	static volatile uint32_t write_ptr=0;static uint32_t read_ptr = 0;
	static uint32_t SR=sizeof(R);
	if(COMM.uart_receiving==0){
		if (__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE)){
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);htim16.Instance->CNT=0;COMM.uart_receiving=1;return;}}
	if (COMM.uart_receiving==1){if(htim16.Instance->CNT>1000){ COMM.uart_receiving=2;}}
	if (COMM.uart_receiving==2){
	__HAL_UART_CLEAR_FLAG(&huart2,UART_FLAG_ORE|UART_FLAG_NE|UART_FLAG_FE);
	write_ptr=(SR-hdma_usart2_rx.Instance->CNDTR)/4;
	while(read_ptr!=write_ptr){
		if((write_ptr-read_ptr+BUF_SIZE)%BUF_SIZE>=NR){
		  if((R[(read_ptr+NR-1)%BUF_SIZE]==201)&&(R[read_ptr]==200)){
				CS=0;for (int x=read_ptr;x<(read_ptr+NR-2);x++){CST=R[x%BUF_SIZE]-2*R[x%BUF_SIZE]*(R[x%BUF_SIZE]<0);CS1=CST/65536;CS2=CST-CS1*65536;CS=CS+CS1+CS2;}
				if(CS==R[(read_ptr+NR-2)%BUF_SIZE]){for (int x=0;x<NR;x++){COMMAND[x]=R[(x+read_ptr)%BUF_SIZE];COMM.com_recievd=UART_RCV;}}}
		  else if(R[read_ptr]==255){STOP_MOTOR();}}
		else if(R[read_ptr]==255){STOP_MOTOR();}
		read_ptr=(read_ptr+1)%BUF_SIZE;}
	COMM.uart_receiving=0;}}

void STOP_MOTOR(void){
	M.on=0;COMM.uart_receiving=2;P.int_l=0;P.int_h=0;CAN[0]=0x40000001;CAN_SND();

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_2;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_1;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP1;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_2;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_1;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH1;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_TIM1_OC5_COMP1;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp2.Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH2;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_TIM1_OC5_COMP2;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}

/**
  * @brief COMP4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP4_Init(void)
{

  /* USER CODE BEGIN COMP4_Init 0 */

  /* USER CODE END COMP4_Init 0 */

  /* USER CODE BEGIN COMP4_Init 1 */

  /* USER CODE END COMP4_Init 1 */
  hcomp4.Instance = COMP4;
  hcomp4.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp4.Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH1;
  hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp4.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp4.Init.BlankingSrce = COMP_BLANKINGSRC_TIM1_OC5_COMP4;
  hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP4_Init 2 */

  /* USER CODE END COMP4_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */
  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 14;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  // פילטר לכל ההודעות (קלאסי)
  FDCAN_FilterTypeDef sFilter;
  sFilter.IdType = FDCAN_STANDARD_ID;
  sFilter.FilterIndex = 0;
  sFilter.FilterType = FDCAN_FILTER_RANGE;;
  sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilter.FilterID1 = 0x000;
  sFilter.FilterID2 = 0x300;

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0);

  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter);
  HAL_FDCAN_Start(&hfdcan1);
//  HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InternalOutput = ENABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */
  hopamp1.Init.PgaGain = PgaGain;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InternalOutput = ENABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */
  hopamp2.Init.PgaGain = PgaGain;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp3.Init.Mode = OPAMP_PGA_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.InternalOutput = ENABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */
  hopamp3.Init.PgaGain = PgaGain;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END OPAMP3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP1;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP2;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP4;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_6) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 60;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 7;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 2;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1699;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 65535;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_ConfigTI1Input(&htim4, TIM_TI1SELECTION_XORCOMBINATION) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 169;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 169;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 1699;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1699;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 65535;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CAN_TERM_Pin|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BEMF_GPIO_Port, BEMF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CAN_TERM_Pin PC11 */
  GPIO_InitStruct.Pin = CAN_TERM_Pin|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BEMF_Pin */
  GPIO_InitStruct.Pin = BEMF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BEMF_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

