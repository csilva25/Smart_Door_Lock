
#include "stm32F30x.h"
#include "STM32f3_discovery.h"
#include "stm32f30x_gpio.h"
#include  "stm32f30x_i2c.h"
#include "stm32f3_discovery_lsm303dlhc.h"
#include "stm32f3_discovery_l3gd20.h"
#include "main.h"
#include  "stdio.h"
#include  "string.h"
#include  "math.h"
#include <stdbool.h>



#define ABS(x)         (x < 0) ? (-x) : x

#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
#define PI                         (float)     3.14159265f

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */
#define		SYSTICK_MASK	0x8000
#define		TIM3_MASK		0x0200

#define MOTOR_A_PWM_PIN GPIO_Pin_12             //D12

#define PULSE_MIN       1910
#define PULSE_MAX       3890
#define DELAY       200 
/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;


/* Private define ------------------------------------------------------------*/
#define FLASH_PAGE_SIZE         ((uint32_t)0x00000800)   /* FLASH Page Size */
#define FLASH_USER_START_ADDR   ((uint32_t)0x08006000)   /* Start @ of user Flash area */

#define USE_FLASH
#undef DEBUG

int    	Flash_write(uint32_t addr, uint32_t data);


RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef RTC_TimeStructure;
RTC_InitTypeDef RTC_InitStructure;
RTC_AlarmTypeDef RTC_AlarmStructure;
NVIC_InitTypeDef NVIC_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
__IO uint32_t AsynchPrediv = 0, SynchPrediv = 0;
__IO uint8_t showtime[50] = {0};
__IO uint8_t showdate[50] = {0};
/* Private function prototypes -----------------------------------------------*/


RCC_ClocksTypeDef RCC_Clocks;
GPIO_InitTypeDef        GPIO_InitStructure;

volatile uint8_t DataReady = 0;
float       Heading, Xangle, Yangle;
int        	pulse_width, i;

void    RTC_Setup(void);
void Demo_GyroConfig(void);
void Demo_GyroReadAngRate (float* pfData);
void Demo_CompassConfig(void);
void Demo_CompassReadMag (float* pfData);
void Demo_CompassReadAcc(float* pfData);
void Read_Compass(float *, float *, float *);

void    Init_Keypad(void);
void  LCD_cursor(void);
void  LCD_setpos(int row, int col);

void    IO_Init(void);
void    I2C2_init(void);

void 	Timer4_Init(void);

void 	InitPwmGpio(void);
int    	InitPwmSignal(int);

unsigned int	TIM3_tick_count = 0;

unsigned int 	tick_count = 0;
volatile int 	ButtonPressed = 0, KeyPressed = 0, alert=0;
volatile bool unlock=false;
void  LCD_write(int,int, char);
void  LCD_clear(void);
void  LCD_contrast(int);                // Contrast level = 1..50
void  LCD_backlight(int);

void  config_password(void);
void  lock_unlock(int position);
void  validate(void);
void  confirm_password(void);
void  password_check(void);
void  config_time(void);
void  time(void);
void  hour(void);
void  min(void);
void  sec(void);
void  key_pad(void);
void  change_to_int(void);
void  change_to_int2(void);
void	SetAlarm(RTC_AlarmTypeDef);
void Delay(uint32_t nTime);
void alarm_time(int);
void choose(void);
static __IO uint32_t TimingDelay;


char        message[16];
char        flash_err[] = "Flash Write Error";
int l=0,col=0,r=0,c=0,tries=0,row=0,hours=12, minutes=0,seconds=0,position,password=0,confirm=0,temp=0,count=0;
char array_keypad[4][3]  = {{ '1', '2', '3'},{'4', '5', '6'},{'7', '8', '9'},{'*', '0', '#'}};
bool check,match=false,validation = false,confirm_pw = false;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//		Main
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main(void) {



	IO_Init();
	InitPwmGpio();
	Demo_CompassConfig();
	InitPwmSignal(100);
	RTC_Setup();
	I2C2_init();

	
	
	
	while(I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET);
	LCD_contrast(45);
	LCD_backlight(0);
	LCD_clear();
	LCD_cursor();

	while (1)
	{
	time();
	
	if(ButtonPressed)
	{	
	LCD_clear();
	choose();
	LCD_clear();
	ButtonPressed = 0;	
	}
	if(KeyPressed)
	{
	LCD_clear();
	validate();
	LCD_clear();
	KeyPressed = 0;
	}

}
}
void choose(void)
{
			l=1;
		 sprintf(message," (Config. Time) " );
						for (i=0; i < strlen(message); i++)
						LCD_write(0, i, message[i]);
		 sprintf(message,"  Config. Code        " );
						for (i=0; i < strlen(message); i++)
						LCD_write(1, i, message[i]);
	Delay(150);
	alarm_time(5);
		while(!alert){
		for (row=0; row<4;row++){
					GPIOC -> ODR = (0x0100 << row);
					Delay(10);
						if (row==0 &&(GPIOB->IDR & 0x1000 << 3 )  )
						{
							sprintf(message," (Config. Time) " );
						for (i=0; i < strlen(message); i++)
						LCD_write(0, i, message[i]);
							sprintf(message,"  Config. Code   " );
						for (i=0; i < strlen(message); i++)
						LCD_write(1, i, message[i]);
							l=1;
						}
						if (row==1 &&(GPIOB->IDR & 0x1000 << 3 ))
						{
							sprintf(message,"  Config. Time " );
						for (i=0; i < strlen(message); i++)
						LCD_write(0, i, message[i]);
										sprintf(message," (Config. Code)  " );
						for (i=0; i < strlen(message); i++)
						LCD_write(1, i, message[i]);
							l=2;
						}
						
						if (row==3 && (GPIOB->IDR & 0x1000 << 3 ) )
						{
							if (l==1)
							config_time();
							if(l==2)
							config_password();
							return;
						}
					}
				}
		alert=0;
			}
		
	
void  key_pad(void)
{
	tries=2;
	wrong:
    confirm =0,	r=0;
	while (alert!=2 && match == false){
	LCD_setpos(1,r);
	for (row=0; row<4;row++)
		{
			GPIOC -> ODR = (0x0100 << row);
			Delay(10);
				for (col=0; col<3;col++)
				{
						if ((GPIOB->IDR & 0x1000 << col ) != 0)
						{
							Delay(100);
						(validation == true && confirm_pw == false) ? LCD_write(1,r,'*'): LCD_write(1,r,array_keypad [row][col]);
						temp = (array_keypad [row][col] - '0');
						(validation == true) ? (confirm =(confirm * 10) + temp) : (password =(password * 10) + temp);
							r++;
						}
				}
						if ( ( row==3 &&  (GPIOB->IDR & 0x1000 << 3 ) != 0) && r>3 && r<9 )
				{
					Delay(100);
					if (check == true)
						password_check();
					if(validation == true &&tries ==0)
						{
							LCD_clear();
						 sprintf(message,"Locked" );
						for (i=0; i < strlen(message); i++)
						LCD_write(0, i, message[i]);
							RTC_Setup();
							alarm_time(5);
							while (!alert)
							GPIOE->ODR ^= 0xFFFF;
							alert=0;
							return;
						}

					if ( match == false && check == true)
					 {
						tries--;
						 RTC_Setup();
						 alarm_time(6);
							LCD_clear();
						 sprintf(message,"Incorrect" );
						for (i=0; i < strlen(message); i++)
						LCD_write(0, i, message[i]);
						 Delay(300);
						 if (	validation == false && check == false){
						 sprintf(message,"Try Again %dtries", tries+1 );
						for (i=0; i < strlen(message); i++)
						LCD_write(0, i, message[i]); }
						 else {
							 sprintf(message,"Try Again %dtries", tries+1 );
						for (i=0; i < strlen(message); i++)
						LCD_write(0, i, message[i]);}
						 
							r=0;
							goto wrong;
					 }
					 if (match == true && check == true)
					{
							LCD_clear();
						sprintf(message,"Correct" );
						for (i=0; i < strlen(message); i++)
						LCD_write(0, i, message[i]);
						Flash_write(FLASH_USER_START_ADDR, password);
						r=0;
					}
					if (validation == false && check == false)
					confirm_password();
				}
					}
					}
			return;
				}



			

void  time(void)
{
	check=false,match=false,validation = false,confirm_pw = false;
	LCD_clear();
	while(!ButtonPressed){
			/* Get the RTC current Time */
		RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
		/* Display time Format : hh:mm:ss */
		sprintf((char*)showtime,"%0.2d:%0.2d:%0.2d",RTC_TimeStructure.RTC_Hours, RTC_TimeStructure.RTC_Minutes, RTC_TimeStructure.RTC_Seconds);


		for (i=0; i < strlen((char *)showtime); i++)
			  LCD_write(0, i+4, showtime[i]);
		
	for (row=0; row<4;row++)
		{
			GPIOC -> ODR = (0x0100 << row);
			Delay(10);
				for (col=0; col<4;col++)
				{
						if ((GPIOB->IDR & 0x1000 << col ) != 0)
						{
							KeyPressed = 1;
							Delay(100);
							return;
						}
					}
		}
}
	}

void  lock_unlock(int position)
{
	if (position < -50)
	{
		sprintf(message,"Door is locking" );
	for (i=0; i < strlen(message); i++)
			  LCD_write(0, i, message[i]);
	}
		if (position > 60)
	{
	sprintf(message,"Door unlocking" );
	for (i=0; i < strlen(message); i++)
			  LCD_write(0, i, message[i]);
		unlock=true;
		RTC_Setup();
		alarm_time(2);
	}
	position = 2900+(position * 22);
	pulse_width = position;
	TIM_SetCompare1(TIM4, pulse_width);
	//Delay(2000);
	return;
}

void validate(void)
{
	LCD_clear();
	sprintf(message, "Enter access code:");
	for (i=0; i < strlen(message); i++)
			  LCD_write(0, i, message[i]);
	validation = true;
	check = true;
	match = false;
	confirm_pw = false;
	confirm = 0;
	password = *((int*)FLASH_USER_START_ADDR);
	key_pad();
	if (match==true)
			lock_unlock(100);
	match = false;
}


void config_time(void)
{
	hours=0,minutes=0,seconds=0;
	sprintf (message,"Enter Time");
	for (i=0; i < strlen(message); i++)
		LCD_write(0, i, message[i]);
	sprintf (message,"HH:MM:SS");
	for (i=0; i < strlen(message); i++)
		LCD_write(1, i, message[i]); 
	count=0;
	while (count<9){
		LCD_setpos(1,count);
	for (row=0; row<4;row++)
		{
			GPIOC -> ODR = (0x0100 << row);
			Delay(10);
				for (col=0; col<3;col++)
				{
						if ((GPIOB->IDR & 0x1000 << col ) != 0)
						{
							Delay(150);
							LCD_write(1,count,array_keypad [row][col]);
							temp = (array_keypad [row][col] - '0');
							confirm =((confirm * 10) + temp);
							count++;
							
						if (count==2){
							
							count++;
							confirm=confirm%12;
							if (confirm == 0)
								hours = 12;
							else
								hours = confirm;
							confirm=0;
						}
						if (count==5){
							count++;
							confirm=confirm%60;
							if (confirm == 0)
								minutes = 60;
							else
								minutes = confirm;
							confirm=0;
						}
						if (count==8){
							count++;
							confirm=confirm%60;
							if (confirm == 0)
								seconds = 60;
							else
								seconds = confirm;
							confirm=0;
							RTC_Setup();
							   
							   
						}
					}
				}
			}
		}
	}


void  config_password(void)
{
password=0;
	LCD_clear();
sprintf(message, "New Access Code?");
	for (i=0; i < strlen(message); i++)
			  LCD_write(0, i, message[i]);
	sprintf(message, "Press D to Cont.");
	for (i=0; i < strlen(message); i++)
			  LCD_write(1, i, message[i]);
	validation = false;
	check = false;
	RTC_Setup();
	alarm_time(5);
	while(!alert){
		GPIOC->ODR = (0x0100 << 3);
		Delay(10);
			if ((GPIOB->IDR & (0x1000 << 3)) !=0)
			{
			  RTC_ClearITPendingBit(RTC_IT_ALRA);
				EXTI_ClearITPendingBit(EXTI_Line17);
				
				LCD_clear();
				sprintf(message, "Enter a code");
				for (i=0; i < strlen(message); i++)
			  LCD_write(0, i, message[i]);
				Delay(120);
				key_pad();
				break;
			}
	}
	alert =0;
	match = false;
	return;
}
void  confirm_password(void){
	LCD_clear();
	sprintf(message, "confirm access code:");
	for (i=0; i < strlen(message); i++)
			  LCD_write(0, i, message[i]);
	validation = true;
	check = true;
	confirm_pw =true;
	key_pad();
	lock_unlock(-80);
	return;
			}
void password_check(void)
	{
	 match = true;
		if (match == (true))
		{
			for ( i = 0; i<7 ; i++)
			{
					if (password != confirm)
						match =false;
			}
		}
		}
void RTC_Alarm_IRQHandler(void)
{
  if(RTC_GetITStatus(RTC_IT_ALRA) != RESET)
  {
		unlock=false; 
		alert=1;
		GPIOE->ODR ^= 0x0200;
		if (unlock == true)
				TIM_SetCompare1(TIM4, 1140);
		if (tries<2)
			alert =2;
    RTC_ClearITPendingBit(RTC_IT_ALRA);
    EXTI_ClearITPendingBit(EXTI_Line17);
		
  } 
return;
}
		
void alarm_time(int time)
{
	RTC_AlarmTypeDef Alarm;
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
	Alarm.RTC_AlarmTime.RTC_Hours = RTC_TimeStructure.RTC_Hours;
	Alarm.RTC_AlarmTime.RTC_Minutes = RTC_TimeStructure.RTC_Minutes;
	Alarm.RTC_AlarmTime.RTC_Seconds = ((RTC_TimeStructure.RTC_Seconds+time)%60);

	SetAlarm(Alarm);
}
//void check12(float* arr)
void SetAlarm(RTC_AlarmTypeDef Alarm_Time) {
	
  /* Enable PWR APB1 Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  
  /* Allow access to Backup */
  PWR_BackupAccessCmd(ENABLE);
   
  
  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
  
  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);
  
  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();

   
  /* Set the alarm  */
  Alarm_Time.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
   Alarm_Time.RTC_AlarmDateWeekDay = 0x31;
  Alarm_Time.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
  // Alarm mask hour, min and second
  Alarm_Time.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay; 
  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &Alarm_Time);
    
  /* Enable RTC Alarm A Interrupt */
  RTC_ITConfig(RTC_IT_ALRA, ENABLE);
  
  /* Enable the alarm */
  RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
  
  //RTC_ClearFlag(RTC_FLAG_ALRAF);
     
}




void RTC_Setup() {
  /* Enable PWR APB1 Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  
  /* Allow access to Backup */
  PWR_BackupAccessCmd(ENABLE);
  
  /* Reset RTC Domain */
  RCC_BackupResetCmd(ENABLE);
  RCC_BackupResetCmd(DISABLE);
  
  /* Allow access to RTC */
  PWR_BackupAccessCmd(ENABLE);
  
  /* The RTC Clock may varies due to LSI frequency dispersion */   
  /* Enable the LSI OSC */ 
  RCC_LSICmd(ENABLE);
  
  /* Wait till LSI is ready */  
  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {
  }
  
  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
  
  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);
  
  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();

  /* RTC prescaler configuration */
  RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
  RTC_InitStructure.RTC_AsynchPrediv = 88;
  RTC_InitStructure.RTC_SynchPrediv = 470;
  RTC_Init(&RTC_InitStructure);
  
 
  /* Set the date: Wednesday August 15th 2012 */
  RTC_DateStructure.RTC_Year = 12;
  RTC_DateStructure.RTC_Month = RTC_Month_August;
  RTC_DateStructure.RTC_Date = 15;
  RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Wednesday;
  RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
  
  /* Set the time to 02h 16mn 30s AM */
  RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
  RTC_TimeStructure.RTC_Hours   = hours;
  RTC_TimeStructure.RTC_Minutes = minutes;
  RTC_TimeStructure.RTC_Seconds = seconds;
  RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);    
  
  RTC_ClearFlag(RTC_FLAG_ALRAF);
  
  /* RTC Alarm A Interrupt Configuration */
  /* EXTI configuration */
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable the RTC Alarm Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
   
}


//In stm32f30x_it.c file:

/**
  * @brief  This function handles RTC Alarm interrupt request.
  * @param  None
  * @retval None
  */


void Read_Compass (float *Hptr, float *Xptr, float *Yptr) {
  int   i;
 __IO float HeadingValue = 0.0f;
float MagBuffer[3] = {0.0f}, AccBuffer[3] = {0.0f};

float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f, RollAng = 0.0f, PitchAng = 0.0f;
float fTiltedX,fTiltedY = 0.0f;




 /* Read Compass data */
      Demo_CompassReadMag(MagBuffer);
      Demo_CompassReadAcc(AccBuffer);

      for(i=0;i<3;i++)
        AccBuffer[i] /= 100.0f;

      fNormAcc = sqrt((AccBuffer[0]*AccBuffer[0])+(AccBuffer[1]*AccBuffer[1])+(AccBuffer[2]*AccBuffer[2]));

      fSinRoll = -AccBuffer[1]/fNormAcc;
			fCosRoll =  sqrt(1.0f-(fSinRoll * fSinRoll));
      fSinPitch = AccBuffer[0]/fNormAcc;
      fCosPitch = sqrt(1.0f-(fSinPitch * fSinPitch));

      RollAng = ((atan2(fSinRoll, fCosRoll))*180/PI);
      PitchAng = ((atan2(fSinPitch, fCosPitch))*180/PI);

      fTiltedX = MagBuffer[0]*fCosPitch+MagBuffer[2]*fSinPitch;
      fTiltedY = MagBuffer[0]*fSinRoll*fSinPitch+MagBuffer[1]*fCosRoll-MagBuffer[1]*fSinRoll*fCosPitch;

      HeadingValue = (float) ((atan2f((float)fTiltedY,(float)fTiltedX))*180)/PI;

      if (HeadingValue < 0)
      {
        HeadingValue = HeadingValue + 360;
      }


      *Hptr = HeadingValue;
      *Xptr = PitchAng;
      *Yptr = RollAng;
}

void Demo_GyroConfig(void)
{
  L3GD20_InitTypeDef L3GD20_InitStructure;
  L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;

  /* Configure Mems L3GD20 */
  L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
  L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
  L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
  L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
  L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
  L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
  L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500;
  L3GD20_Init(&L3GD20_InitStructure);

  L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
  L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
  L3GD20_FilterConfig(&L3GD20_FilterStructure) ;

  L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}

/**
  * @brief  Calculate the angular Data rate Gyroscope.
  * @param  pfData : Data out pointer
  * @retval None
  */
void Demo_GyroReadAngRate (float* pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;

  L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);

  L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);

  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & 0x40))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }

  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & 0x30)
  {
  case 0x00:
    sensitivity=L3G_Sensitivity_250dps;
    break;

  case 0x10:
    sensitivity=L3G_Sensitivity_500dps;
    break;

  case 0x20:
    sensitivity=L3G_Sensitivity_2000dps;
    break;
  }
  /* divide by sensitivity */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)RawData[i]/sensitivity;
  }
}

/**
  * @brief  Configure the Mems to compass application.
  * @param  None
  * @retval None
  */
void Demo_CompassConfig(void)
{
  LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;
  LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
  LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;

  /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
  LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
  LSM303DLHC_InitStructure.MagOutput_DataRate =LSM303DLHC_ODR_30_HZ ;
  LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_8_1_GA;
  LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
  LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);

   /* Fill the accelerometer structure */
  LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
  LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
  LSM303DLHCAcc_InitStructure.Axes_Enable= LSM303DLHC_AXES_ENABLE;
  LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
  LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
  LSM303DLHCAcc_InitStructure.Endianness=LSM303DLHC_BLE_LSB;
  LSM303DLHCAcc_InitStructure.High_Resolution=LSM303DLHC_HR_ENABLE;
  /* Configure the accelerometer main parameters */
  LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);

  /* Fill the accelerometer LPF structure */
  LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

  /* Configure the accelerometer LPF main parameters */
  LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}

/**
* @brief Read LSM303DLHC output register, and calculate the acceleration ACC=(1/SENSITIVITY)* (out_h*256+out_l)/16 (12 bit rappresentation)
* @param pnData: pointer to float buffer where to store data
* @retval None
*/
void Demo_CompassReadAcc(float* pfData)
{
  int16_t pnRawData[3];
  uint8_t ctrlx[2];
  uint8_t buffer[6], cDivider;
  uint8_t i = 0;
  float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;

  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);

  if(ctrlx[1]&0x40)
    cDivider=64;
  else
    cDivider=16;

  /* check in the control register4 the data alignment*/
  if(!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) /* Little Endian Mode or FIFO mode */
  {
    for(i=0; i<3; i++)
    {
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i])/cDivider;
    }
  }
  else /* Big Endian Mode */
  {
    for(i=0; i<3; i++)
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])/cDivider;
  }
  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);


  if(ctrlx[1]&0x40)
  {
    /* FIFO mode */
    LSM_Acc_Sensitivity = 0.25;
  }

  else
  {
    /* normal mode */
    /* switch the sensitivity value set in the CRTL4*/
    switch(ctrlx[0] & 0x30)
    {
    case LSM303DLHC_FULLSCALE_2G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
      break;
    case LSM303DLHC_FULLSCALE_4G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
      break;
    case LSM303DLHC_FULLSCALE_8G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
      break;
    case LSM303DLHC_FULLSCALE_16G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
      break;
    }
  }

  /* Obtain the mg value for the three axis */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)pnRawData[i]/LSM_Acc_Sensitivity;
  }

}

/**
  * @brief  calculate the magnetic field Magn.
* @param  pfData: pointer to the data out
  * @retval None
  */
void Demo_CompassReadMag (float* pfData)
{
  static uint8_t buffer[6] = {0};
  uint8_t CTRLB = 0;
  uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
  uint8_t i =0;
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB, 1);

  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, buffer+1, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, buffer+2, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, buffer+3, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, buffer+4, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, buffer+5, 1);
  /* Switch the sensitivity set in the CRTLB*/
  switch(CTRLB & 0xE0)
  {
  case LSM303DLHC_FS_1_3_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
    break;
  case LSM303DLHC_FS_1_9_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
    break;
  case LSM303DLHC_FS_2_5_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
    break;
  case LSM303DLHC_FS_4_0_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
    break;
  case LSM303DLHC_FS_4_7_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
    break;
  case LSM303DLHC_FS_5_6_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
    break;
  case LSM303DLHC_FS_8_1_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
    break;
  }

  for(i=0; i<2; i++)
  {
    pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])*1000)/Magn_Sensitivity_XY;
  }
  pfData[2]=(float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/Magn_Sensitivity_Z;

}





/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LSM303DLHC_TIMEOUT_UserCallback(void)
{
  return 0;
}

/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
  return 0;
}


void InitPwmGpio()
{

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = MOTOR_A_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; /* Use the alternative pin functions */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /* GPIO speed - has nothing to do with the timer timing */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /* Push-pull */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; /* Setup pull-down resistors */
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Connect the timer output to the LED pins

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_2); /* TIM4_CH1 -> MOTOR_A_PWM */
}


int InitPwmSignal(int pwm_freq)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    double ms_pulses;

    /* Calculates the timing. This is common for all channels */
    int clk = 72e6; 		// 72MHz -> system core clock. Default on the stm32f3 discovery
    //int clk = 36e6; 		// (APB1 max) 36MHz. Default on the stm32f3 discovery

    int tim_freq = 2e6; 	// in Hz (2MHz) Base frequency of the pwm timer

    int prescaler = ( (clk / tim_freq) - 1 );

    // Calculate the period for a given pwm frequency
    int pwm_period = tim_freq / pwm_freq;       // 2MHz / 50Hz = 40000


    // Calculate a number of pulses per millisecond.
    ms_pulses = (float)pwm_period / ( 1000.0 / pwm_freq ); // for 50Hz we get: 40000 / (1/50 * 1000)


    //  Enable the TIM4 peripheral
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE );

    // Setup the timing and configure the TIM4 timer
    TIM_TimeBaseStructInit(& TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_Period = pwm_period - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);


    // Initialize the OC for PWM
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = (int)(ms_pulses*10); 	// preset pulse width 1 ms
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 	// Pulse polarity

    // Setup channels
    // Channel 1 (PD12)
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // Channel 2  (PD13)
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // Start the timer
    TIM_Cmd(TIM4 , ENABLE);

    return pwm_period;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void IO_Init() {

    EXTI_InitTypeDef   EXTI_InitStructure;
		GPIO_InitTypeDef   GPIO_InitStructure;
		NVIC_InitTypeDef   NVIC_InitStructure;


		/* SysTick end of count event each 1ms */
		RCC_GetClocksFreq(&RCC_Clocks);
		SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);


		/* GPIOE Periph clock enable */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

		/* Configure PE14 and PE15 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12
			| GPIO_Pin_11| GPIO_Pin_10| GPIO_Pin_9| GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             //GPIO_PuPd_NOPULL
		GPIO_Init(GPIOE, &GPIO_InitStructure);

				RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

		/* Configure PE14 and PE15 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;             //GPIO_PuPd_NOPULL
		GPIO_Init(GPIOB, &GPIO_InitStructure);

				RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

		/* Configure PE14 and PE15 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             //GPIO_PuPd_NOPULL
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		/* Enable GPIOA clock */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

		/* Configure PA0 pin as input floating */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Enable SYSCFG clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

		/* Connect EXTI0 Line to PA0 pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);


		/* Configure EXTI0 line */
		EXTI_InitStructure.EXTI_Line = EXTI_Line0;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		/* Enable and set EXTI0 Interrupt to the lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);



}



void I2C2_init(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        I2C_InitTypeDef  I2C_InitStructure;

        RCC_I2CCLKConfig(RCC_I2C2CLK_SYSCLK);

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);



        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_4);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_4);
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
        GPIO_Init(GPIOA, &GPIO_InitStructure);



        I2C_DeInit(I2C2);
        I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
        I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
        I2C_InitStructure.I2C_DigitalFilter = 0x00;
        I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
        I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
        I2C_InitStructure.I2C_Timing = 0xC062121F;

        I2C_Init(I2C2, &I2C_InitStructure);
        I2C_Cmd(I2C2, ENABLE);
}


void  LCD_write(int row, int col, char data) {


        // Move to sepcified row, col

        //while(I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET);

        I2C_TransferHandling(I2C2, 0x50 , 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

        I2C_SendData(I2C2, 0xFE);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

        I2C_SendData(I2C2, 0x45);


        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);
        if (!row)               // if row == 0
            I2C_SendData(I2C2, col);
         else                  // else row asumed to be 1
            I2C_SendData(I2C2, (0x40 + col));



        I2C_TransferHandling(I2C2, 0x50 , 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);


        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);
        I2C_SendData(I2C2, data);

}


//
//      Set LCD Contrast - Level should be 1..50 (Seems to work best if > 35)
//

void  LCD_contrast(int level) {

        //while(I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET);

        I2C_TransferHandling(I2C2, 0x50 , 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

        I2C_SendData(I2C2, 0xFE);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

        I2C_SendData(I2C2, 0x52);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

        I2C_SendData(I2C2, level);

        Delay(20);
}

//
//      Set LCD Backlight - Level should be 1..8 (Seems to work best if > 1)
//

void  LCD_backlight(int level) {

        //while(I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET);

        I2C_TransferHandling(I2C2, 0x50 , 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

        I2C_SendData(I2C2, 0xFE);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

        I2C_SendData(I2C2, 0x53);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

        I2C_SendData(I2C2, level);

        Delay(20);
}


void  LCD_clear() {

        //while(I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET);

        I2C_TransferHandling(I2C2, 0x50 , 2, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

        I2C_SendData(I2C2, 0xFE);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

        I2C_SendData(I2C2, 0x51);

        Delay(20);


}
void  LCD_setpos(int row, int col) {

        // Move to sepcified row, col

        //while(I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET);

        I2C_TransferHandling(I2C2, 0x50 , 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

        I2C_SendData(I2C2, 0xFE);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

        I2C_SendData(I2C2, 0x45);


        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);
        if (!row)               // if row == 0
            I2C_SendData(I2C2, col);
         else                  // else row asumed to be 1
            I2C_SendData(I2C2, (0x40 + col));


}

//
//      Turn on LCD Cursor - Makes cursor visible
//
void  LCD_cursor(void) {

        //while(I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET);

        I2C_TransferHandling(I2C2, 0x50 , 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

        I2C_SendData(I2C2, 0xFE);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

        I2C_SendData(I2C2, 0x47);  	// underline   (Blinking = 0x4B)

        Delay(20);
}





/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */


void Delay(uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }

}


void SysTick_Handler(void)
{
     TimingDelay_Decrement();

     tick_count++;

if (!(tick_count % 500))
		GPIOE->ODR ^= SYSTICK_MASK;
}


void EXTI0_IRQHandler(void)
{
  if (EXTI_GetITStatus(USER_BUTTON_EXTI_LINE) == SET)
  {

    ButtonPressed = 1;

    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
  }
}
#ifdef USE_FLASH

//
//	Flash Write Function
//
int Flash_write(uint32_t addr, uint32_t data) {
	int MemoryProgramStatus;


	// Unlock the Flash to enable the flash control register access
	FLASH_Unlock();

	// Clear pending flags (if any)
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);


	// Erase page containing user data
	if (FLASH_ErasePage(FLASH_USER_START_ADDR)!= FLASH_COMPLETE){
	 /* Error occurred while sector erase.
		 User can add here some code to deal with this error  */
	  while (1) { }
	}


	// Program the user Flash area with data word
	if (FLASH_ProgramWord(addr, data) != FLASH_COMPLETE)
	{

	  /* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
	  while (1) { }
	}


	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock();


	/* Check if the programmed data is OK
	  MemoryProgramStatus = 0: data programmed correctly
	  MemoryProgramStatus != 0: number of words not programmed correctly ******/
	MemoryProgramStatus = PASSED;

	if (data != *(__IO uint32_t *)FLASH_USER_START_ADDR) {
	  MemoryProgramStatus = FAILED;
	}

#ifdef DEBUG
	sprintf(message, "%x", data);
	for (i=0; i < strlen(message); i++)
			  LCD_write(0, i, message[i]);				// Display number on 1st line of display

	ButtonPressed = 0;
	while (!ButtonPressed);								// Wait for button press
	ButtonPressed = 0;
	LCD_clear();
#endif

	return(MemoryProgramStatus);

}
#endif

