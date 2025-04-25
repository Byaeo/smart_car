/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"
#include "motor.h"
#include "niming.h"
#include "pid.h"

#include "cJSON.h"
#include <string.h>
#include "HC_SR04.h"



#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern short Encode1Count ;
extern short Encode2Count ;
extern float Motor1Speed ;
extern float Motor2Speed ;
extern uint16_t TimerCount ;



extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern tPid pidFollow;    
extern tPid pidMPU6050YawMovement;  
extern uint8_t Usart1_ReadBuf[255];	
extern float p,i,d,a,b;
extern uint8_t OledString[50];
extern float Mileage;

extern tPid pidHW_Tracking;
extern tPid pidOpenmv_Tracking;

extern uint8_t g_ucaHW_Read[4] ;
extern int8_t g_cThisState ;
extern int8_t g_cLastState ; 
extern float g_fHW_PID_Out;
extern float g_fHW_PID_Out1;
extern float g_fHW_PID_Out2;

extern uint8_t g_ucUsart3ReceiveData;  
extern uint8_t g_ucUsart2ReceiveData;  

extern uint8_t Usart3String[50];
extern float g_fHC_SR04_Read;
extern float g_fFollow_PID_Out;


extern float pitch,roll,yaw; 

extern float  g_fMPU6050YawMovePidOut ; 
extern float  g_fMPU6050YawMovePidOut1 ; 
extern float  g_fMPU6050YawMovePidOut2 ; 

extern uint8_t g_ucMode;

extern int g_lHW_State;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId StopTaskHandle;
osThreadId LedTaskHandle;
osThreadId OledTaskHandle;
osThreadId MultiModeTaskHandle;
osMessageQId myQueueModeHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartStopTask(void const * argument);
void StartLedTask(void const * argument);
void StartOledTask(void const * argument);
void StartMultiModeTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myQueueMode */
  osMessageQDef(myQueueMode, 1, uint8_t);
  myQueueModeHandle = osMessageCreate(osMessageQ(myQueueMode), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of StopTask */
  osThreadDef(StopTask, StartStopTask, osPriorityHigh, 0, 128);
  StopTaskHandle = osThreadCreate(osThread(StopTask), NULL);

  /* definition and creation of LedTask */
  osThreadDef(LedTask, StartLedTask, osPriorityNormal, 0, 128);
  LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);

  /* definition and creation of OledTask */
  osThreadDef(OledTask, StartOledTask, osPriorityNormal, 0, 128);
  OledTaskHandle = osThreadCreate(osThread(OledTask), NULL);

  /* definition and creation of MultiModeTask */
  osThreadDef(MultiModeTask, StartMultiModeTask, osPriorityAboveNormal, 0, 128);
  MultiModeTaskHandle = osThreadCreate(osThread(MultiModeTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartStopTask */
/**
  * @brief  Function implementing the StopTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartStopTask */
void StartStopTask(void const * argument)
{

  /* USER CODE BEGIN StartStopTask */
  uint8_t receivedMode;  
  /* Infinite loop */
  for(;;)
  {
  
    if (xQueuePeek(myQueueModeHandle, &receivedMode, 0) == errQUEUE_EMPTY)
    {
      receivedMode = 0; 
    }
    if(receivedMode == 0)
    {
      motorPidSetSpeed(0,0);
    }


    osDelay(10);
  }
  /* USER CODE END StartStopTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  for(;;)
  {

    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin); 
    osDelay(300);
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
* @brief Function implementing the OledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOledTask */
void StartOledTask(void const * argument)
{
  /* USER CODE BEGIN StartOledTask */

  uint8_t readMode;  

  /* Infinite loop */
  for(;;)
  {
   
    if (xQueuePeek(myQueueModeHandle, &readMode, 0) == errQUEUE_EMPTY)
    {
      readMode = 0;   
    }


    sprintf((char *)OledString," g_ucMode:%d",readMode);
    OLED_ShowString(0,6,OledString,12);	

    sprintf((char*)OledString, "lHW:%d  ", g_lHW_State);
    OLED_ShowString(0,7,OledString,12);//


    sprintf((char *)Usart3String," g_ucMode:%d",readMode);
    HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);


    //0LEDœ‘ æπ¶ƒ‹
    sprintf((char*)OledString, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);
    OLED_ShowString(0,0,OledString,12);

    sprintf((char*)OledString, "Mileage:%.2f", Mileage);
    OLED_ShowString(0,1,OledString,12);

    sprintf((char*)OledString, "U:%.2fV", adcGetBatteryVoltage());
    OLED_ShowString(0,2,OledString,12);

    sprintf((char *)OledString,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());
    OLED_ShowString(0,3,OledString,12);

    sprintf((char *)OledString,"p:%.2f r:%.2f \r\n",pitch,roll);
    OLED_ShowString(0,4,OledString,12);

    sprintf((char *)OledString,"y:%.2f  \r\n",yaw);
    OLED_ShowString(0,5,OledString,12);

    //¿∂—¿APPœ‘ æ
    sprintf((char*)Usart3String, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);
    HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
 
    sprintf((char*)Usart3String, "Mileage:%.2f", Mileage);
    HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);

    sprintf((char*)Usart3String, "U:%.2fV", adcGetBatteryVoltage());
    HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
    sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());
    HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);

    sprintf((char *)Usart3String,"p:%.2f r:%.2f \r\n",pitch,roll);
    HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);

    sprintf((char *)Usart3String,"y:%.2f  \r\n",yaw);
    HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);



    for(uint8_t i = 0; i < 20 ; i++)
    {
      if(mpu_dmp_get_data(&pitch,&roll,&yaw) == 0)
      {
        printf("mpu_dmp_get_data()  success\n"); 
        break;
      }

      if(i == 20-1 )
      {
        printf("mpu_dmp_get_data()  fail\n");
      }
    }


    osDelay(10);
  }
  /* USER CODE END StartOledTask */
}

/* USER CODE BEGIN Header_StartMultiModeTask */
/**
* @brief Function implementing the MultiModeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMultiModeTask */
void StartMultiModeTask(void const * argument)
{
  /* USER CODE BEGIN StartMultiModeTask */

  uint8_t readMode;  

  /* Infinite loop */
  for(;;)
  {
   
    if (xQueuePeek(myQueueModeHandle, &readMode, 0) == errQUEUE_EMPTY)
    {
      readMode = 0;  
    }

    if(readMode == 1)
    {
      //∫ÏÕ‚PID—≠º£
      g_ucaHW_Read[0] = READ_HW_OUT_1;
      g_ucaHW_Read[1] = READ_HW_OUT_2;
      g_ucaHW_Read[2] = READ_HW_OUT_3;
      g_ucaHW_Read[3] = READ_HW_OUT_4;

      if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
      {
        
        g_cThisState = 0;
      }
      else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
      {
       
        g_cThisState = -1;
      }
      else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
      {
       
        g_cThisState = -2;
      }
      else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
      {
      
        g_cThisState = -3;
      }
      else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
      {
        
        g_cThisState = 1;
      }
      else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
      {
        
        g_cThisState = 2;
      }
      else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
      {
       
        g_cThisState = 3;
      }
      g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);

      g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;
      g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;
      if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;
      if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
      if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;
      if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
      if(g_cThisState != g_cLastState)
      {
        motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);
      }

      g_cLastState = g_cThisState;

    }
 
    if(readMode == 2)
    {
      //±‹’œ¬ﬂº≠
      if(HC_SR04_Read() > 25)
      {
        motorPidSetSpeed(1,1);
        osDelay(100);
      }
      else 	
      {
        motorPidSetSpeed(-1,1);
        osDelay(500);
        if(HC_SR04_Read() > 25)
        {
          motorPidSetSpeed(1,1);
          osDelay(100);
        }
        else 
        {
          motorPidSetSpeed(1,-1);
          osDelay(1000);
          if(HC_SR04_Read() >25)
          {
            motorPidSetSpeed(1,1);
            osDelay(100);
          }
          else
          {
            motorPidSetSpeed(-1,-1);
            osDelay(1000);
            motorPidSetSpeed(-1,1);
            osDelay(50);
          }
        }
      }
    }
    if(readMode == 3)
    {
      g_fHC_SR04_Read=HC_SR04_Read();
      if(g_fHC_SR04_Read < 60)   
      {
        g_fFollow_PID_Out = PID_realize(&pidFollow,g_fHC_SR04_Read);
        if(g_fFollow_PID_Out > 6) g_fFollow_PID_Out = 6;
        if(g_fFollow_PID_Out < -6) g_fFollow_PID_Out = -6;
        motorPidSetSpeed(g_fFollow_PID_Out,g_fFollow_PID_Out);
      }
      else motorPidSetSpeed(0,0);
      osDelay(10);
    }
    if(readMode == 4)
    {
			//mpu6050 pid control
      sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);
      HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);


      for(uint8_t i = 0; i < 20 ; i++)
      {
        if(mpu_dmp_get_data(&pitch,&roll,&yaw) == 0)
        {
          printf("mpu_dmp_get_data() success\n");
        }

        if(i == 20-1 )
        {
          printf("mpu_dmp_get_data()  fail\n");
        }

        g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);

        g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;
        g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
        if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;
        if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
        if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;
        if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
        motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartMultiModeTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

