/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "c2cpp.h"
#include "bno055_stm32.h"
#include "../../../Common/Src/SRAM4.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

// Timing parameter
uint64_t _micros = 0;
uint64_t timeStamp = 0;
uint64_t xicroStamp = 0;
uint64_t starttime = 0;
uint64_t runstarttime = 0;
uint64_t runtime = 0;

// FDCAN1
FDCAN_TxHeaderTypeDef TxHeader1;
FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t TxData1[8] = {0x00, 0xDA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t RxData1[8];

// FDCAN2
FDCAN_TxHeaderTypeDef TxHeader2;
FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t TxData2[8] = {0x00, 0xDA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t RxData2[8];

// Used when read data from Hub motor
uint8_t TxWriteMsg1_isReturn = 0;
uint8_t TxWriteMsg2_isReturn = 0;

float RightMotor_PulseFeedback = 0;	// need to change to integer and do unwrapping later
float RightMotor_SpeedFeedback = 0;
float RightMotor_CurrentFeedback = 0;
float LeftMotor_PulseFeedback = 0;
float LeftMotor_SpeedFeedback = 0;
float LeftMotor_CurrentFeedback = 0;

// Robot Raw Data Input
float Right_DegRel = 0;
float Left_DegRel = 0;
float Right_DegSec = 0;
float Left_DegSec = 0;

// Robot Estimated State
float estimated_rightvel = 0;
float estimated_leftvel = 0;
float Var_RW = 0;
float Var_LW = 0;

// Robot Odometry
float Robot_LinVel = 0;
float Robot_AngVel = 0;
float Robot_X = 0;
float Robot_Y = 0;
float Robot_Yaw = 0;
float timestep = 0;
float Robot_LinVel_Var = 0;
float Robot_AngVel_Var = 0;

// Quaternion
float Robot_qx = 0;
float Robot_qy = 0;
float Robot_qz = 0;
float Robot_qw = 0;

// Command velocity
float cmd_vel_linear = 0;
float cmd_vel_angular = 0;
float RightMotor_CmdVel = 0;
float LeftMotor_CmdVel = 0;

int indx = 0;
uint8_t error = 0;
int run = 1;

int isMain = 0;
int txcount = 0;

// BNO055
double qw=0;
double qx=0;
double qy=0;
double qz=0;

double ax=0;
double ay=0;
double az=0;

double gx=0;
double gy=0;
double gz=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Pulse2Position();
void ForwardKinematic(float right_linvel, float left_linvel, float wheel_distance);
void InverseKinematic(float cmd_linvel, float cmd_angvel, float wheel_distance, float wheel_radius);
void WheelOdometry(float linear_velocity, float angular_velocity, float time_step);
void RPY2Quaternion(float roll, float pitch, float yaw);
//////////////////////////////////
void RightMotor_TestCommand();
void LeftMotor_TestCommand();
void BothMotor_Set_SpeedMode(float initial_right_speed_rpm, float initial_left_speed_rpm, uint8_t acc_time, uint8_t dec_time);
void Motor_Set_TargetSpeed(float right_speed_rpm, float left_speed_rpm);
void BothMotor_Enable();
void BothMotor_Release();
void BothMotor_EmergencyBrake();
void BothMotor_Get_Current();
void BothMotor_Get_Speed();
void BothMotor_Get_Position();
void TxData_Clear(uint8_t* TxData);
char compareTxRxMessage(uint8_t* TxData,uint8_t* RxData);
uint64_t micros();
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
	uint64_t dubugStamp;
	setup();
  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  	//Timer2 for micros
    HAL_TIM_Base_Start_IT(&htim2);

    // Configure global filter to reject all non-matching frames
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    // Start FDCAN1
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
  	  Error_Handler();
    }
    // Start FDCAN2
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
  	  Error_Handler();
    }
    // Activate the notification for new data in FIFO0 for FDCAN1
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
  	  Error_Handler();
    }
    // Activate the notification for new data in FIFO1 for FDCAN2
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
    {
  	  Error_Handler();
    }

    // Configure TX Header for FDCAN1 (left motor)
    TxHeader1.Identifier = 0x11;
    TxHeader1.IdType = FDCAN_STANDARD_ID;
    TxHeader1.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader1.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader1.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader1.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader1.MessageMarker = 0;

    // Configure TX Header for FDCAN2 (right motor)
    TxHeader2.Identifier = 0x22;
    TxHeader2.IdType = FDCAN_STANDARD_ID;
    TxHeader2.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader2.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader2.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader2.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader2.MessageMarker = 0;

    bno055_assignI2C(&hi2c1);
    bno055_setup();
    bno055_setOperationModeNDOF();

    BothMotor_Set_SpeedMode(10, 10, 1, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // 100Hz Control loop
	  if (micros() - timeStamp > 10000) {	// timestep = 10ms
		  timeStamp = micros();
		  runstarttime = micros();
		  isMain++;
		  if (run == 0){
//			  BothMotor_Release();
//			  BothMotor_Enable();	//used only after release brake
			  BothMotor_EmergencyBrake();
		  }
		  else if (run == 1) {
//			  Motor Read Position & Speed
			  BothMotor_Get_Position();
			  BothMotor_Get_Speed();
			  runtime = micros() - runstarttime;
			  Pulse2Position();
			  estimated_rightvel = update_rightwheel(Right_DegRel, Right_DegSec)*(M_PI/180)*0.085;	// DegSec to m/s
			  estimated_leftvel = update_leftwheel(Left_DegRel, Left_DegSec)*(M_PI/180)*0.085;		// DegSec to m/s
			  ForwardKinematic(estimated_rightvel, estimated_leftvel, 0.39377);
			  /*------------------------------*/
			  timestep = (micros() - starttime) * 0.000001;
			  starttime = micros();
			  /*------------------------------*/
			  WheelOdometry(Robot_LinVel, Robot_AngVel, timestep);
			  Robot_LinVel_Var = get_lin_vel_variance();
			  Robot_AngVel_Var = get_ang_vel_variance();
			  RPY2Quaternion(0.0, 0.0, Robot_Yaw);

			  cmd_vel_linear = shared_ptr->cmd_vel_linear;
			  cmd_vel_angular = shared_ptr->cmd_vel_angular;
			  InverseKinematic(cmd_vel_linear, cmd_vel_angular, 0.39377, 0.085);

			  Motor_Set_TargetSpeed(RightMotor_CmdVel, LeftMotor_CmdVel);
//			  Motor_Set_TargetSpeed(10, 10);

//			  Motor_Set_TargetSpeed((0.08901179185 * 60) / (0.085 * 2 * M_PI), (0.08901179185* 60) / (0.085 * 2 * M_PI));

			  // BNO055 Read Raw IMU Data
			  bno055_vector_t v = bno055_getVectorQuaternion();
			  qw = v.w;
			  qx = v.x;
			  qy = v.y;
			  qz = v.z;
			  v = bno055_getVectorLinearAccel();
			  ax = v.x;
			  ay = v.y;
			  az = v.z;
			  v = bno055_getVectorGyroscope();
			  gx = v.x;
			  gy = v.y;
			  gz = v.z;
		  }

		  shared_ptr->robot_x = Robot_X;
		  shared_ptr->robot_y = Robot_Y;
		  shared_ptr->robot_qx = Robot_qx;
		  shared_ptr->robot_qy = Robot_qy;
		  shared_ptr->robot_qz = Robot_qz;
		  shared_ptr->robot_qw = Robot_qw;
		  shared_ptr->robot_linvel = Robot_LinVel;
		  shared_ptr->robot_angvel = Robot_AngVel;
		  shared_ptr->nav_pub_flag = 1;

		  shared_ptr->imu_qx = qx;
		  shared_ptr->imu_qy = qy;
		  shared_ptr->imu_qz = qz;
		  shared_ptr->imu_qw = qw;
		  shared_ptr->imu_gx = gx;
		  shared_ptr->imu_gy = gy;
		  shared_ptr->imu_gz = gz;
		  shared_ptr->imu_ax = ax;
		  shared_ptr->imu_ay = ay;
		  shared_ptr->imu_az = az;
		  shared_ptr->imu_pub_flag = 1;

	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (HAL_GetTick() - dubugStamp > 500)
	  {
		  dubugStamp = HAL_GetTick();
		  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	  }
  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 7;
  hfdcan1.Init.DataTimeSeg1 = 8;
  hfdcan1.Init.DataTimeSeg2 = 7;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 10;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x11;
  sFilterConfig.FilterID2 = 0x7FF;	/* For acceptance, MessageID and FilterID1 must match exactly 0x7FF*/
  sFilterConfig.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK){

	  /* Filter configuration Error */
	  error = 4;
	  Error_Handler();
  }
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 2;
  hfdcan2.Init.NominalTimeSeg1 = 13;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 7;
  hfdcan2.Init.DataTimeSeg1 = 8;
  hfdcan2.Init.DataTimeSeg2 = 7;
  hfdcan2.Init.MessageRAMOffset = 1280;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 0;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 1;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 10;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x22;
  sFilterConfig.FilterID2 = 0x7FF;	/* For acceptance, MessageID and FilterID1 must match exactly */
  sFilterConfig.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK){

	  /* Filter configuration Error */
	  error = 5;
	  Error_Handler();
  }
  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00C0EAFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 99;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 576000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Pulse2Position()
{
	Right_DegRel = RightMotor_PulseFeedback * 0.0878;
	Left_DegRel = LeftMotor_PulseFeedback * 0.0878;
	Right_DegSec = RightMotor_SpeedFeedback * 6.0;
	Left_DegSec = LeftMotor_SpeedFeedback * 6.0;
}

void ForwardKinematic(float right_linvel, float left_linvel, float wheel_distance)
{
	Robot_LinVel = (right_linvel + left_linvel)*0.5;
	Robot_AngVel = (right_linvel - left_linvel)/wheel_distance;

}

void InverseKinematic(float cmd_linvel, float cmd_angvel, float wheel_distance, float wheel_radius)
{
	float right_speed, left_speed;	// unit: m/s
	right_speed = cmd_linvel + cmd_angvel*wheel_distance*0.5;
	left_speed = cmd_linvel - cmd_angvel*wheel_distance*0.5;
	// m/s to rpm
	RightMotor_CmdVel = (right_speed * 60) / (wheel_radius * 2 * M_PI);
	LeftMotor_CmdVel = (left_speed * 60) / (wheel_radius * 2 * M_PI);
	// Saturate motor if speed is too much
	uint8_t sat_value = 15;
	if (fabs(RightMotor_CmdVel) > sat_value){
		if (RightMotor_CmdVel > 0){
			RightMotor_CmdVel = sat_value;
		}
		else if (RightMotor_CmdVel < 0){
			RightMotor_CmdVel = -sat_value;
		}
	}
	if (fabs(LeftMotor_CmdVel) > sat_value){
		if (LeftMotor_CmdVel > 0){
			LeftMotor_CmdVel = sat_value;
		}
		else if (LeftMotor_CmdVel < 0){
			LeftMotor_CmdVel = -sat_value;
		}
	}
}

void WheelOdometry(float linear_velocity, float angular_velocity, float time_step)
{
	float temp_tetra = Robot_Yaw + (angular_velocity*time_step*0.5);
	Robot_X = Robot_X + cos(temp_tetra)*linear_velocity*time_step;
	Robot_Y = Robot_Y + sin(temp_tetra)*linear_velocity*time_step;
	Robot_Yaw = Robot_Yaw + angular_velocity*time_step;
}

// heading - y - pitch
// attitude - z - yaw
// bank - x - row
void RPY2Quaternion(float roll, float pitch, float yaw)
{
	float c1 = cos(pitch*0.5); //cp
	float s1 = sin(pitch*0.5);	//sp
	float c2 = cos(yaw*0.5);	//cy
	float s2 = sin(yaw*0.5);	//sy
	float c3 = cos(roll*0.5);	//cr
	float s3 = sin(roll*0.5);	//sr

	Robot_qw =c1*c2*c3 - s1*s2*s3;	//cp*cy*cr + sp*sy*sr
	Robot_qx =c1*c2*s3 + s1*s2*c3;	//cp*cy*sr + sp*sy*cr
	Robot_qy =s1*c2*c3 + c1*s2*s3;	//sp*cy*cr + cp*sy*sr
	Robot_qz =c1*s2*c3 - s1*c2*s3;	//cp*sy*cr + sp*cy*sr
}
// FDCAN1 Callback
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	error = 9;
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		/* Retreive Rx message from RX FIFO0 */
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, RxData1) != HAL_OK)
		{
			/* Reception Error */
			Error_Handler();
		}
		// Check if Rx message is return message
		if (RxData1[1] == 0xDB){
//			TxWriteMsg1_isReturn = compareTxRxMessage(TxData1, RxData1);
			// Check if Rx message is Tx write command return message
			if (TxData1[1] == 0xDA){
				TxWriteMsg1_isReturn = compareTxRxMessage(TxData1, RxData1);
			}
			// Check if Rx message is Tx read command return message
			else if ((TxData1[1] == 0xDC)){
				// read current
				if (RxData1[3] == 0xE2){
					RightMotor_CurrentFeedback = ((RxData1[6]<<8) | (RxData1[7])) / 100.0;
					TxWriteMsg1_isReturn = 1;
				}
				// read speed
				else if (RxData1[3] == 0xE4){
					RightMotor_SpeedFeedback = -(((int16_t)((RxData1[6]<<8) | (RxData1[7])) / 8192.0) * 3000.0);
					TxWriteMsg1_isReturn = 1;
				}
				// read position
				else if (RxData1[3] == 0xE8){
					RightMotor_PulseFeedback = -((RxData1[4]<<24) | (RxData1[5]<<16) | (RxData1[6]<<8) | (RxData1[7]));
					TxWriteMsg1_isReturn = 1;
				}
			}
		}
		// Check if Rx message is heartbeat message
//		else if (RxData1[1] == 0xFE){
//			if (RxData1[3] == 0x20){
//				RightMotor_PulseFeedback = -((RxData1[4]<<24) | (RxData1[5]<<16) | (RxData1[6]<<8) | (RxData1[7]));
//			}
//			else if (RxData1[3] == 0x21){
//				RightMotor_CurrentFeedback = ((RxData1[4]<<8) | (RxData1[5])) / 100.0;
//				RightMotor_SpeedFeedback = (((RxData1[6]<<8) | (RxData1[7])) / 8192.0) * 3000.0;
//			}
//		}
		if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
		{
			/* Reception Error */
			Error_Handler();
		}
	}
}
// FDCAN2 Callback
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	error = 10;
	if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	{
		/* Retreive Rx message from RX FIFO1 */
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, RxData2) != HAL_OK)
		{
			/* Reception Error */
			Error_Handler();
		}
		// Check if Rx message is return message
		if (RxData2[1] == 0xDB){
//			TxWriteMsg2_isReturn = compareTxRxMessage(TxData2, RxData2);
			// Check if Rx message is Tx write command return message
			if (TxData2[1] == 0xDA){
				TxWriteMsg2_isReturn = compareTxRxMessage(TxData2, RxData2);
			}
			// Check if Rx message is Tx read command return message
			else if ((TxData2[1] == 0xDC)){
				// read current
				if (RxData2[3] == 0xE2){
					LeftMotor_CurrentFeedback = ((RxData2[6]<<8) | (RxData2[7])) / 100.0;
					TxWriteMsg2_isReturn = 1;
				}
				// read speed
				else if (RxData2[3] == 0xE4){
					LeftMotor_SpeedFeedback = ((int16_t)((RxData2[6]<<8) | (RxData2[7])) / 8192.0) * 3000.0;
					TxWriteMsg2_isReturn = 1;
				}
				// read position
				else if (RxData2[3] == 0xE8){
					LeftMotor_PulseFeedback = (RxData2[4]<<24) | (RxData2[5]<<16) | (RxData2[6]<<8) | (RxData2[7]);
					TxWriteMsg2_isReturn = 1;
				}
			}
		}
		// Check if Rx message is heartbeat message
//		else if (RxData2[1] == 0xFE){
//			if (RxData2[3] == 0x20){
//				LeftMotor_PulseFeedback = (RxData2[4]<<24) | (RxData2[5]<<16) | (RxData2[6]<<8) | (RxData2[7]);
//			}
//			else if (RxData2[3] == 0x21){
//				LeftMotor_CurrentFeedback = ((RxData2[4]<<8) | (RxData2[5])) / 100.0;
//				LeftMotor_SpeedFeedback = (((RxData2[6]<<8) | (RxData2[7])) / 8192.0) * 3000.0;
//			}
//		}
		if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
		{
			/* Reception Error */
			Error_Handler();
		}
	}
}

void RightMotor_TestCommand()
{
	// Clear Tx data
	TxData_Clear(TxData1);

	// Set Working mode to Position mode (used as Test command)
	TxData1[3] = 0x19;	// internal address
	TxData1[7] = 0x3F;	// set position mode value
	// Sent command to ZLAC706-CAN motor driver
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, TxData1) != HAL_OK)	{Error_Handler();}
	// Wait until tx message is received at the rx
	TxWriteMsg1_isReturn = 0;
	while (!TxWriteMsg1_isReturn);
}

void LeftMotor_TestCommand()
{
	// Clear Tx data
	TxData_Clear(TxData2);

	// Set Working mode to Position mode (used as Test command)
	TxData2[3] = 0x19;	// internal address
	TxData2[7] = 0x3F;	// set position mode value
	// Sent command to ZLAC706-CAN motor driver
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, TxData2) != HAL_OK)	{Error_Handler();}
	// Wait until tx message is received at the rx
	TxWriteMsg2_isReturn = 0;
	while (!TxWriteMsg2_isReturn);
}

void BothMotor_Set_SpeedMode(float initial_right_speed_rpm, float initial_left_speed_rpm, uint8_t acc_time, uint8_t dec_time)
{
	// Clear Tx data
	TxData_Clear(TxData1);
	TxData_Clear(TxData2);

	// Set Working mode to Speed mode for right motor
	TxData1[3] = 0x19;		// internal address
	TxData1[7] = 0x2F;		// set speed mode value
	// Set Working mode to Speed mode for left motor
	TxData2[3] = 0x19;		// internal address
	TxData2[7] = 0x2F;		// set speed mode value
	// Sent command to ZLAC706-CAN motor driver
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, TxData1) != HAL_OK)	{Error_Handler();}	// declare more than 1
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, TxData2) != HAL_OK)	{Error_Handler();}

	// Set Acceleration and Deceleration for right motor
	TxData1[3] = 0x13;		// internal address
	TxData1[6] = acc_time;	// set acceleration = acc_time x 100ms
	TxData1[7] = dec_time;	// set deceleration = dec_time x 100ms
	// Set Acceleration and Deceleration for left motor
	TxData2[3] = 0x13;		// internal address
	TxData2[6] = acc_time;	// set acceleration = acc_time x 100ms
	TxData2[7] = dec_time;	// set deceleration = dec_time x 100ms
	// Sent command to ZLAC706-CAN motor driver
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, TxData1) != HAL_OK)	{Error_Handler();}
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, TxData2) != HAL_OK)	{Error_Handler();}

	Motor_Set_TargetSpeed(initial_right_speed_rpm, initial_left_speed_rpm);
	BothMotor_Enable();
	BothMotor_EmergencyBrake();
}

void Motor_Set_TargetSpeed(float right_speed_rpm, float left_speed_rpm)
{
	// Clear Tx data
	TxData_Clear(TxData1);
	TxData_Clear(TxData2);

	// Set Right motor target speed
	TxData1[3] = 0x11;		// internal address
	uint32_t set_value = (fabs(right_speed_rpm) * 8192.0) / 3000;	// calculate 32 bit set value from actual speed
	if (right_speed_rpm >= 0){
		set_value = ~set_value + 1;	// reverse (two complement)
	} else {
		set_value = set_value;	// forward
	}
	TxData1[4] = (set_value >> 24) & 0xFF;	// set Byte4
	TxData1[5] = (set_value >> 16) & 0xFF; 	// set Byte5
	TxData1[6] = (set_value >> 8) & 0xFF;	// set Byte6
	TxData1[7] = set_value & 0xFF;			// set Byte7

	// Set Left motor target speed
	TxData2[3] = 0x11;		// internal address
	set_value = (fabs(left_speed_rpm) * 8192.0) / 3000;	// calculate 32 bit set value from actual speed
	if (left_speed_rpm >= 0){
		set_value = set_value;	// forward
	} else {
		set_value = ~set_value + 1;	// reverse (two complement)
	}
	TxData2[4] = (set_value >> 24) & 0xFF;	// set Byte4
	TxData2[5] = (set_value >> 16) & 0xFF; 	// set Byte5
	TxData2[6] = (set_value >> 8) & 0xFF;	// set Byte6
	TxData2[7] = set_value & 0xFF;			// set Byte7

	// Sent command to ZLAC706-CAN motor driver
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, TxData1) != HAL_OK)	{error = 2; Error_Handler();}
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, TxData2) != HAL_OK)	{error = 3; Error_Handler();}
}

void BothMotor_Enable()
{
	// Clear Tx data
	TxData_Clear(TxData1);
	TxData_Clear(TxData2);

	// Enable the right motor
	TxData1[3] = 0x10;	// internal address
	TxData1[7] = 0x1F;	// set enable motor value
	// Enable the left motor
	TxData2[3] = 0x10;	// internal address
	TxData2[7] = 0x1F;	// set enable motor value
	// Sent command to ZLAC706-CAN motor driver
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, TxData1) != HAL_OK)	{Error_Handler();}
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, TxData2) != HAL_OK)	{Error_Handler();}
}

// only work for right motor for now
void BothMotor_Release()
{
	// Clear Tx data
	TxData_Clear(TxData1);
	TxData_Clear(TxData2);

	// Release the right motor with down time
	TxData1[3] = 0x10;	// internal address
	TxData1[7] = 0x0F;	// set release motor value
	// Release the left motor with down time
	TxData2[3] = 0x10;	// internal address
	TxData2[7] = 0x0F;	// set release motor value
	// Sent command to ZLAC706-CAN motor driver
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, TxData1) != HAL_OK)	{Error_Handler();}
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, TxData2) != HAL_OK)	{Error_Handler();}
}

void BothMotor_EmergencyBrake()
{
	// Clear Tx data
	TxData_Clear(TxData1);
	TxData_Clear(TxData2);

	// Emergency stop the right motor
	TxData1[3] = 0x30;	// internal address
	TxData1[7] = 0x1F;	// set emergency stop value
	// Emergency stop the left motor
	TxData2[3] = 0x30;	// internal address
	TxData2[7] = 0x1F;	// set emergency stop value
	// Sent command to ZLAC706-CAN motor driver
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, TxData1) != HAL_OK)	{Error_Handler();}
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, TxData2) != HAL_OK)	{Error_Handler();}
}

void BothMotor_Get_Current()
{
	// Clear Tx data
	TxData_Clear(TxData1);
	TxData_Clear(TxData2);

	// Get right motor current
	TxData1[1] = 0xDC;	// set tx read command
	TxData1[3] = 0xE2;	// internal address
	// Get left motor current
	TxData2[1] = 0xDC;	// set tx read command
	TxData2[3] = 0xE2;	// internal address
	// Sent command to ZLAC706-CAN motor driver
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, TxData1) != HAL_OK)	{Error_Handler();}
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, TxData2) != HAL_OK)	{Error_Handler();}
	// Wait until tx message is received at the rx
	TxWriteMsg1_isReturn = 0;
	TxWriteMsg2_isReturn = 0;
	uint64_t timeout = micros();
	while (!(TxWriteMsg1_isReturn && TxWriteMsg2_isReturn)){
		// 1 ms request timeout
		if (micros() - timeout > 1000){
			break;
		}
	}
}

void BothMotor_Get_Speed()
{
	// Clear Tx data
	TxData_Clear(TxData1);
	TxData_Clear(TxData2);

	// Get right motor speed
	TxData1[1] = 0xDC;	// set tx read command
	TxData1[3] = 0xE4;	// internal address
	// Get left motor speed
	TxData2[1] = 0xDC;	// set tx read command
	TxData2[3] = 0xE4;	// internal address
	// Sent command to ZLAC706-CAN motor driver
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, TxData1) != HAL_OK)	{Error_Handler();}
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, TxData2) != HAL_OK)	{Error_Handler();}
	// Wait until tx message is received at the rx
	TxWriteMsg1_isReturn = 0;
	TxWriteMsg2_isReturn = 0;
	uint64_t timeout = micros();
	while (!(TxWriteMsg1_isReturn && TxWriteMsg2_isReturn)){
		// 1 ms request timeout
		if (micros() - timeout > 1000){
			break;
		}
	}
}

void BothMotor_Get_Position()
{
	// Clear Tx data
	TxData_Clear(TxData1);
	TxData_Clear(TxData2);

	// Get right motor pulse
	TxData1[1] = 0xDC;	// set tx read command
	TxData1[3] = 0xE8;	// internal address
	// Get left motor pulse
	TxData2[1] = 0xDC;	// set tx read command
	TxData2[3] = 0xE8;	// internal address
	// Sent command to ZLAC706-CAN motor driver
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, TxData1) != HAL_OK)	{Error_Handler();}
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, TxData2) != HAL_OK)	{Error_Handler();}
	// Wait until tx message is received at the rx
	TxWriteMsg1_isReturn = 0;
	TxWriteMsg2_isReturn = 0;
	uint64_t timeout = micros();
	while (!(TxWriteMsg1_isReturn && TxWriteMsg2_isReturn)){
		// 1 ms request timeout
		if (micros() - timeout > 1000){
			break;
		}
	}
}

void TxData_Clear(uint8_t* TxData)
{
	// reset Tx data to {00, [DA/DC], 00, 00, 00, 00, 00, 00}
	for (int i=0; i<8; i++){
		if (i == 1){
			TxData[i] = 0xDA;
		}
		else {
			TxData[i] = 0;
		}
	}
}

// compare Tx buffer to Rx buffer after write command
char compareTxRxMessage(uint8_t* TxData,uint8_t* RxData)
{
	// compare every element except second element(DA,DB)
	for(int i=0;i<8;i++){
		if(i != 1){
			if(TxData[i] != RxData[i])
				return 0;	// not equal
		}
	}
	return 1;
}

uint64_t micros()
{
	return _micros + htim2.Instance->CNT;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		_micros += 4294967295;
	}
}
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
