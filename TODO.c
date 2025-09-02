///TODO CON UNA FUNCION DE UART 


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "TJ_MPU6050.h"
#include <string.h>
#include <stdio.h>

#define TRANSMITTER_BOARD


I2C_HandleTypeDef hi2c1;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void Error_Handler(void);
static void EXTI0_IRQHandler_Config(void);
void UART_Print(const char *mensaje);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);


volatile int counter; 
volatile int count , CONTUART , CONTPERDON; 

RawData_Def myAccelRaw, myGyroRaw;
ScaledData_Def myAccelScaled, myGyroScaled;

int cont_plana = 0;
int cont_levantada=0;
int contador_inclinada_abajo = 0;
int cont_inclinada = 0;
int cont_girada_izq = 0;
int cont_girada_der = 0;
int cont_neutral = 0;

UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;
__IO uint32_t VirtualUserButtonStatus = 0; 


/* Buffer used for transmission */
uint8_t aTxBuffer[] = "STM32 Ready\n"; // Mensaje inicial
uint8_t aRxBuffer[100]; // Buffer para recibir datos
uint8_t message[] = "Hola desde STM32\n";

/* DEFINICION DE ESTRUCTURA DEL CANAL DEL ADC */
ADC_HandleTypeDef    NEWHANDLE;
ADC_ChannelConfTypeDef newConfig;


//VARIABLES ADC



#define  PERIOD_VALUE       (uint32_t)(13333 - 1)  /* Period Value  */
#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE/5) 


///tim
TIM_HandleTypeDef    TimHandle;
uint32_t uwPrescalerValue = 0;
/* Private functions ---------------------------------------------------------*/
volatile int lectura_sensor1=0; // 36K A 15K 3800 a 2900 canal12
volatile int lectura_sensor2=0;//30 10K  4095 a 3200 canal 11
volatile int lectura_sensor3=0;// 30 A 10 4095 a 2700 canal 10
volatile int lectura_sensor4=0;// 30 11K 4095 a 3200 canal 9
volatile int lectura_sensor5=0;// 30 12KL 4095 a 2900 canal 8
volatile int lectura_sensor6=0;// //54 27k 2000 1600 canal 6


volatile float angulo1=0;
volatile float angulo2=0;
volatile float angulo3=0;
volatile float angulo4=0;
volatile float angulo5=0;
volatile float angulo6=0;


void ADC_SELECT_CHANNEL6(void);
void ADC_SELECT_CHANNEL8(void);
void ADC_SELECT_CHANNEL9(void);
void ADC_SELECT_CHANNEL10(void);
void ADC_SELECT_CHANNEL11(void);
void ADC_SELECT_CHANNEL12(void);

void sensor_1(void);
void sensor_2(void);
void sensor_3(void);
void sensor_4(void);
void sensor_5(void);
void sensor_6(void);

void LeerADC(void);
// Llamar a cada función de detección
void    DetectarMal();
void    DetectarBien();
void    DetectarPerdon();
void 		DetectarHola();
void    DetectarGracias();
void    DetectarDenada();
void    DetectarNo();
void    DetectarSi();
void    DetectarLunes();
void		DetectarMiercoles();
void		DetectarViernes();
void		DetectarEnojado();


int main(void)

{
 
	MPU_ConfigTypeDef myMpuConfig;
	
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
	MPU6050_Init(&hi2c1);
	
	BSP_LED_Init(LED3);
	
	GPIO_InitTypeDef          GPIO_ADC_InitStruct, GPIO_ADC_InitStruct2, GPIO_ADC_InitStruct3, GPIO_ADC_InitStruct4, GPIO_ADC_InitStruct5, GPIO_ADC_InitStruct6;
	
	//ADC
	/* ADC Periph clock enable */
  __HAL_RCC_ADC_CLK_ENABLE();
  /* ADC Periph interface clock configuration */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);
	
	/* CONFIGURACION GPIO DEL ADC */
  GPIO_ADC_InitStruct.Pin = GPIO_PIN_7; // Canal 12
	GPIO_ADC_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_ADC_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_ADC_InitStruct);

	GPIO_ADC_InitStruct.Pin = GPIO_PIN_1; // Canal 6
	HAL_GPIO_Init(GPIOA, &GPIO_ADC_InitStruct);
	
	GPIO_ADC_InitStruct.Pin = GPIO_PIN_3; // Canal 8
	HAL_GPIO_Init(GPIOA, &GPIO_ADC_InitStruct);
	
	GPIO_ADC_InitStruct.Pin = GPIO_PIN_4; // Canal 9
	HAL_GPIO_Init(GPIOA, &GPIO_ADC_InitStruct);

	GPIO_ADC_InitStruct.Pin = GPIO_PIN_5; // Canal 10
	HAL_GPIO_Init(GPIOA, &GPIO_ADC_InitStruct);
	
	GPIO_ADC_InitStruct.Pin = GPIO_PIN_6; // Canal 11
	HAL_GPIO_Init(GPIOA, &GPIO_ADC_InitStruct);
  
		
  NEWHANDLE.Instance          = ADC1;
	
	if (HAL_ADC_DeInit(&NEWHANDLE) != HAL_OK)
  {
    /* ADC de-initialization Error */
    Error_Handler();
  }
 
  NEWHANDLE.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;          /* Asynchronous clock mode, input ADC clock not divided */
  NEWHANDLE.Init.Resolution            = ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
  NEWHANDLE.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
  NEWHANDLE.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  NEWHANDLE.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
  NEWHANDLE.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
  NEWHANDLE.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
  NEWHANDLE.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
  NEWHANDLE.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  NEWHANDLE.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
  NEWHANDLE.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
  NEWHANDLE.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
  NEWHANDLE.Init.DMAContinuousRequests = DISABLE;                       /* DMA one-shot mode selected (not applied to this example) */
  NEWHANDLE.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
  NEWHANDLE.Init.OversamplingMode      = DISABLE;  


  if (HAL_ADC_Init(&NEWHANDLE) != HAL_OK)
  {
    /* ADC initialization Error */
    Error_Handler();
  }
	
  /* Run the ADC calibration in single-ended mode */
  if (HAL_ADCEx_Calibration_Start(&NEWHANDLE, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }
	
	///////////////////////////////////////////////
	
	//INTERRUPCION/////////////////////////////
	/* DEFINICION DE ESTRUCTURA DEL BOTON  */
	GPIO_InitTypeDef   InterruptButtonConfig;
	
	/* CONFIGURACION DEL BOTON EXTERNO PARA LA INTERRUPCION */
  InterruptButtonConfig.Mode = GPIO_MODE_IT_FALLING;
  InterruptButtonConfig.Pull = GPIO_PULLUP;
  InterruptButtonConfig.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &InterruptButtonConfig);
	
	 /* Enable and set EXTI OF PIN_0 AT THE HIGHEST PRIORITY */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
///////////////////////////////////////////////////////////////////////////////	
	/* PREESCALAR DEL TIMER */
	__HAL_RCC_TIM2_CLK_ENABLE();
  uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;// prescalar == fclock/fconteo periodo = fc*tiempo q quiero

  /* Set TIMx instance */
  TimHandle.Instance = TIM2;
  TimHandle.Init.Period            = 300 - 1; //CONTEO EJ: SI ESTO ES 5000 EL CONTEO ES CADA MEDIO SEGUNDO  //periodo 
  TimHandle.Init.Prescaler         = uwPrescalerValue;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP; //VA CRECIENDO
  TimHandle.Init.RepetitionCounter = 0;
	
	HAL_TIM_Base_Init(&TimHandle);
	HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
	

	
	//INICIAR EL TIMER
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  //INICIAR LA INTERRUPCION
	
	  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  } 
	
  /* Configure LED3 */
  BSP_LED_Init(LED3);
	
	
	////UART////////////////////
	
	
  UartHandle.Instance        = USART1;

  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }

	
	///////////////////////
	


  /* -2- Configure IO in output push-pull mode to drive external LEDs */


	
	myMpuConfig.Accel_Full_Scale=AFS_SEL_4g;
	myMpuConfig.ClockSource= Internal_8MHz;
	myMpuConfig.CONFIG_DLPF= DLPF_184A_188G_Hz;
	myMpuConfig.Gyro_Full_Scale= FS_SEL_500;
	myMpuConfig.Sleep_Mode_Bit=0;
	
	
	MPU6050_Config(&myMpuConfig);
	

  while (1)
  {
//		MPU6050_Get_Accel_Scale(&myAccelScaled);

//		if (myAccelScaled.z > 0.8f) {  // Mano plana (horizontal normal)
//    cont_plana++;
//		}
//		else if (myAccelScaled.z < -0.8f) {  // Mano plana invertida (hacia abajo)
//				cont_plana++;
//		}
//		else if (myAccelScaled.x > 0.7f) {  // Mano levantada (vertical)
//				cont_levantada++;
//		}
//		else if (myAccelScaled.x < -0.7f) {  // Mano inclinada hacia abajo
//				contador_inclinada_abajo++;
//		}
//		else if (myAccelScaled.y > 0.7f) {  // Mano girada a la izquierda
//				cont_girada_izq++;
//		}
//		else if (myAccelScaled.y < -0.7f) {  // Mano girada a la derecha
//				cont_girada_der++;
//		}


			
  }
  /* USER CODE END 3 */
}
/////FUNCIONES/////////////////////////////////
//ADC//

void ADC_SELECT_CHANNEL6(void){
	newConfig.Channel      = ADC_CHANNEL_6;                /* Sampled channel number */
  newConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  newConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  newConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  newConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  newConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
  if (HAL_ADC_ConfigChannel(&NEWHANDLE, &newConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
}

void ADC_SELECT_CHANNEL8(void){
	newConfig.Channel      = ADC_CHANNEL_8;                /* Sampled channel number */
  newConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  newConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  newConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  newConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  newConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
  if (HAL_ADC_ConfigChannel(&NEWHANDLE, &newConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
}

void ADC_SELECT_CHANNEL9(void){
	newConfig.Channel      = ADC_CHANNEL_9;                /* Sampled channel number */
  newConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  newConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  newConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  newConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  newConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
  if (HAL_ADC_ConfigChannel(&NEWHANDLE, &newConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
}

void ADC_SELECT_CHANNEL10(void){
	newConfig.Channel = ADC_CHANNEL_10;
  newConfig.Rank = ADC_REGULAR_RANK_1;
	newConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  newConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  newConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  newConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
  if (HAL_ADC_ConfigChannel(&NEWHANDLE, &newConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void ADC_SELECT_CHANNEL11(void){
	newConfig.Channel      = ADC_CHANNEL_11;                /* Sampled channel number */
  newConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  newConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  newConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  newConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  newConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
  if (HAL_ADC_ConfigChannel(&NEWHANDLE, &newConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
}
void ADC_SELECT_CHANNEL12(void){
	newConfig.Channel      = ADC_CHANNEL_12;                /* Sampled channel number */
  newConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  newConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  newConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  newConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  newConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
  if (HAL_ADC_ConfigChannel(&NEWHANDLE, &newConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
}


void LeerADC(void) {
    
			 // Canal 12
		ADC_SELECT_CHANNEL12();
		HAL_ADC_Start(&NEWHANDLE);
		HAL_ADC_PollForConversion(&NEWHANDLE, 10);
		lectura_sensor1 = HAL_ADC_GetValue(&NEWHANDLE);
		angulo1 = ((float)(lectura_sensor1 - 2800) * 90.0) / (3800.0 - 2800.0);
		HAL_ADC_Stop(&NEWHANDLE);
			
			// Canal 11
		ADC_SELECT_CHANNEL11();
		HAL_ADC_Start(&NEWHANDLE);
		HAL_ADC_PollForConversion(&NEWHANDLE, 10);
		lectura_sensor2 = HAL_ADC_GetValue(&NEWHANDLE);
		angulo2 = ((float)(lectura_sensor2 - 3800) * 90.0) / (4090.0 - 3800.0);
		HAL_ADC_Stop(&NEWHANDLE);

		// Canal 10
		ADC_SELECT_CHANNEL10();
		HAL_ADC_Start(&NEWHANDLE);
		HAL_ADC_PollForConversion(&NEWHANDLE, 10);
		lectura_sensor3 = HAL_ADC_GetValue(&NEWHANDLE);
		angulo3 = ((float)(lectura_sensor3 - 3800) * 90.0) / (4095.0 - 3800.0);
		HAL_ADC_Stop(&NEWHANDLE);

		// Canal 9
		ADC_SELECT_CHANNEL9();
		HAL_ADC_Start(&NEWHANDLE);
		HAL_ADC_PollForConversion(&NEWHANDLE, 10);
		lectura_sensor4 = HAL_ADC_GetValue(&NEWHANDLE);
		angulo4 = ((float)(lectura_sensor4 - 3200) * 90.0) / (4095.0 - 3200.0);
		HAL_ADC_Stop(&NEWHANDLE);

		// Canal 8
		ADC_SELECT_CHANNEL8();
		HAL_ADC_Start(&NEWHANDLE);
		HAL_ADC_PollForConversion(&NEWHANDLE, 10);
		lectura_sensor5 = HAL_ADC_GetValue(&NEWHANDLE);
		angulo5 = ((float)(lectura_sensor5 - 2750) * 90.0) / (4000.0 - 2750.0);
		HAL_ADC_Stop(&NEWHANDLE);


		// Canal 6
		ADC_SELECT_CHANNEL6();
		HAL_ADC_Start(&NEWHANDLE);
		HAL_ADC_PollForConversion(&NEWHANDLE, 10);
		lectura_sensor6 = HAL_ADC_GetValue(&NEWHANDLE);
		angulo6 = ((float)(lectura_sensor6 - 1700) * 90.0) / (2800.0 - 1700.0);
		HAL_ADC_Stop(&NEWHANDLE);



    DetectarMal();
		DetectarBien();
    DetectarPerdon();
    DetectarHola();
    DetectarGracias();
		DetectarDenada();
    DetectarNo();
    DetectarSi();
		DetectarLunes();
		DetectarMiercoles();
		DetectarViernes();
		DetectarEnojado();
		
}
	

// Función para la seña "Mal"
void DetectarMal(void) {
    if (angulo1 < 30.0 && angulo2 > 60.0 && angulo3 > 60.0 &&
        angulo4 > 60.0 && angulo5 > 60.0 && myAccelScaled.x > 0.97  && myAccelScaled.y < 1.1  && myAccelScaled.z < 1.1) {
        UART_Print("MAL\n");
    }
}
// Función para la seña "Bien"
void DetectarBien(void) {
    if (angulo1 < 30.0 && angulo2 > 60.0 && angulo3 > 60.0 &&
        angulo4 > 60.0 && angulo5 > 60.0 && myAccelScaled.x < 0.85   && myAccelScaled.y < 1.1  && myAccelScaled.z < 1.1) {
        UART_Print("BIEN\n");
    }
}

// Función para la seña "Perdón"
void DetectarPerdon(void) {
    if (angulo1 > 60.0 && angulo2 < 30.0 && angulo3 < 30.0 &&
        angulo4 < 30.0 && angulo5 > 60.0 && myAccelScaled.x > 0.99 && myAccelScaled.y < 1.1  && myAccelScaled.z < 1.1) {
					CONTPERDON++;
        UART_Print("PERDON\n");
    }
}

// Función para la seña "Sí"
void DetectarSi(void) {
    if (angulo1 < 35.0 && angulo2 < 30.0 && angulo3 < 30.0 &&
        angulo4 < 30.0 && angulo5 > 60.0 && myAccelScaled.x < 0.94 && myAccelScaled.y < 1.1  && myAccelScaled.z < 1.1) {
        UART_Print("SI\n");;
    }
}

// Función para la seña "Hola"
void DetectarHola(void) {
    if (angulo1 > 70.0 && angulo2 > 70.0 && angulo3 > 70.0 &&
        angulo4 < 30.0 && angulo5 < 30.0 && myAccelScaled.x > 0.97 && myAccelScaled.y < 1.1  && myAccelScaled.z < 1.1) {
        UART_Print("HOLA\n");
    }
}

// Función para la seña "No"
void DetectarNo(void) {
    if (angulo1 < 60.0 && angulo1 > 20.0 && angulo2 < 60.0 && angulo2 > 20.0 && angulo3 < 60.0 && angulo3 > 20.0&&
        angulo4 < 30.0 && angulo5 < 30.0 && myAccelScaled.x > 0.9 < 1 && myAccelScaled.y <1.1 && myAccelScaled.z <1.1 ) {
        UART_Print("NO\n"); 
    }
}

// Función para la seña "Gracias"
void DetectarGracias(void) {
    if (angulo1 > 60.0 && angulo2 > 60.0 && angulo3 < 30.0 &&
        angulo4 > 60.0 && angulo5 > 60.0 && myAccelScaled.x > 0.4 && myAccelScaled.y >-0.3  && myAccelScaled.z < 1.1) {
				
				CONTUART++;
        UART_Print("GRACIAS\n");
    }
}

// Función para la seña "DENADA"
void DetectarDenada(void) {
    if (angulo1 > 65.0 && angulo2 < 30.0 && angulo3 < 30.0 &&
        angulo4 < 30.0 && angulo5 < 30.0 && myAccelScaled.x > 0.67 && myAccelScaled.y < 1.1  && myAccelScaled.z < 1.1
		) {				
				CONTUART++;
        UART_Print("DENADA\n");
    }
}
// Función para la seña "LUNES"
void DetectarLunes(void) {
	if (angulo1 > 65.0 && angulo2 > 65.0 && angulo3 < 30.0 &&
        angulo4 < 30.0 && angulo5 < 30.0 && myAccelScaled.x > 0.97 && myAccelScaled.y < 1.1  && myAccelScaled.z < 1.1
		) {
        UART_Print("LUNES\n");
    }
}

// Función para la seña "MIERCOLES"
void DetectarMiercoles(void) {
    if (angulo1 <30.0 && angulo2 > 65.0 && angulo3 > 65.0 &&
        angulo4 > 65.0 && angulo5 < 30.0 && myAccelScaled.x > 0.97 && myAccelScaled.y < 1.1  && myAccelScaled.z < 1.1// && myAccelScaled.x > 0.67 && myAccelScaled.y < 1.1  && myAccelScaled.z < 1.1
		) {
        UART_Print("MIERCOLES\n");
    }
}

// Función para la seña "VIERNES"
void DetectarViernes(void) {
    if (angulo1 < 30.0 && angulo2 > 65.0 && angulo3 > 65.0 &&
        angulo4 < 30.0 && angulo5 < 30.0 && myAccelScaled.x > 0.97 && myAccelScaled.y < 1.1  && myAccelScaled.z < 1.1//&& myAccelScaled.x > 0.67 && myAccelScaled.y < 1.1  && myAccelScaled.z < 1.1
		) {
        UART_Print("VIERNES\n");
    }
}

// Función para la seña "ENOJADO"
void DetectarEnojado(void) {
    if (angulo1 < 45.0 && angulo2 > 70.0 && angulo3 < 30.0 &&
        angulo4 < 30.0 && angulo5 < 30.0 && myAccelScaled.x <0.89  && myAccelScaled.y <1.1 && myAccelScaled.z <1.1 
		) {
        UART_Print("ENOJADO\n");
    }
}





//Funcion para la UART

void UART_Print(const char *mensaje)
{
    static char ultimo_mensaje[100] = ""; // Almacena el último mensaje enviado

    // Compara el nuevo mensaje con el último mensaje
    if (strcmp(ultimo_mensaje, mensaje) != 0) // Si los mensajes son diferentes
    {
        char buffer[100];
        strncpy(buffer, mensaje, sizeof(buffer) - 1);
        buffer[sizeof(buffer) - 1] = '\0'; // Asegura la terminación nula

        HAL_UART_Transmit(&UartHandle, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

        // Actualiza el último mensaje transmitido
        strncpy(ultimo_mensaje, mensaje, sizeof(ultimo_mensaje) - 1);
        ultimo_mensaje[sizeof(ultimo_mensaje) - 1] = '\0'; // Asegura terminación nula
			}
}
///////////////////////////////////////////////

//INTERRUPCIONES///

void HAL_GPIO_EXT0_Callback(uint16_t GPIO_Pin)
{
	
}
//ISR GPIO PIN (TOMA DE DATOS A PARTIR DEL TOQUE DEL BOTON)
void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&TimHandle); 
	counter++;
	MPU6050_Get_Accel_Scale(&myAccelScaled);
	LeerADC();

}
void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); //RESETEAR LA BANDERA
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	
	MPU6050_Init(&hi2c1);
	count++;
	TIM2_IRQHandler();
  	
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();

}

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



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;

  
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;
  
  
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}





static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif