/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "stdlib.h"
#include "adc.h"
#include "math.h"
#include "dac.h"
#include "tim.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// tension du signal en sortie du filtre passe haut
#define tension_cc 5 // tension de la composante continue en entrée du banc
#define tension_max 690 // tension crete du signal en entrée du banc
#define facteur_corection_res 812 // en 1e-3, correction du resultat du calcul de la resistance
#define facteur_corection_cond 1222 // en 1e-3, correction du resultat du calcul de la capacite
//resistance utilise : 3300 Ohm, 10k Ohm, 33 kOhm, 100 kOhm

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
ADC_ChannelConfTypeDef sConfig = {0};
int stat_mesure = 0; // -1 : sous-régime, 0 : regime etablie, 1 : sur-régime
int type_mesure = 0; // = 0 pour resistance, = 1 pour condensateur
int out_boundarie = 0; // booléen permettant de savoir si le composant à mesurer est en dehors de la plage de capture
uint16_t sinus12bit_360[360]; // tableau pour sinusoïde à 360 echantillon (basse fréquence)
uint16_t sinus12bit_180[180]; // tableau pour sinusoïde à 180 echantillon
uint16_t sinus12bit_90[90]; // tableau pour sinusoïde à 90 echantillon
uint16_t sinus12bit_45[45]; // tableau pour sinusoïde à 45 echantillon  (haute fréquence)
uint32_t freq_reel = 50; // frequence initial de 50 Hz
uint32_t res_reel = 3300; // resistance de mesure initial de 3.3 kOhm
uint16_t tension_mes = 1; // tension mesurer de base non nul afin d'eviter de possible singularité

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId affichageHandle;
osThreadId Mesure_crete_1Handle;
osThreadId Choix_res_condHandle;
osThreadId modif_freq_resHandle;
osThreadId ges_freq_resHandle;
osThreadId calcul_valHandle;
osMessageQId Send_aff_vcc1Handle;
osMessageQId Send_aff_vcc2Handle;
osMessageQId Queue_freqHandle;
osMessageQId Queue_resHandle;
osMessageQId Queue_cond_estHandle;
osMessageQId Queue_res_estHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Show_Value(void const * argument);
void Mesure_crete_crete_1(void const * argument);
void Choix_mesure(void const * argument);
void Modif_freq_res(void const * argument);
void Ges_freq_res(void const * argument);
void Calcul_val(void const * argument);

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
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	  int i;
	  for(i=0; i<45;i++)	{
		  sinus12bit_360[i] = 2048 + 1024*sin(i*3.14159/180);
		  sinus12bit_180[i] = 2048 + 1024*sin(i*3.14159/90);
		  sinus12bit_90[i] = 2048 + 1024*sin(i*3.14159/45);
		  sinus12bit_45[i] = 2048 + 1024*sin(2*i*3.14159/45);
	  }
	  for(i=45; i<90;i++)	{
		  sinus12bit_360[i] = 2048 + 1024*sin(i*3.14159/180);
		  sinus12bit_180[i] = 2048 + 1024*sin(i*3.14159/90);
		  sinus12bit_90[i] = 2048 + 1024*sin(i*3.14159/45);
	    }
	  for(i=90; i<180;i++)	{
		  sinus12bit_360[i] = 2048 + 1024*sin(i*3.14159/180);
		  sinus12bit_180[i] = 2048 + 1024*sin(i*3.14159/90);
	    }
	  for(i=180; i<360;i++)	{
		  sinus12bit_360[i] = 2048 + 1024*sin(i*3.14159/180);
	    }

	  if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) sinus12bit_360, 360,
			  DAC_ALIGN_12B_R) != HAL_OK) {
		  Error_Handler();
	  }

	  if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) {
		  Error_Handler();
	  }

	  // On construit l'ensemble des tableau et on démarre le DMA avec le signal de fréquence maximum, soit
	  // de 5 kHz

	  HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, 0);
	  HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, 0);

	  // Les LEDs 13 et 14 permettent de savoir quel résistance on sélectionne sur le multiplexeur

	  //	Led 13		|	0		|	1		|	0		|	1		| ( 0 = éteint, 1 = allumé )
	  //	Led 14		|	0		|	0		|	1		|	1		|
	  //	Resistance	|   3.3k	|	10k		|	33k		|	100k	| (k pour kOhm)


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
  /* definition and creation of Send_aff_vcc1 */
  osMessageQDef(Send_aff_vcc1, 2, uint16_t);
  Send_aff_vcc1Handle = osMessageCreate(osMessageQ(Send_aff_vcc1), NULL);

  /* definition and creation of Send_aff_vcc2 */
  osMessageQDef(Send_aff_vcc2, 2, uint16_t);
  Send_aff_vcc2Handle = osMessageCreate(osMessageQ(Send_aff_vcc2), NULL);

  /* definition and creation of Queue_freq */
  osMessageQDef(Queue_freq, 2, uint16_t);
  Queue_freqHandle = osMessageCreate(osMessageQ(Queue_freq), NULL);

  /* definition and creation of Queue_res */
  osMessageQDef(Queue_res, 2, uint16_t);
  Queue_resHandle = osMessageCreate(osMessageQ(Queue_res), NULL);

  /* definition and creation of Queue_cond_est */
  osMessageQDef(Queue_cond_est, 2, uint32_t);
  Queue_cond_estHandle = osMessageCreate(osMessageQ(Queue_cond_est), NULL);

  /* definition and creation of Queue_res_est */
  osMessageQDef(Queue_res_est, 2, uint32_t);
  Queue_res_estHandle = osMessageCreate(osMessageQ(Queue_res_est), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of affichage */
  osThreadDef(affichage, Show_Value, osPriorityNormal, 0, 1024);
  affichageHandle = osThreadCreate(osThread(affichage), NULL);

  /* definition and creation of Mesure_crete_1 */
  osThreadDef(Mesure_crete_1, Mesure_crete_crete_1, osPriorityNormal, 0, 128);
  Mesure_crete_1Handle = osThreadCreate(osThread(Mesure_crete_1), NULL);

  /* definition and creation of Choix_res_cond */
  osThreadDef(Choix_res_cond, Choix_mesure, osPriorityNormal, 0, 128);
  Choix_res_condHandle = osThreadCreate(osThread(Choix_res_cond), NULL);

  /* definition and creation of modif_freq_res */
  osThreadDef(modif_freq_res, Modif_freq_res, osPriorityNormal, 0, 128);
  modif_freq_resHandle = osThreadCreate(osThread(modif_freq_res), NULL);

  /* definition and creation of ges_freq_res */
  osThreadDef(ges_freq_res, Ges_freq_res, osPriorityNormal, 0, 128);
  ges_freq_resHandle = osThreadCreate(osThread(ges_freq_res), NULL);

  /* definition and creation of calcul_val */
  osThreadDef(calcul_val, Calcul_val, osPriorityNormal, 0, 128);
  calcul_valHandle = osThreadCreate(osThread(calcul_val), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Show_Value */
/**
* @brief Function implementing the affichage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Show_Value */
void Show_Value(void const * argument)
{
  /* USER CODE BEGIN Show_Value */
  // cette tâche à pour but d'afficher à l'écran la tension crete mesurer au borne du composant à mersurer
  char text[50]={};
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodMs = 300;
  uint32_t res_est; // Variable contenant l'estimation de la resistance
  uint32_t cond_est; // Variable contenant l'estimation du condensateur
  int wait=0; // variable d'attente, voir plus loin

  /* Infinite loop */
  for(;;)
  {
	  // Affichage du type de mesure en premier
	  if (type_mesure) {
		  sprintf(text," Mesure Condensateur");
		  BSP_LCD_DisplayStringAtLine(2,(uint8_t*) text);
	  }
	  else {
		  sprintf(text," Mesure Resistance  ");
		  BSP_LCD_DisplayStringAtLine(2,(uint8_t*) text);
	  }

	  // Affichage de la valeur image de la tension au borne de l'impédance à mesurer
	  if(xQueueReceive(Send_aff_vcc1Handle,&tension_mes,0)) {
		  sprintf(text," Tension crete bras = %04u mV",tension_mes);
		  BSP_LCD_DisplayStringAtLine(4,(uint8_t*) text);
	  }

	  // affichage de l'impédance si le régime est établie
	  if (stat_mesure==0) {
		  if(xQueueReceive(Queue_res_estHandle,&res_est,50)) {
			  sprintf(text," Possible resistance estimee = %012lu Ohm",res_est);
			  BSP_LCD_DisplayStringAtLine(8,(uint8_t*) text);
		  }
		  if(xQueueReceive(Queue_cond_estHandle,&cond_est,50)) {
			  sprintf(text," Possible condensateur estimee = %012lu pF",cond_est);
			  BSP_LCD_DisplayStringAtLine(10,(uint8_t*) text);
		  }
		  // On supprime les lignes affichant les pseudo-mesures du type d'impédance que l'on ne mesure pas,
		  // et on supprime aussi le message d'attente
		  BSP_LCD_ClearStringLine(6);
		  if (type_mesure) {
			  BSP_LCD_ClearStringLine(8);
		  }
		  else {
			  BSP_LCD_ClearStringLine(10);
		  }

	  }
	  else {
		  if (out_boundarie==0) {
			  // defilement de point expliquant que le système modifie la fréquence ou la résistance afin de trouver
			  // le bon calibre à adopter pour mesurer l'impédance.
			  switch (wait) {
				  case 0 :
					  sprintf(text," Transition frequence-resistance     ");
					  BSP_LCD_DisplayStringAtLine(6,(uint8_t*) text);
					  wait++;
					  break;
				  case 1 :
					  sprintf(text," Transition frequence-resistance .   ");
					  BSP_LCD_DisplayStringAtLine(6,(uint8_t*) text);
					  wait++;
					  break;
				  case 2 :
					  sprintf(text," Transition frequence-resistance ..  ");
					  BSP_LCD_DisplayStringAtLine(6,(uint8_t*) text);
					  wait++;
					  break;
				  case 3 :
					  sprintf(text," Transition frequence-resistance ... ");
					  BSP_LCD_DisplayStringAtLine(6,(uint8_t*) text);
					  wait++;
					  break;
				  case 4 :
					  sprintf(text," Transition frequence-resistance ....");
					  BSP_LCD_DisplayStringAtLine(6,(uint8_t*) text);
					  wait = 0;
					  break;
				  default :
					  sprintf(text," Transition frequence-resistance ....");
					  BSP_LCD_DisplayStringAtLine(6,(uint8_t*) text);
					  wait = 0;
					  break;
			  }
		  }
		  else {
			  // La résistance ou le condensateur est trop faible ou trop élevé
			  sprintf(text," En dehors de la capacite de mesure  ");
			  BSP_LCD_DisplayStringAtLine(6,(uint8_t*) text);
		  }
		  BSP_LCD_ClearStringLine(8);
		  BSP_LCD_ClearStringLine(10);
	  }

	  vTaskDelayUntil(&xLastWakeTime, xPeriodMs);

  }
  /* USER CODE END Show_Value */
}

/* USER CODE BEGIN Header_Mesure_crete_crete_1 */
/**
* @brief Function implementing the Mesure_crete_1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Mesure_crete_crete_1 */
void Mesure_crete_crete_1(void const * argument)
{
  /* USER CODE BEGIN Mesure_crete_crete_1 */
  // Cette tâche à pour but de récuperer la tension sur l'ADC1 et de déterminer si le système est en sur-régime,
  // sous-régime ou régime établie.
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfig.Channel = ADC_CHANNEL_8;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);
  // On démarre le ADC pour capturer la tension crete au borne de l'impédance

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodMs = 100;
  uint16_t Mesure=0; // variable qui contiendra la capture de l'ADC1


  /* Infinite loop */
  for(;;)
  {
	  // Capture de la tension sur l'ADC1
	  HAL_ADC_Start(&hadc3);
	  HAL_ADC_PollForConversion(&hadc3, 100);
	  Mesure = (uint16_t)HAL_ADC_GetValue(&hadc3);

	  // Détermination du type de régime. Les termes 2/10 et 8/10 ont été établie a posteriori d'un ensemble de mesure: ils
	  // permettent de trouver un compromis entre le nombre de résistance et de fréquence à utiliser pour obtenir
	  // une mesure assez précise
	  if (Mesure < (tension_max-tension_cc)*2/10) { //
		  stat_mesure = -1; // sous-regime
	  }
	  else if (Mesure > (tension_max-tension_cc)*8/10) { //
		  stat_mesure = 1; // sur-regime
	  }
	  else {
		  stat_mesure = 0; // régime établie
	  }

	  // On envoie la variable mesure à la tache Show_value pour affichage sur le LCD
	  xQueueSend(Send_aff_vcc1Handle,&Mesure,10);

	  vTaskDelayUntil(&xLastWakeTime, xPeriodMs);
  }
  /* USER CODE END Mesure_crete_crete_1 */
}

/* USER CODE BEGIN Header_Choix_mesure */
/**
* @brief Function implementing the Choix_res_cond thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Choix_mesure */
void Choix_mesure(void const * argument)
{
  /* USER CODE BEGIN Choix_mesure */
  // Cette tache à pour but de permettre à l'utilisateur de mesurer soit une résistance, soit un condensateur.
	// La selection se fait en cliquant soit à gauche de l'écran, soit à droite.
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodMs = 100;
  static TS_StateTypeDef  TS_State;
  /* Infinite loop */
  for(;;)
  {
	  BSP_TS_GetState(&TS_State);
	  if(TS_State.touchDetected){
		  if (TS_State.touchX[0]<200) {
			  type_mesure=1;
		  }
		  else {
			  type_mesure=0;
		  }

	  }
	  vTaskDelayUntil(&xLastWakeTime, xPeriodMs);
  }
  /* USER CODE END Choix_mesure */
}

/* USER CODE BEGIN Header_Modif_freq_res */
/**
* @brief Function implementing the modif_freq_res thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Modif_freq_res */
void Modif_freq_res(void const * argument)
{
  /* USER CODE BEGIN Modif_freq_res */
	// Cette tâche à pour but de modifier le choix de la fréquence ou le choix de la résistance en fonction de ce que
	// choisie la tache Ges_freq_res. Cette tache doit être rapide mais pas trop afin d'éviter de faire bugger le DMA.
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodMs = 400;
  uint16_t freq = 0; // indice de la fréquence, qui peut valoir 0,1,2,3,4 ou 5.
  uint16_t res = 0; // indice de la résistance, qui peut valoir 0,1,2 ou 3.
  HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, 0); // Les LEDs 13 et 14 sont éteintes afin de dire que la résistance
  HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, 0); // au début est la plus faible.

  uint16_t* p_tab=sinus12bit_360; // on récuperer le pointeur d'un des tableau afin de modifier la variable à mettre
  	  // dans le DMA sur une seul ligne.
  int N=360; // variable donnant le nombre d'échantillon dans le tableau: utile pour le DMA.

  /* Infinite loop */
  for(;;)
  {
	  if (xQueueReceive(Queue_freqHandle,&freq,0)) {
		  // L'avantage d'utiliser une file d'attente est de permettre d'empecher de relancer constament de DMA.
		  // l'obtention des counter periods du timer 7 et le choix des tableau permettant d'obtenir une fréquence
		  // bien précise à été obtenue grace à la formule :
		  // 	frequence = 150 Mhz / (Counter_Period*Nombre_echantillon_tableau)
		  // Il faut noter que Counter_Period > 444 et qu'il faut maximiser Nombre_echantillon_tableau afin d'obtenir
		  // un signal "bien" sinusoïdale.
		  switch (freq) {
			  case 0:
				htim7.Init.Period = 5555;
				p_tab = sinus12bit_360;
				N=360;
				freq_reel = 50; // frequence de 50 Hz
				break;
		  	  case 1:
				htim7.Init.Period = 1111;
				p_tab = sinus12bit_360;
				N=360;
				freq_reel = 250; // frequence de 250 Hz
				break;
		  	  case 2:
		  		htim7.Init.Period = 555;
		  		p_tab = sinus12bit_360;
				N=360;
				freq_reel = 500; // frequence de 500 Hz
				break;
		  	  case 3:
		  		htim7.Init.Period = 555;
		  		p_tab = sinus12bit_180;
				N=180;
				freq_reel = 1000; // frequence de 1 kHz
				break;
		  	  case 4:
		  		htim7.Init.Period = 444;
		  		p_tab = sinus12bit_90;
				N=90;
				freq_reel = 2500; // frequence de 2.5 kHz
				break;
		  	  case 5:
		  		htim7.Init.Period = 444;
		  		p_tab = sinus12bit_45;
				N=45;
				freq_reel = 5000; // frequence de 5 kHz
				break;
		  	  default:
		  		htim7.Init.Period = 5555;
		  		p_tab = sinus12bit_360;
				N=360;
				freq_reel = 50; // frequence de 50 Hz
		  }

		  if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
			  Error_Handler(); // Changement du Counter Period du Timer 7
		  }
		  if (HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1) != HAL_OK) {
		  	Error_Handler(); // Arret du DMA
		  		  }
		  if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) p_tab, N, DAC_ALIGN_12B_R) != HAL_OK) {
			  Error_Handler(); // Redémarrage du DMA avec la nouvelle fréquence
		  }
	  }
	  if (xQueueReceive(Queue_resHandle,&res,0)) {
		  // Une fois plus, la fil d'attente permet de ne pas modifier constamment le choix de la résistance de calibrage.
		  switch (res) {
		  	  case 0:
				  HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, 0);
				  HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, 0);
				  res_reel=3300; // resistance de calibrage à 3.3 kOhm
		  		  break;
		  	  case 1:
				  HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, 1);
				  HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, 0);
				  res_reel=10000; // resistance de calibrage à 10 kOhm
		  		  break;
		  	  case 2:
				  HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, 0);
				  HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, 1);
				  res_reel=33000; // resistance de calibrage à 33 kOhm
		  		  break;
		  	  case 3:
				  HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, 1);
				  HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, 1);
				  res_reel=100000; // resistance de calibrage à 100 kOhm
		  		  break;
		  	  default:
				  HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, 0);
				  HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, 0);
				  res_reel=3300; // resistance de calibrage à 3.3 kOhm
		  }
	  }
	  vTaskDelayUntil(&xLastWakeTime, xPeriodMs);
  }
  /* USER CODE END Modif_freq_res */
}

/* USER CODE BEGIN Header_Ges_freq_res */
/**
* @brief Function implementing the ges_freq_res thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ges_freq_res */
void Ges_freq_res(void const * argument)
{
  /* USER CODE BEGIN Ges_freq_res */
	// Cette tache permet de choisir la resistance dans le multiplexeur/demultiplexeur et la
	// frequence selon le type de régime du système. Il faut noter que la période de cette tâche
	// doit être suffisament lent afin de permettre un changement de de fréquence/résistance trop rapide.

	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xPeriodMs = 2000;

	uint16_t freq = 0; // indice de fréquence future
	uint16_t res = 0; // indice de résistance future
	uint16_t freq_current = 0; // indice de fréquence actuelle
	uint16_t res_current = 0; // indice de résistance actuelle

	/* Infinite loop */
	for(;;)
	{

	  if (type_mesure == 0) { //pour mesure R
		  freq = 5; // meilleur precision en haute fréquence car détecteur de crête moins sujet à
		  //	des atténuations sévères.
		  if (stat_mesure==1) {
			  HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, 1);
			  HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, 0);
			  // Les LEDs 11 et 12 permettent de prendre en compte de l'état du système:
			  //	LED 11	|	1			|	0			|	0				|
			  //	LED 12	|	0			|	1			|	0				|
			  //	Système	|	Sur-régime	| Sous-régime	| Régime-établie	|
			  //
			  if (res_current<3) {
			  	  res++; // On augmente la résistance de calibrage si on est en sur-régime, ce qui permet
			  	  //de diminiuer la tension au borne de la résistance à mesurer
			  }
		  }
		  else if (stat_mesure==-1) {
			  HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, 0);
			  HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, 1);
			  if (res_current>0) {
			  	  res--; // sinon on diminue la résistance de calibrage afin  d'augmenter la tension au borne de la
			  	  // résistance de mesure
			  }
		  }
		  else {
		  		  HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, 0);
		  		  HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, 0);
		  	  }
	  }

	  //


	  else if (type_mesure == 1) { //pour mesure C
		  if (stat_mesure==1) {
			  HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, 1);
			  HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, 0);
			  if (res_current<3) {
				  res++; // on augmente d'abord la résistance de calibrage à fréquence fixe, afin de
				  // diminuer la tension au borne du condensateur.
			  }
			  else {
				  if (freq_current<5) {
					  // Si jamais la mise en place de la plus forte résistance n'a pas permis de changer de régime,
					  // alors on augmente la fréquence afin de laisser moins de temps au condensateur de se charger,
					  // ce qui va donc diminuer sa tension crête à ses bornes. On remet la résistance à 0 afin de
					  // recommencer la mesure du type de régime
					  freq++;
					  res = 0;
				  }
			  }
		  }
		  else if (stat_mesure==-1) { // même chose, mais dans l'autre sens.
			  HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, 0);
			  HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, 1);
			  if (res_current>0) {
				  res--;
			  }
			  else {
				  if (freq_current>0) {
					  freq--;
					  res = 3;
				  }
			  }
		  }
		  else {
		  		  HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, 0);
		  		  HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, 0);
		  	  }
	  }
	  else {
		  HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, 0);
		  HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, 0);
	  }

	  //

	  if (freq_current!=freq) {
		  xQueueSend(Queue_freqHandle,&freq,10); // si la fréquence doit être modifier, alors on envoie la nouvelle fréquence
		  // à la tache Modif_freq_res afin qu'elle effectue le changement.
	  }
	  if (res_current!=res) {
		  xQueueSend(Queue_resHandle,&res,10); // idem pour la résistance.
	  }

	  //

	  freq_current=freq;
	  res_current=res; // mise à jour des état de l'indice de la résistance et de la fréquence

	  //

	  if (((res_current==0) &&(freq_current==0)&&(stat_mesure))||((res_current==3) &&(freq_current==5)&&(stat_mesure))) {
		  out_boundarie = 1;
		  // si on a atteint la fréquence maximale et la résistance de calibrage maximal,
		  // et que l'on est toujours en sur-régime, alors on est en dehors de la plage de calibration.
		  // Inversement, si on est à la fréquence et résistance minimal, et que le système est en sous-régime,
		  // alors on est également en dehors de la plage de calibration.
	  }
	  else {
		  out_boundarie = 0;
	  }

	  vTaskDelayUntil(&xLastWakeTime, xPeriodMs);
	}
  /* USER CODE END Ges_freq_res */
}

/* USER CODE BEGIN Header_Calcul_val */
/**
* @brief Function implementing the calcul_val thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Calcul_val */
void Calcul_val(void const * argument)
{
  /* USER CODE BEGIN Calcul_val */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodMs = 1000;

  uint32_t res_est = 0; // variable contenant la valeur de resistance estimée, en Ohm
  uint32_t cond_est = 0; // varaible contenant la valuer de resistance estimée, en pF

  /* Infinite loop */
  for(;;)
  {
	  if (stat_mesure==0) {
		  if (type_mesure==0) {
			  res_est = (uint32_t)((((float)(facteur_corection_res))/1000*(float)res_reel)/(((float)(tension_max-tension_cc))/((float)(tension_mes+1))-1.0+0.01));
			  xQueueSend(Queue_res_estHandle,&res_est,10);
		  }
		  else if (type_mesure==1){
			  cond_est = (uint32_t)(facteur_corection_cond*1000000000.0/(2*3.1415*(float)res_reel*(float)freq_reel)*sqrt(
					  (((float)(tension_max-tension_cc)))/(((float)(tension_mes+1)))*(
							  ((float)(tension_max-tension_cc))/((float)(tension_mes+1)))-1)	);
			  xQueueSend(Queue_cond_estHandle,&cond_est,10);
		  }
		  // Se référer au schéma du montage pour comprendre d'ou viennent les formules.
		  // Les facteurs de correction proviennent d'une comparaison entre les mesures de différentes résistances
		  // et condensateurs réalisés avec une machine permettant de d'obtenir des valeurs très précise des composants,
		  // et celles obtenues avec le banc d'essai. (voir Correction_V2.ods)
	  }
	  vTaskDelayUntil(&xLastWakeTime, xPeriodMs);
  }
  /* USER CODE END Calcul_val */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

