/******************************************************************************
*   AnalogInput.c
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   The Analog Input module of the Formula SAE Datalogger provides public
*   functions to set the power supply (5V/12V) provided to the 4 analog
*   sensors as well as the required input conditioning MUX selection. The
*   supplies are rated to supply a maximum of 20mA a either 5V or 12V per
*   sensor. The analog signals will be measured via ADC1 channels, with PIT1
*   providing hardware triggering at 8kHz.
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/04/2019
*
*   Created on: 04/29/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "AnalogInput.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/******************************************************************************
*   Private Function Prototypes
******************************************************************************/
static void anlginCalibrateADC1(void);

static void anlginSamplerTask(void *);

/******************************************************************************
*   Private Definitions
******************************************************************************/
#define ENABLE                      0x01U
#define SET                         0x01U
#define ALT_1_GPIO                  0x01U
#define OUTPUT                      0x01U
#define BIT_0_MASK        ((uint8_t)0x01U)
#define BYTE_0_MASK      ((uint32_t)0xFFU)
#define PIT1                        0x01U
#define PIT_8KHZ_LDVAL         749999999U
#define PIT1_TRG                    0x05U
#define BUS_CLK                     0x00U
#define MODE_12BIT                  0x01U
#define LONG_SAMPLES                0x01U
#define CLK_DIV_8                   0x03U
#define B_CHNLS                     0x01U
#define AVG_32_SAMP                 0x03U
#define HW_TRG                      0x01U
#define SINGLE_ENDED                0x00U

/* Task parameters */
#define ANLGINSAMPLERTASK_PRIORITY     4U
#define ANLGINSAMPLERTASK_STKSIZE    256U

/* Register Flags */
#define COCO_FLAG ((ADC1->SC1[0] & ADC_SC1_COCO_MASK) >> ADC_SC1_COCO_SHIFT)
#define CALI_FAIL_FLAG ((ADC1->SC3 & ADC_SC3_CALF_MASK) >> ADC_SC3_CALF_SHIFT)

/* AIN Power pin numbers, all PORTB */
#define AIN1_POWER_PIN_NUM            23U
#define AIN2_POWER_PIN_NUM            22U
#define AIN3_POWER_PIN_NUM            21U
#define AIN4_POWER_PIN_NUM            20U

/* AIN Conditioning pin numbers, all PORTC */
#define AIN1_COND_PIN_NUM             13U
#define AIN2_COND_PIN_NUM             12U
#define AIN3_COND_PIN_NUM             15U
#define AIN4_COND_PIN_NUM             14U

/* Bit # corresponding to certain AIN in 8-bit state/power field message */
#define AIN1_POWER_BIT_NUM   ((uint8_t)0U)
#define AIN2_POWER_BIT_NUM   ((uint8_t)1U)
#define AIN3_POWER_BIT_NUM   ((uint8_t)2U)
#define AIN4_POWER_BIT_NUM   ((uint8_t)3U)
#define AIN1_STATE_BIT_NUM   ((uint8_t)4U)
#define AIN2_STATE_BIT_NUM   ((uint8_t)5U)
#define AIN3_STATE_BIT_NUM   ((uint8_t)6U)
#define AIN4_STATE_BIT_NUM   ((uint8_t)7U)

/* Bit offset for each AIN sampling rate position in SR message */
#define AIN1_SR_OFFSET      ((uint32_t)0U)
#define AIN2_SR_OFFSET      ((uint32_t)8U)
#define AIN3_SR_OFFSET     ((uint32_t)16U)
#define AIN4_SR_OFFSET     ((uint32_t)24U)

/* ADC1 channels for each AIN*/
#define AIN1_SIG_CHNL                  5U
#define AIN2_SIG_CHNL                  4U
#define AIN3_SIG_CHNL                  7U
#define AIN4_SIG_CHNL                  6U

/* Structure definition for Analog Input data and configurations */
typedef struct AnlgInData_t
{
    uint8_t power_state_field;
    uint8_t ain1_samp_rate;
    uint8_t ain2_samp_rate;
    uint8_t ain3_samp_rate;
    uint8_t ain4_samp_rate;
    uint16_t ain1_data;
    uint16_t ain2_data;
    uint16_t ain3_data;
    uint16_t ain4_data;
} AnlgInData_t;

/******************************************************************************
*   Private Macros
******************************************************************************/
/* Macros accepting FIVE_VOLTS/TWELVE_VOLTS (0/1) to either set or clear AIN
 * power pin */
#define AIN1_POWER_SET(x) (GPIOB->PDOR |= ((x) << AIN1_POWER_PIN_NUM))
#define AIN2_POWER_SET(x) (GPIOB->PDOR |= ((x) << AIN2_POWER_PIN_NUM))
#define AIN3_POWER_SET(x) (GPIOB->PDOR |= ((x) << AIN3_POWER_PIN_NUM))
#define AIN4_POWER_SET(x) (GPIOB->PDOR |= ((x) << AIN4_POWER_PIN_NUM))

/* Macros accepting TWELVE_VOLTS/FIVE_VOLTS (0/1) to either set or clear AIN
 * conditioning pin. */
#define AIN1_COND_SET(x) (GPIOC->PDOR |= ((x) << AIN1_COND_PIN_NUM))
#define AIN2_COND_SET(x) (GPIOC->PDOR |= ((x) << AIN2_COND_PIN_NUM))
#define AIN3_COND_SET(x) (GPIOC->PDOR |= ((x) << AIN3_COND_PIN_NUM))
#define AIN4_COND_SET(x) (GPIOC->PDOR |= ((x) << AIN4_COND_PIN_NUM))

/* Macros accepting 8 bit field to find and isolate AIN's power bit and
 * shift over to bit 0 position for relevant SET function */
#define AIN1_POWER_BIT(x) (((x) >> AIN1_POWER_BIT_NUM) & BIT_0_MASK)
#define AIN2_POWER_BIT(x) (((x) >> AIN2_POWER_BIT_NUM) & BIT_0_MASK)
#define AIN3_POWER_BIT(x) (((x) >> AIN3_POWER_BIT_NUM) & BIT_0_MASK)
#define AIN4_POWER_BIT(x) (((x) >> AIN4_POWER_BIT_NUM) & BIT_0_MASK)

/* Macros accepting 8 bit field to find and isolate AIN's conditioning bit and
 * shift over to bit 0 position for relevant SET function.
 * NOTE: Power and conditioning set macros use the same bit for each AIN, but
 * since a '0' corresponds to a FIVE_VOLTS power SET but a '0' corresponds to a
 * TWELVE_VOLTS conditioning set, the conditioning SET macro must receive the
 * inverse of this bit so both SET macros are FIVE_VOLTS or TWELVE_VOLTS */
#define AIN1_COND_BIT(x) (AIN1_POWER_BIT(~x))
#define AIN2_COND_BIT(x) (AIN2_POWER_BIT(~x))
#define AIN3_COND_BIT(x) (AIN3_POWER_BIT(~x))
#define AIN4_COND_BIT(x) (AIN4_POWER_BIT(~x))

/* Macros accepting 32 bit field to find and isolate AIN's sampling rate byte
 * and shift over to bits [0:7], then casting to individual 8-bit value */
#define AIN1_SAMP_RATE(x) ((uint8_t)(((x) >> AIN1_SR_OFFSET) & BYTE_0_MASK))
#define AIN2_SAMP_RATE(x) ((uint8_t)(((x) >> AIN2_SR_OFFSET) & BYTE_0_MASK))
#define AIN3_SAMP_RATE(x) ((uint8_t)(((x) >> AIN3_SR_OFFSET) & BYTE_0_MASK))
#define AIN4_SAMP_RATE(x) ((uint8_t)(((x) >> AIN4_SR_OFFSET) & BYTE_0_MASK))

/******************************************************************************
*   Private Variables
******************************************************************************/
/* Data structure that will hold all Analog Input data and configurations.
 * Secured by Mutex "anlginCurrentDataKey" */
static AnlgInData_t anlginCurrentData;

/* Mutex key that will protect anlginCurrentData structure. Must be pended on
 * if writing/reading anlginCurrentData is desired to ensure synchronization */
static SemaphoreHandle_t anlginCurrentDataKey;

/* Task handle for Analog In sampler task, used by ADC1_ISRHandler() to post
 * task notification */
static TaskHandle_t anlginSamplerTaskHandle = NULL;

/******************************************************************************
*   AnlgInInit() - Public function to configure all GPIO used as power and
*   conditioning selects as outputs. Initializes Analog In data structure and
*   creates Mutex to protect data structure. Also initializes PIT1 to trigger
*   ADC1 at 8kHz. Configures and calibrates ADC1.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void AnlgInInit()
{
    BaseType_t task_create_return;

    SIM->SCGC5 |= SIM_SCGC5_PORTB(ENABLE);
    SIM->SCGC5 |= SIM_SCGC5_PORTC(ENABLE);

    PORTB->PCR[AIN1_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTB->PCR[AIN2_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTB->PCR[AIN3_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTB->PCR[AIN4_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);

    PORTC->PCR[AIN1_COND_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTC->PCR[AIN2_COND_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTC->PCR[AIN3_COND_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTC->PCR[AIN4_COND_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);

    PORTC->PCR[9] = PORT_PCR_MUX(0);
    PORTC->PCR[8] = PORT_PCR_MUX(0);
    PORTC->PCR[10] = PORT_PCR_MUX(0);
    PORTC->PCR[11] = PORT_PCR_MUX(0);

    GPIOB->PDDR |= GPIO_PDDR_PDD((OUTPUT << AIN1_POWER_PIN_NUM) |
                                 (OUTPUT << AIN2_POWER_PIN_NUM) |
                                 (OUTPUT << AIN3_POWER_PIN_NUM) |
                                 (OUTPUT << AIN4_POWER_PIN_NUM));

    GPIOC->PDDR |= GPIO_PDDR_PDD((OUTPUT << AIN1_COND_PIN_NUM) |
                                 (OUTPUT << AIN2_COND_PIN_NUM) |
                                 (OUTPUT << AIN3_COND_PIN_NUM) |
                                 (OUTPUT << AIN4_COND_PIN_NUM));

    /* Initial configuration: 5V input conditioning and 5V power supply */
    anlginCurrentData.power_state_field = (uint8_t)0x00;
    anlginCurrentData.ain1_data = (uint16_t)0x0000;
    anlginCurrentData.ain2_data = (uint16_t)0x0000;
    anlginCurrentData.ain3_data = (uint16_t)0x0000;
    anlginCurrentData.ain4_data = (uint16_t)0x0000;

    /* Create Analog In current data structure mutex */
    anlginCurrentDataKey = xSemaphoreCreateMutex();
    while(anlginCurrentDataKey == NULL){ /* Error trap */ }

    /* Create Analog In sampler task */
    task_create_return = xTaskCreate(anlginSamplerTask,
                                     "Analog In Sampler Task",
                                     ANLGINSAMPLERTASK_STKSIZE,
                                     NULL,
                                     ANLGINSAMPLERTASK_PRIORITY,
                                     &anlginSamplerTaskHandle);

    while(task_create_return == pdFAIL){ /* Error trap */ }



    /* PIT1 initialization for 125us period, 8kHz trigger frequency */
    SIM->SCGC6 |= SIM_SCGC6_PIT(ENABLE);
    PIT->MCR &= ~PIT_MCR_MDIS(ENABLE);
    PIT->CHANNEL[PIT1].LDVAL = PIT_8KHZ_LDVAL;
    PIT->CHANNEL[PIT1].TCTRL |= PIT_TCTRL_TEN(ENABLE);

    /* ADC1 initialization for PIT1 triggering, 60MHz/8 = 7.5MHz ADCK, 12 bit
     * conversions, long sampling, ADxxB channels mux, HW triggering, enabled
     * COCO interrupts, single ended signals, and AIN1 as the initial channel.
     * ADC1 is calibrated in the middle of initialization as to achieve
     * specified accuracy (must be done before any conversions are started). */
    SIM->SCGC3 |= SIM_SCGC3_ADC1(ENABLE);

    SIM->SOPT7 = ((SIM->SOPT7 & ~SIM_SOPT7_ADC1TRGSEL_MASK) |
                   SIM_SOPT7_ADC1TRGSEL(PIT1_TRG) |
                   SIM_SOPT7_ADC1ALTTRGEN(ENABLE));

    ADC1->CFG1 = ADC_CFG1_ADICLK(BUS_CLK) | ADC_CFG1_MODE(MODE_12BIT) |
                 ADC_CFG1_ADLSMP(LONG_SAMPLES) | ADC_CFG1_ADIV(CLK_DIV_8);

    ADC1->CFG2 = ADC_CFG2_MUXSEL(B_CHNLS);
//    anlginCalibrateADC1();
    ADC1->SC2 = ADC_SC2_ADTRG(HW_TRG);

    ADC1->SC1[0] = ADC_SC1_AIEN(ENABLE) | ADC_SC1_DIFF(SINGLE_ENDED) |
                   ADC_SC1_ADCH(AIN1_SIG_CHNL);

    /* Enable ADC1 interrupts (for COCO ISR), with priority change for use
     * with FreeRTOS */
    NVIC_SetPriority(ADC1_IRQn, 2U);
    NVIC_ClearPendingIRQ(ADC1_IRQn);
    NVIC_EnableIRQ(ADC1_IRQn);
}

/******************************************************************************
*   anlginSamplerTask() -
*
*   Parameters:.
*
*   Return: None
******************************************************************************/
static void anlginSamplerTask(void *pvParameters)
{
//    const TickType_t NOTIF_BLOCK_TIME = pdMS_TO_TICKS( 2000UL );
//    const TickType_t MUTEX_BLOCK_TIME = pdMS_TO_TICKS( 1UL );
    uint32_t notify_count;
    BaseType_t take_return;

    while(1)
    {
        notify_count = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

        while(notify_count == 0){ /*Error trap, task notify timed out */ }

        take_return = xSemaphoreTake(anlginCurrentDataKey, portMAX_DELAY);

        while(take_return == pdFAIL){ /* Error trap, Mutex key never available */ }

        anlginCurrentData.ain1_data = ((uint16_t)(ADC1->R[0]));

        xSemaphoreGive(anlginCurrentDataKey);
    }
}


/******************************************************************************
*   AnlgInSet() - Public function to set power and conditioning pins to achieve
*   requested results from received user message. Saves the new Analog In data
*   from message in Mutex protected private data structure.
*
*   Parameters:
*
*       AnlgInMsg_t msg - Message structure received from telemetry unit
*       with 8 bit power and state field (msg.power_state_field) and 32 bit
*       sampling rates field (msg.sampling_rate_field).
*
*       msg.power_state_field[0:3] corresponds to AIN[1:4] power, with a bit of
*       0 representing a power output of 5V (1 = 12V).
*
*       msg.power_state_field[4:7] corresponds to AIN[1:4] state, with a bit of
*       0 representing a non-active sensor input (1 = active).
*
*       msg.sampling_rate_field[0:7] corresponds to AIN1 sampling rate, and
*       msg.sampling_rate_field[8:15] corresponds to AIN2 sampling rate, etc.
*
*   Return: None
******************************************************************************/
void AnlgInSet(AnlgInMsg_t msg)
{
    BaseType_t take_return;
    /* Convert state/power message to individual conditioning/power/state bits
     * for each AIN */
    AIN1_POWER_SET(AIN1_POWER_BIT(msg.power_state_field));
    AIN2_POWER_SET(AIN2_POWER_BIT(msg.power_state_field));
    AIN3_POWER_SET(AIN3_POWER_BIT(msg.power_state_field));
    AIN4_POWER_SET(AIN4_POWER_BIT(msg.power_state_field));

    AIN1_COND_SET(AIN1_COND_BIT(msg.power_state_field));
    AIN2_COND_SET(AIN2_COND_BIT(msg.power_state_field));
    AIN3_COND_SET(AIN3_COND_BIT(msg.power_state_field));
    AIN4_COND_SET(AIN4_COND_BIT(msg.power_state_field));

    /* Pend on Mutex to update data structure */
    take_return = xSemaphoreTake(anlginCurrentDataKey, portMAX_DELAY);
    while(take_return == pdFAIL){ /* Error trap, Mutex key never available */ };

    anlginCurrentData.power_state_field = msg.power_state_field;
    anlginCurrentData.ain1_samp_rate = AIN1_SAMP_RATE(msg.sampling_rate_field);
    anlginCurrentData.ain2_samp_rate = AIN2_SAMP_RATE(msg.sampling_rate_field);
    anlginCurrentData.ain3_samp_rate = AIN3_SAMP_RATE(msg.sampling_rate_field);
    anlginCurrentData.ain4_samp_rate = AIN4_SAMP_RATE(msg.sampling_rate_field);

    xSemaphoreGive(anlginCurrentDataKey);
}

/******************************************************************************
*   anlginCalibrateADC1() - Private function that calibrates ADC1 after reset
*   and before any conversions are initiated.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void anlginCalibrateADC1()
{
   uint16_t calibration_var;

   /* Enable hardware averaging 32 samples just for calibration sequence.
    * Initiate calibration. */
   ADC1->SC3 = ADC_SC3_AVGE(ENABLE) | ADC_SC3_AVGS(AVG_32_SAMP) |
               ADC_SC3_CAL(ENABLE);

   /* COCO flag will be set upon calibration complete, with CALF flag being set
    * if the calibration failed. */
   while(COCO_FLAG != SET){}
   while(CALI_FAIL_FLAG == SET){}

   /* As specified in section 39.5.6 of the K66 reference manual, a 16-bit
    * variable must sum various calibration value registers and divide by two
    * to calculate plus side and minus side gain values to complete calibration */
   calibration_var = (uint16_t)((ADC1->CLP0) + (ADC1->CLP1) + (ADC1->CLP2) +
                                (ADC1->CLP3) + (ADC1->CLP4) + (ADC1->CLPS));

   calibration_var = calibration_var >> 1U;
   calibration_var |= (uint16_t)0x80U;
   ADC1->PG = (uint32_t)calibration_var;

   calibration_var = (uint16_t)((ADC1->CLM0) + (ADC1->CLM1) + (ADC1->CLM2) +
                                (ADC1->CLM3) + (ADC1->CLM4) + (ADC1->CLMS));

   calibration_var = calibration_var >> 1U;
   calibration_var |= (uint16_t)0x80U;
   ADC1->MG = (uint32_t)calibration_var;

   /* Hardware averaging 32 samples is disabled as it is not wanted to normal
    * operation. */
   ADC1->SC3 &= ~(ADC_SC3_AVGE_MASK | ADC_SC3_AVGS_MASK);
}

/******************************************************************************
*   ADC1_IRQHandler() - Interrupt handler for ADC1 COCO (conversion complete)
*   flag. Posts task notification to Analog In Sampler Task.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void ADC1_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken;
//    uint16_t calibration_var;
//    calibration_var = (uint16_t)(ADC1->SC1[0]);
    xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(anlginSamplerTaskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
