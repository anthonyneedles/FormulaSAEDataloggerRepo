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
*   Comments up to date as of: 05/22/2019
*
*   Created on: 05/10/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "AnalogInput.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "Debug.h"

/******************************************************************************
*   Private Definitions
******************************************************************************/
#define ENABLE                      0x01U
#define SET                         0x01U
#define ALT_1_GPIO                  0x01U
#define OUTPUT                      0x01U
#define BIT_0_MASK        ((uint8_t)0x01U)
#define SAMP_RATE_MASK    ((uint8_t)0x03U)
#define PIT1                        0x01U
#define PIT_8KHZ_LDVAL              7499U
#define PIT_200HZ_LDVAL           299999U
#define PIT1_TRG                    0x05U
#define BUS_CLK                     0x00U
#define MODE_12BIT                  0x01U
#define LONG_SAMPLES                0x01U
#define CLK_DIV_8                   0x03U
#define B_CHNLS                     0x01U
#define AVG_32_SAMP                 0x03U
#define HW_TRG                      0x01U
#define SINGLE_ENDED                0x00U
#define SAMP_RATE_100HZ             0x02U

#define AINSAMPLERTASK_PRIORITY        6U
#define AINSAMPLERTASK_STKSIZE       256U

/* PORTB. */
#define AIN1_POWER_PIN_NUM            23U
#define AIN2_POWER_PIN_NUM            22U
#define AIN3_POWER_PIN_NUM            21U
#define AIN4_POWER_PIN_NUM            20U

/* PORTC. */
#define AIN1_COND_PIN_NUM             13U
#define AIN2_COND_PIN_NUM             12U
#define AIN3_COND_PIN_NUM             15U
#define AIN4_COND_PIN_NUM             14U

/* ADC1 channels for each AIN. */
#define AIN1_SIG_CHNL                  5U
#define AIN2_SIG_CHNL                  4U
#define AIN3_SIG_CHNL                  7U
#define AIN4_SIG_CHNL                  6U

/* Bit # corresponding to certain AIN in 8-bit state/power field message. */
#define AIN1_POWER_BIT_NUM   ((uint8_t)0U)
#define AIN2_POWER_BIT_NUM   ((uint8_t)1U)
#define AIN3_POWER_BIT_NUM   ((uint8_t)2U)
#define AIN4_POWER_BIT_NUM   ((uint8_t)3U)
#define AIN1_STATE_BIT_NUM   ((uint8_t)4U)
#define AIN2_STATE_BIT_NUM   ((uint8_t)5U)
#define AIN3_STATE_BIT_NUM   ((uint8_t)6U)
#define AIN4_STATE_BIT_NUM   ((uint8_t)7U)

/* Bit offset for each AIN sampling rate position in SR message. */
#define AIN1_SR_OFFSET      ((uint32_t)0U)
#define AIN2_SR_OFFSET      ((uint32_t)2U)
#define AIN3_SR_OFFSET      ((uint32_t)4U)
#define AIN4_SR_OFFSET      ((uint32_t)6U)

/* Sets AIN power pin, providing 12V power to AIN sensor. */
#define A1_PWR_12V() (GPIOB->PDOR |= ((1U) << AIN1_POWER_PIN_NUM))
#define A2_PWR_12V() (GPIOB->PDOR |= ((1U) << AIN2_POWER_PIN_NUM))
#define A3_PWR_12V() (GPIOB->PDOR |= ((1U) << AIN3_POWER_PIN_NUM))
#define A4_PWR_12V() (GPIOB->PDOR |= ((1U) << AIN4_POWER_PIN_NUM))

/* Clears AIN power pin, providing 5V power to AIN sensor. */
#define A1_PWR_5V() (GPIOB->PDOR &= ~((1U) << AIN1_POWER_PIN_NUM))
#define A2_PWR_5V() (GPIOB->PDOR &= ~((1U) << AIN2_POWER_PIN_NUM))
#define A3_PWR_5V() (GPIOB->PDOR &= ~((1U) << AIN3_POWER_PIN_NUM))
#define A4_PWR_5V() (GPIOB->PDOR &= ~((1U) << AIN4_POWER_PIN_NUM))

/* Sets AIN conditioning pin, providing 5V conditioning to AIN sensor signal. */
#define A1_COND_5V() (GPIOC->PDOR |= ((1U) << AIN1_COND_PIN_NUM))
#define A2_COND_5V() (GPIOC->PDOR |= ((1U) << AIN2_COND_PIN_NUM))
#define A3_COND_5V() (GPIOC->PDOR |= ((1U) << AIN3_COND_PIN_NUM))
#define A4_COND_5V() (GPIOC->PDOR |= ((1U) << AIN4_COND_PIN_NUM))

/* Clears AIN conditioning pin, providing 12V conditioning to AIN sensor signal. */
#define A1_COND_12V() (GPIOC->PDOR &= ~((1U) << AIN1_COND_PIN_NUM))
#define A2_COND_12V() (GPIOC->PDOR &= ~((1U) << AIN2_COND_PIN_NUM))
#define A3_COND_12V() (GPIOC->PDOR &= ~((1U) << AIN3_COND_PIN_NUM))
#define A4_COND_12V() (GPIOC->PDOR &= ~((1U) << AIN4_COND_PIN_NUM))

/* Register Flags. */
#define COCO_FLAG ((ADC1->SC1[0] & ADC_SC1_COCO_MASK) >> ADC_SC1_COCO_SHIFT)
#define CALI_FAIL_FLAG ((ADC1->SC3 & ADC_SC3_CALF_MASK) >> ADC_SC3_CALF_SHIFT)

/* Macros accepting 8 bit field to find and isolate AIN's power bit and
 * shift over to bit 0 position for relevant SET function */
#define AIN1_POWER_BIT(x) (((x) >> AIN1_POWER_BIT_NUM) & BIT_0_MASK)
#define AIN2_POWER_BIT(x) (((x) >> AIN2_POWER_BIT_NUM) & BIT_0_MASK)
#define AIN3_POWER_BIT(x) (((x) >> AIN3_POWER_BIT_NUM) & BIT_0_MASK)
#define AIN4_POWER_BIT(x) (((x) >> AIN4_POWER_BIT_NUM) & BIT_0_MASK)

/* Macros accepting 8 bit field to find and isolate AIN's conditioning bit and
 * shift over to bit 0 position for relevant SET function.
 *
 * NOTE: Power and conditioning set macros use the same bit for each AIN, but
 * a '0' corresponds to a FIVE_VOLTS power set but a '0' corresponds to a
 * TWELVE_VOLTS conditioning set. */
#define AIN1_COND_BIT(x) (AIN1_POWER_BIT(x))
#define AIN2_COND_BIT(x) (AIN2_POWER_BIT(x))
#define AIN3_COND_BIT(x) (AIN3_POWER_BIT(x))
#define AIN4_COND_BIT(x) (AIN4_POWER_BIT(x))

/* Macros accepting 8 bit field to find and isolate AIN's sampling rate byte
 * and shift over to bits [0:7], then casting to individual 8-bit value */
#define AIN1_SAMP_RATE(x) ((uint8_t)(((x) >> AIN1_SR_OFFSET) & SAMP_RATE_MASK))
#define AIN2_SAMP_RATE(x) ((uint8_t)(((x) >> AIN2_SR_OFFSET) & SAMP_RATE_MASK))
#define AIN3_SAMP_RATE(x) ((uint8_t)(((x) >> AIN3_SR_OFFSET) & SAMP_RATE_MASK))
#define AIN4_SAMP_RATE(x) ((uint8_t)(((x) >> AIN4_SR_OFFSET) & SAMP_RATE_MASK))

/******************************************************************************
*   Private Variables
******************************************************************************/
/* Data structure that will hold all Analog Input data and configurations.
 * Secured by Mutex "ainCurrentDataKey". */
static ain_data_t ainCurrentData;

/* Mutex key that will protect ainCurrentData structure. Must be pended on
 * if writing/reading ainCurrentData is desired to ensure synchronization. */
static SemaphoreHandle_t ainCurrentDataKey;

/* Task handle for Analog In sampler task, used by ADC1_ISRHandler() to post
 * task notification. */
static TaskHandle_t ainSamplerTaskHandle = NULL;

static volatile uint16_t ainADCSample;
static volatile uint8_t ainCurrentChannel;

/******************************************************************************
*   Private Function Prototypes
******************************************************************************/
static void ainCalibrateADC1(void);

static void ainSamplerTask(void *);

static void ainResurrectModule(void);

/******************************************************************************
*   AInInit() - Public function to configure all GPIO used as power and
*   conditioning selects as outputs. Initializes Analog In data structure and
*   creates Mutex to protect data structure. Also initializes PIT1 to trigger
*   ADC1 at 8kHz. Configures and calibrates ADC1.
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
void AInInit()
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
    ainCalibrateADC1();
    ADC1->SC2 = ADC_SC2_ADTRG(HW_TRG);

    ainCurrentChannel = AIN1_SIG_CHNL;

    ADC1->SC1[0] = ADC_SC1_AIEN(ENABLE) | ADC_SC1_DIFF(SINGLE_ENDED) |
                   ADC_SC1_ADCH(ainCurrentChannel);

    /* Initial configuration: 12V input conditioning and 12V power supply. */
    ainCurrentData.power_state_field = (uint8_t)0xFFU;
    ainCurrentData.ain1_data = (uint16_t)0x0000U;
    ainCurrentData.ain2_data = (uint16_t)0x0000U;
    ainCurrentData.ain3_data = (uint16_t)0x0000U;
    ainCurrentData.ain4_data = (uint16_t)0x0000U;
    ainCurrentData.ain1_samp_rate = SAMP_RATE_100HZ;
    ainCurrentData.ain1_samp_rate = SAMP_RATE_100HZ;
    ainCurrentData.ain1_samp_rate = SAMP_RATE_100HZ;
    ainCurrentData.ain1_samp_rate = SAMP_RATE_100HZ;

    /* Convert power/conditioning message to individual bits for each AIN and
     * changes power output and input conditioning accordingly */
    (AIN1_POWER_BIT(ainCurrentData.power_state_field) == 1U) ? A1_PWR_12V() : A1_PWR_5V();
    (AIN2_POWER_BIT(ainCurrentData.power_state_field) == 1U) ? A2_PWR_12V() : A2_PWR_5V();
    (AIN3_POWER_BIT(ainCurrentData.power_state_field) == 1U) ? A3_PWR_12V() : A3_PWR_5V();
    (AIN4_POWER_BIT(ainCurrentData.power_state_field) == 1U) ? A4_PWR_12V() : A4_PWR_5V();

    (AIN1_COND_BIT(ainCurrentData.power_state_field) == 1U) ? A1_COND_12V() : A1_COND_5V();
    (AIN2_COND_BIT(ainCurrentData.power_state_field) == 1U) ? A2_COND_12V() : A2_COND_5V();
    (AIN3_COND_BIT(ainCurrentData.power_state_field) == 1U) ? A3_COND_12V() : A3_COND_5V();
    (AIN4_COND_BIT(ainCurrentData.power_state_field) == 1U) ? A4_COND_12V() : A4_COND_5V();

    ainCurrentDataKey = xSemaphoreCreateMutex();
    while(ainCurrentDataKey == NULL){ /* Out of heap memory (DEBUG TRAP). */ }

    task_create_return = xTaskCreate(ainSamplerTask,
                                     "Analog In Sampler Task",
                                     AINSAMPLERTASK_STKSIZE,
                                     NULL,
                                     AINSAMPLERTASK_PRIORITY,
                                     &ainSamplerTaskHandle);

    while(task_create_return == pdFAIL){ /* Out of heap memory (DEBUG TRAP). */ }

    NVIC_SetPriority(ADC1_IRQn, 2U);
    NVIC_ClearPendingIRQ(ADC1_IRQn);
    NVIC_EnableIRQ(ADC1_IRQn);
}

/******************************************************************************
*   ainSamplerTask() - This FreeRTOS task pends on a task notification that will
*   be posted whenever a new sample is taken from ADC1. This sample is written
*   into the corresponding analog input channel from which the sample was taken.
*
*   Parameters:
*
*       void *pvParameters - A value that will passed into the task when it is
*       created as the task's parameter. Not used for this task.
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void ainSamplerTask(void *pvParameters)
{
    uint32_t notify_count;

    while(1)
    {
        /* Place task into idle state until ADC1 ISR notifies task. */
        notify_count = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        if(notify_count == 0)
        { /* Resurrect if failed to be notified. */
            ainResurrectModule();
        } else {}

        xSemaphoreTake(ainCurrentDataKey, portMAX_DELAY);

        switch(ainCurrentChannel)
        {
            case AIN1_SIG_CHNL:
                ainCurrentData.ain4_data = ainADCSample;
                break;

            case AIN2_SIG_CHNL:
                ainCurrentData.ain1_data = ainADCSample;
                break;

            case AIN3_SIG_CHNL:
                ainCurrentData.ain2_data = ainADCSample;
                break;

            case AIN4_SIG_CHNL:
                ainCurrentData.ain3_data = ainADCSample;
                break;

            default:
                break;
        }

        xSemaphoreGive(ainCurrentDataKey);
    }
}

/******************************************************************************
*   ADC1_IRQHandler() - Interrupt handler for ADC1 COCO (conversion complete)
*   flag. Posts task notification to Analog In Sampler Task. Performs a channel
*   switch on the ADC1 as well as progressing the order of which channel is
*   currently being sampled.
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
void ADC1_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    switch(ainCurrentChannel)
    {
        case AIN1_SIG_CHNL:
            ainCurrentChannel = AIN2_SIG_CHNL;
            break;

        case AIN2_SIG_CHNL:
            ainCurrentChannel = AIN3_SIG_CHNL;
            break;

        case AIN3_SIG_CHNL:
            ainCurrentChannel = AIN4_SIG_CHNL;
            break;

        case AIN4_SIG_CHNL:
            ainCurrentChannel = AIN1_SIG_CHNL;
            break;

        default:
            break;
    }

    /* Clear channel field and populate with new channel. */
    ADC1->SC1[0] = ((ADC1->SC1[0] & ~ADC_SC1_ADCH_MASK) | ADC_SC1_ADCH(ainCurrentChannel));

    ainADCSample = ADC1->R[0];

    vTaskNotifyGiveFromISR(ainSamplerTaskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/******************************************************************************
*   AInSet() - Public function to set power and conditioning pins to achieve
*   requested results from received user message. Saves the new Analog In data
*   from message in Mutex protected private data structure.
*
*   Parameters:
*
*       ain_msg_t msg - Message structure received from telemetry unit
*       with 8 bit power and state field (msg.power_state_field) and 32 bit
*       sampling rates field (msg.sampling_rate_field).
*
*       msg.power_state_field[0:3] corresponds to AIN[1:4] power, with a bit of
*       0 representing a power output of 5V (1 = 12V).
*
*       msg.power_state_field[4:7] corresponds to AIN[1:4] state, with a bit of
*       0 representing a non-active sensor input (1 = active).
*
*       msg.sampling_rate_field[0:1] corresponds to AIN1 sampling rate, and
*       msg.sampling_rate_field[2:3] corresponds to AIN2 sampling rate, and
*       msg.sampling_rate_field[4:5] corresponds to AIN3 sampling rate, and
*       msg.sampling_rate_field[6:7] corresponds to AIN4 sampling rate.
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
void AInSet(ain_msg_t msg)
{
    /* Convert power/conditioning message to individual bits for each AIN and
     * changes power output and input conditioning accordingly */
    (AIN1_POWER_BIT(msg.power_state_field) == 1U) ? A1_PWR_12V() : A1_PWR_5V();
    (AIN2_POWER_BIT(msg.power_state_field) == 1U) ? A2_PWR_12V() : A2_PWR_5V();
    (AIN3_POWER_BIT(msg.power_state_field) == 1U) ? A3_PWR_12V() : A3_PWR_5V();
    (AIN4_POWER_BIT(msg.power_state_field) == 1U) ? A4_PWR_12V() : A4_PWR_5V();

    (AIN1_COND_BIT(msg.power_state_field) == 1U) ? A1_COND_12V() : A1_COND_5V();
    (AIN2_COND_BIT(msg.power_state_field) == 1U) ? A2_COND_12V() : A2_COND_5V();
    (AIN3_COND_BIT(msg.power_state_field) == 1U) ? A3_COND_12V() : A3_COND_5V();
    (AIN4_COND_BIT(msg.power_state_field) == 1U) ? A4_COND_12V() : A4_COND_5V();

    /* Pend on Mutex to update data structure. */
    xSemaphoreTake(ainCurrentDataKey, portMAX_DELAY);

    ainCurrentData.power_state_field = msg.power_state_field;
    ainCurrentData.ain1_samp_rate = AIN1_SAMP_RATE(msg.sampling_rate_field);
    ainCurrentData.ain2_samp_rate = AIN2_SAMP_RATE(msg.sampling_rate_field);
    ainCurrentData.ain3_samp_rate = AIN3_SAMP_RATE(msg.sampling_rate_field);
    ainCurrentData.ain4_samp_rate = AIN4_SAMP_RATE(msg.sampling_rate_field);

    xSemaphoreGive(ainCurrentDataKey);
}

/******************************************************************************
*   AInGetData() - Public function to copy current analog in data structure
*   for use in transmission/storage.
*
*   Parameters:
*
*       ain_data_t *ldata - Pointer to caller-side data structure which will
*       have current data copied to it.
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
void AInGetData(ain_data_t *ldata)
{
    /* Pend on Mutex to update data structure */
    xSemaphoreTake(ainCurrentDataKey, portMAX_DELAY);

    *ldata = ainCurrentData;

    xSemaphoreGive(ainCurrentDataKey);
}

/******************************************************************************
*   ainCalibrateADC1() - Private function that calibrates ADC1 after reset
*   and before any conversions are initiated.
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void ainCalibrateADC1()
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
*   ainResurrectModule() - Private function that attempts so solve a timeout
*   error by reinitializing module. Disables preemption and interrupts of a
*   system priority <= configMAX_SYSCALL_INTERRUPT_PRIORITY (effectively, these
*   are interrupts set to have a priority <= 1 by NVIC_SetPriority()) when
*   deleting RTOS structures.
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void ainResurrectModule()
{
    taskENTER_CRITICAL();

    NVIC_DisableIRQ(ADC1_IRQn);
    vSemaphoreDelete(ainCurrentDataKey);
    vTaskDelete(ainSamplerTaskHandle);

    taskEXIT_CRITICAL();

    AInInit();
}
