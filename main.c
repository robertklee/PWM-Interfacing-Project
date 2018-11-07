//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

#include "stm32f0xx.h"
#include "stm32f0xx_spi.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);

volatile unsigned char previousEdgeFound = 0;  // 0/1: first/not first edge

int
main(int argc, char* argv[]){


    trace_printf("This is Part 2 of Introductory Lab...\n");
    trace_printf("System clock: %u Hz\n", SystemCoreClock);

    myGPIOA_Init();     /* Initialize I/O port PA */
    myTIM2_Init();      /* Initialize timer TIM2 */
    myEXTI_Init();      /* Initialize EXTI */


    trace_printf("oh no pooh you're eating loops");

    GPIOB->BRR |= 0x0010;

    while( ((SPI1->SR & 0x0080) != 0) || (SPI1->SR & 0x0002) == 0); // Page 759 of reference manual, bit 1 is TXE, bit 7 is BSY



    while (1)
    {
    	trace_printf("(loops)");
    }

    return 0;

}


void myGPIOA_Init()
{
    /* Enable clock for GPIOA peripheral */
    // Relevant register: RCC->AHBENR
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* Configure PA1 as input */
    // Relevant register: GPIOA->MODER
    GPIOA->MODER &= ~(GPIO_MODER_MODER1);

    /* Ensure no pull-up/pull-down for PA1 */
    // Relevant register: GPIOA->PUPDR
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
}

void myGPIOB_Init()
{
    /* Enable clock for GPIOB peripheral */
    // Relevant register: RCC->AHBENR
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // set GPIOB[3 and 5] to be AF mode, set GPIOB[4] to be general purpose output
    GPIOB->MODER = (GPIOB->MODER & ~(0x00000980)) | 0x00000980;

    /* Ensure no pull-up/pull-down for PB3,4,5 */
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5);

    GPIO_PinAFConfig(GPIOB, 3, GPIO_AF_0); // set PB3's AF to SPI1_SCK
    GPIO_PinAFConfig(GPIOB, 5, GPIO_AF_0); // set PB5's AF to SPI1_MOSI
}

void mySPI1_Init()
{
	SPI_InitTypeDef SPI_InitStructInfo;
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;
	//TODO...
	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = 0; //TODO do we need to change this? ... ;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, SPI_InitStruct);
	//TODO ...
	SPI_Cmd(SPI1, ENABLE);

}


void myTIM2_Init()
{
    /* Enable clock for TIM2 peripheral */
    // Relevant register: RCC->APB1ENR
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Configure TIM2: buffer auto-reload, count up, stop on overflow,
     * enable update events, interrupt on overflow only */
    // Relevant register: TIM2->CR1
    TIM2->CR1 = ((uint16_t)0x008C); /* 008C sets auto-reload preload enabled, stops the
                                     *  counter on an update event, and counter ONLY
                                     *  controlled by over.underflow */

    /* Set clock prescaler value */
    TIM2->PSC = myTIM2_PRESCALER;
    /* Set auto-reloaded delay */
    TIM2->ARR = myTIM2_PERIOD;

    /* Update timer registers */
    // Relevant register: TIM2->EGR
    TIM2->EGR =((uint16_t)0x0001);

    /* Assign TIM2 interrupt priority = 0 in NVIC */
    // Relevant register: NVIC->IP[3], or use NVIC_SetPriority
    NVIC_SetPriority(TIM2_IRQn, 64); //TODO could be better as 64?

    /* Enable TIM2 interrupts in NVIC */
    // Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
    NVIC_EnableIRQ(TIM2_IRQn);

    /* Enable update interrupt generation */
    // Relevant register: TIM2->DIER
    TIM2->DIER |= TIM_DIER_UIE;
}


void myEXTI_Init()
{
    /* Map EXTI1 line to PA1 */
    // Relevant register: SYSCFG->EXTICR[0]
    SYSCFG->EXTICR[0] &= ((uint32_t) 0xFFFFFF0F);

    /* EXTI1 line interrupts: set rising-edge trigger */
    // Relevant register: EXTI->RTSR
    EXTI->RTSR |= ((uint32_t) 0x00000002);

    /* Unmask interrupts from EXTI1 line */
    // Relevant register: EXTI->IMR
    EXTI->IMR |= ((uint32_t) 0x00000002);

    /* Assign EXTI1 interrupt priority = 0 in NVIC */
    // Relevant register: NVIC->IP[1], or use NVIC_SetPriority
    NVIC_SetPriority(EXTI0_1_IRQn, 0);


    /* Enable EXTI1 interrupts in NVIC */
    // Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
    /* Check if update interrupt flag is indeed set */
    if ((TIM2->SR & TIM_SR_UIF) != 0)
    {
        trace_printf("\n*** Overflow! ***\n");

        /* Clear update interrupt flag */
        // Relevant register: TIM2->SR
        TIM2->SR &= ~(TIM_SR_UIF);

        /* Restart stopped timer */
        // Relevant register: TIM2->CR1
        TIM2->CR1 |= TIM_CR1_CEN;
    }
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
    // Your local variables...
    volatile unsigned int frequency = 0;
    volatile unsigned int period = 0;
    volatile unsigned int counterValue = 0;
    /* Check if EXTI1 interrupt pending flag is indeed set */
    if ((EXTI->PR & EXTI_PR_PR1) != 0)
    {
        //
        // 1. If this is the first edge:
        //  - Clear count register (TIM2->CNT).
        //  - Start timer (TIM2->CR1).
        //    Else (this is the second edge):
        //  - Stop timer (TIM2->CR1).
        //  - Read out count register (TIM2->CNT).
        //  - Calculate signal period and frequency.
        //  - Print calculated values to the console.
        //    NOTE: Function trace_printf does not work
        //    with floating-point numbers: you must use
        //    "unsigned int" type to print your signal
        //    period and frequency.
        //
        // 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
        //

        if (!previousEdgeFound) {
            TIM2->CNT = ((uint32_t) 0x0);
            TIM2->CR1 |= TIM_CR1_CEN;
            previousEdgeFound = 1;
        }

        else
        {
            TIM2->CR1 &= ~(TIM_CR1_CEN);
            EXTI->IMR &= ~((uint32_t) 0x00000002);
            counterValue = TIM2->CNT;
            frequency = (unsigned int)((48000000000.0)/(counterValue)); // extra factor of 1000 for mHz
            trace_printf("Signal Frequency: %d.%03d Hz\t", frequency/1000, frequency%1000);
            period = 1000000000.0 / frequency; //period in ms
            trace_printf("Signal Period: %d.%06d s \n", period/1000000, period%1000000);
            TIM2->CNT = ((uint32_t) 0x0);
            previousEdgeFound = 0;
            EXTI->IMR |= ((uint32_t) 0x00000002);
        }
        EXTI->PR |= (EXTI_PR_PR1); // to clear, software must write 1
    }
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
