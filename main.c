#include <stdio.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>

//global variables
volatile uint8_t minutes = 0;
volatile uint8_t seconds = 0;
volatile uint8_t hundredths = 0;
volatile uint8_t startFlag = 0;
volatile uint8_t lapFlag = 0;
volatile uint8_t stopFlag = 0;
volatile uint8_t resetFlag = 1;
volatile uint8_t lapMinutes = 0;
volatile uint8_t lapSeconds = 0;
volatile uint8_t lapHundredths = 0;

//functions
void reset_state();
void initGPIO();
void initTIM14();
void display();
void checkPB();
void convert2BCDASCII(uint8_t min, uint8_t sec, uint8_t hund, char* resultPtr);
void delay_ms(uint32_t ms);
void init_LED();

void initGPIO(void) {
    RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN);

    //enable buttons (sw0, sw1, sw2 & sw3) as inputs
    GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 |
                     GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1 |
                     GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR3);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 | 
                    GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0);

    //configure LEDs PB0-PB3 as outputs
    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 |
                     GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 |
                     GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0);
}

void initTIM14(void){
    //enable TIM14 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

    TIM14->PSC = 1;
    TIM14->ARR = 39999;

    //enable update interrupt
    TIM14->DIER |= TIM_DIER_UIE;

    //enable TIM14 interrupt in NVIC
    NVIC_EnableIRQ(TIM14_IRQn);

    //start time
    TIM14->CR1 |= TIM_CR1_CEN;
}

void TIM14_IRQHandler(void){
    if (TIM14->SR & TIM_SR_UIF){
        TIM14->SR &= ~TIM_SR_UIF; //clear interrupt flag

        if (startFlag && !stopFlag){
            hundredths++;
            if (hundredths >= 100){
                hundredths = 0;     //reset hundredths
                seconds++;          //increment seconds
                if (seconds >= 60){
                    seconds = 0;    //reset seconds
                    minutes++;      //increment minutes
                }
            }
        }
    }
}

//convert time to BCD ASCII format
void convert2BCDASCII(uint8_t min, uint8_t sec, uint8_t hund, char* resultPtr){
    resultPtr[0] = (min/10) + '0';
    resultPtr[1] = (min%10) + '0';
    resultPtr[2] = ':';
    resultPtr[3] = (sec/10) + '0';
    resultPtr[4] = (sec%10) + '0';
    resultPtr[5] = '.';
    resultPtr[6] = (hund/10) + '0';
    resultPtr[7] = (hund%10) + '0';
    resultPtr[8] = '\0';
}

void display(void){
    char timeString[16];

    if (resetFlag){
        lcd_command(CLEAR);
        lcd_putstring("Stop Watch");
        lcd_command(0xC0);
        lcd_putstring("Press SW0...");

        GPIOB->ODR = GPIO_ODR_3; //D3 (reset) LED on
    }
    else if (stopFlag){
        if ((GPIOB->ODR && GPIO_ODR_2) ==0 ){
        lcd_command(CLEAR);}
        //lcd_putstring("Time");
        lcd_command(0xC0);
        convert2BCDASCII(minutes, seconds, hundredths, timeString);
        lcd_putstring(timeString);

        GPIOB->ODR = GPIO_ODR_2; //D2 (STOP) LED on
    }
    else if (lapFlag){
        lcd_command(CLEAR);
        lcd_putstring("Time");
        lcd_command(0xC0);
        convert2BCDASCII(lapMinutes, lapSeconds, lapHundredths, timeString);
        lcd_putstring(timeString);

        GPIOB->ODR = GPIO_ODR_1; //D1 (LAP) LED on
    }
    else{
        lcd_command(CLEAR);
        lcd_putstring("Time");
        lcd_command(0xC0);
        convert2BCDASCII(minutes, seconds, hundredths, timeString);
        lcd_putstring(timeString);

        GPIOB->ODR = GPIO_ODR_0; //D0 (START) LED on
    }
}

void checkPB(void){
    //simple debounce delay
    delay_ms(20);

    //SW0 (START) pressed
    if (!(GPIOA->IDR & GPIO_IDR_0)){
        startFlag = 1;
        lapFlag = 0;
        stopFlag = 0;
        resetFlag = 0;
        TIM14->CR1 |= TIM_CR1_CEN; //start timer

        //wait for button to be release
        while (!(GPIOA->IDR & GPIO_IDR_0));
    }
    //sw1 (LAP) pressed
    else if (!(GPIOA->IDR & GPIO_IDR_1)){
        if (startFlag && !stopFlag){
            lapFlag = 1;
            //stopFlag = 1;
            lapMinutes = minutes;
            lapSeconds = seconds;
            lapHundredths = hundredths;

            //TIM14->CR1 &= ~TIM_CR1_CEN; //stop timer

            //wait for button to be release
            //while (!(GPIOA->IDR & GPIO_IDR_1));
        }
    }
    //sw2 (STOP) pressed
    else if (!(GPIOA->IDR & GPIO_IDR_2)){
        if (startFlag){
            stopFlag = 1;
            lapFlag = 0;
            TIM14->CR1 &= ~TIM_CR1_CEN; //stop timer

            //wait for button to be release
            while (!(GPIOA->IDR & GPIO_IDR_2));
        }
    }
    //sw3 (RESET) pressed
    else if (!(GPIOA->IDR & GPIO_IDR_3)){
        reset_state();

        //wait for button to be release
        while (!(GPIOA->IDR & GPIO_IDR_3));
    }
}

//reset function
void reset_state() {    
    minutes = 0;
    seconds = 0;
    hundredths = 0;
    startFlag = 0;
    lapFlag = 0;
    stopFlag = 0;
    resetFlag = 1;
    TIM14->CR1 &= ~TIM_CR1_CEN; //stop timer

    display();
}

void delay_ms(uint32_t ms){
    for (volatile uint32_t i = 0; i < ms * 800; i++);
}

void init_LED(){
    init_LCD();
    lcd_command(CLEAR);
    lcd_putstring("Stop Watch");
    lcd_command(0xC0);
    lcd_putstring("Press SW0...");
}

int main(void) {
    initGPIO();
    initTIM14();
    init_LCD();
    reset_state();
    
    while(1){
        checkPB();
        display();
        delay_ms(100); //small delay to reduce CPU usage
    }
}