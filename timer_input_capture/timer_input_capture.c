/*
Instructions for Timer Setup (can be done with STM32CubeMX or Platoform.io):
    1. Configure the system clock to any value (e.g., 96 MHz)
    2. Select Input Capture Direct Mode for Timer #1
    3. Set the Prescler to 96 (or whatever the value of your clock frequency is). This will divide
        the clock by 96, and bring the timer clock to 1 MHZ. Hence, the timer counts up every microsecond.
    4. Set ARR to 0xFFFF (a big value). Sets the upper limit - could be smaller if desired.
    5. Enable the TIMx Capture Compare interrupt.

Adapted from the following source:
https://controllerstech.com/hcsr04-ultrasonic-sensor-and-stm32/
*/


// Variable Definitions
uint32_t val1 = 0;
uint32_t val2 = 0;
uint32_t diff = 0;
uint8_t is_first_captured = 0; // Has the first value been captured
uint8_t distance  = 0;


// Defines for GPIO Port and Pin
#define TRIG_PORT GPIOE
#define TRIG_PIN GPIO_PIN_8


// Function Prototypes
static void HCSR04_Read (void);
void delay_in_us (uint16_t time);


// Function that delays in microseconds (usec)
void delay_in_us (uint16_t time) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) <  time);
}


// Input Capture Callback ISR
// No function prototype because it is ISR and a HAL function
void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) { // If the interrupt source is CHANNEL 1
		if (is_first_captured == 0) { // If the first value is not captured
			val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Read the first value
			is_first_captured = 1;
			// Change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (is_first_captured == 1) { // If the first is already captured
			val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // Read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // Reset the counter

			if (val2 > val1) {
				diff = val2 - val1; // diff is in microseconds
			}

			else if (val1 > val2) {
				diff = (0xffff - val1) + val2;  // diff is in microseconds
			}

			distance = diff * .034/2;  // UNITS BREAKDOWN: (10^(-6) s) * (10^4 m/s) = 10^(-2) m = cm -> distance is in cm
			is_first_captured = 0; // Set back to false

			// Set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}


// Drive TRIG Pin
static void HCSR04_Read (void) {
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}


// Structure for a main function
int main(void) {
    /***************************************/
    /*Intialize all needed peripherals here*/
    /***************************************/

    // Start the TIM Input Capture measurement in interrupt mode
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

    while (1) {
       HCSR04_Read();  // Set TRIG Pin
       HAL_Delay(60);  // Wait ms until next sensor reading
    }
}