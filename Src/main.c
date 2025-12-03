/*
 * CmpE443 - PI 1: GPIO and Timer Implementation
 * ------------------------------------------------
 * Hardware (PI 1):
 * - 4x3 Keypad -> GPIOF (Pins defined below)
 * - Red LED    -> PA3 (Active-LOW: ON when system is OFF)
 * - Green LED  -> PA2 (Active-LOW: ON when system is ON)
 *
 * PI 1 Logic:
 * 1. System starts in OFF state (Red LED ON, Green LED OFF).
 * 2. Polls keypad for input.
 * 3. On '#' press, check buffered password against "1234".
 * 4. If correct: Toggle the system state (OFF -> ON, or ON -> OFF).
 * 5. Update LEDs based on the new state.
 * 6. All delays (e.g., debouncing) are handled by TIM6.
 * 7. Initialize TIM2 (for PI 2: IC/OC) - Clock only, do not start.
 * 8. Initialize TIM7 (for PI 3: ADC Trigger) - Clock only, do not start.

 * CmpE443 - PI 2: IC/OC Implementation
 * ------------------------------------------------
 * Hardware (PI 2):
 * - Vibration Sensor -> TIM15 (Input Capture)
 * - Buzzer -> TIM16 (PWM)
 *
 * PI 2 Logic:
 * 1. TIM15 Input Capture monitors vibration sensor on PA2 (both edges).
 * 2. Vibration pulses are captured via interrupt and timestamped.
 * 3. Pulse frequency analysis determines if vibration is irregular (theft).
 * 4. When irregular vibration detected: Enter MONITORING state (Blue LED ON).
 * 5. TIM16 PWM generates buzzer tone on PA6 when acceleration threshold exceeded.
 * 6. Buzzer frequency alternates between 1kHz and 500Hz every second.
 * 7. System returns to IDLE when acceleration drops below threshold.
 * 8. Buzzer stays ON until correct password entered via keypad.

 */

 #include <stdio.h>
 #include <stdint.h>
 #include <stdbool.h>
 #include <string.h>

 //=============================================================================
 // 1. System Parameters (Mock Values for PI 3)
 //=============================================================================

 // The duration in seconds that acceleration must be continuously detected before the alarm is triggered
 #define ACCELERATION_TIME_THRESHOLD_S 3

 // The correct password for disarming the alarm via Keypad
 #define CORRECT_PASSWORD "1234"

 // Buffer size for storing the password input from the keypad
 #define PASSWORD_BUFFER_SIZE 10

 // A mock threshold for the accelerometer's ADC reading to be considered as significant motion
 #define ACCELERATION_ADC_THRESHOLD 50

 // The number of vibration pulses required to trigger the monitoring state within the reset interval
 #define VIBRATION_PULSE_THRESHOLD 180

 // The time interval in seconds after which vibration count is reset
 #define VIBRATION_RESET_INTERVAL_S 1

 // The number of acceleration values to keep in the sliding window
 #define ACCELERATION_WINDOW_SIZE 20

 // The threshold for the computed acceleration pattern value to trigger the alarm
 #define ACCELERATION_PATTERN_THRESHOLD 0.75f

 // Interval in milliseconds between accelerometer readings in monitoring mode
 #define ACCELEROMETER_READ_INTERVAL_MS 200

 // Maximum number of vibration pulse intervals to store for analysis
 #define VIBRATION_INTERVAL_BUFFER_SIZE 200

 // Threshold for vibration frequency irregularity to trigger monitoring (higher = more irregular)
 #define VIBRATION_IRREGULARITY_THRESHOLD 0.6f


 //=============================================================================
 // PI 1&2: BARE-METAL REGISTER DEFINITIONS
 //=============================================================================

 // For handling floats
#define CPACR_BASE 0xE000ED88
#define CPACR (*((volatile uint32_t *) CPACR_BASE))

 // --- RCC (Clock Control) Registers ---
 // Base: 0x4002 1000
 #define RCC_AHB2ENR     *((volatile uint32_t *) 0x4002104C) // For GPIOA, GPIOF
 #define RCC_APB1ENR1    *((volatile uint32_t *) 0x40021058) // For TIM6
 #define RCC_APB2ENR     *((volatile uint32_t *) 0x40021060) // For TIM15
#define RCC_CCIPR1      *((volatile uint32_t *) 0x40021088) // Added for ADC



 // --- GPIOA (LEDs) Registers (PA3 = Green, PA4 = Blue, PA5 = Red) ---
 // Base: 0x4202 0000
 #define GPIOA_MODER     *((volatile uint32_t *) 0x42020000) // Mode Register (Offset 0x00)
 #define GPIOA_ODR       *((volatile uint32_t *) 0x42020014) // Output Data Register (Offset 0x14)
 #define GPIOA_AFRL *((volatile uint32_t *) 0x42020020)
 #define GPIOA_PUPDR *((volatile uint32_t *) 0x4202000C)

 // --- GPIOF (Keypad) Registers (Cols: PF0-2, Rows: PF3,5,13,14) ---
 // Base: 0x4202 1400
 #define GPIOF_MODER     *((volatile uint32_t *) 0x42021400) // Mode Register (Offset 0x00)
 #define GPIOF_ODR       *((volatile uint32_t *) 0x42021414) // Output Data Register (Offset 0x14)
 #define GPIOF_IDR       *((volatile uint32_t *) 0x42021410) // Input Data Register (Offset 0x10)
 #define GPIOF_PUPDR     *((volatile uint32_t *) 0x4202140C) // Pull-up/Pull-down Register (Offset 0x0C)

// --- PI 1: TIM6 (Delay Timer) Registers ---
// Base: 0x4000 1000
 #define RCC_APB1ENR1_TIM6EN (1 << 4) // TIM6 Clock Enable Bit
 #define TIM6_CR1        *((volatile uint32_t *) 0x40001000) // Control Register 1 (Offset 0x00)
 #define TIM6_SR         *((volatile uint32_t *) 0x40001010) // Status Register (Offset 0x10)
 #define TIM6_PSC        *((volatile uint32_t *) 0x40001028) // Prescaler (Offset 0x28)
 #define TIM6_ARR        *((volatile uint32_t *) 0x4000102C) // Auto-Reload Register (Offset 0x2C)
#define TIM6_DIER		 *((volatile uint32_t *) 0x4000100C)

 // --- PI 2: TIM15 (IC/OC Timer) Registers ---
// Base: 0x4001 4000
#define RCC_APB2ENR_TIM15EN (1 << 16) // TIM15 Clock Enable Bit


typedef struct {
    volatile uint32_t ISR;      // 0x00
    volatile uint32_t IER;      // 0x04
    volatile uint32_t CR;       // 0x08
    volatile uint32_t CFGR;     // 0x0C
    volatile uint32_t CFG2;     // 0x10
    volatile uint32_t SMPR1;    // 0x14
    volatile uint32_t SMPR2;    // 0x18
    uint32_t reserved2;         // 0x1C
    volatile uint32_t TR1;      // 0x20
    volatile uint32_t TR2;      // 0x24
    volatile uint32_t TR3;      // 0x28
    uint32_t reserved3;         // 0x2C
    volatile uint32_t SQR1;     // 0x30
    volatile uint32_t SQR2;     // 0x34
    volatile uint32_t SQR3;     // 0x38
    volatile uint32_t SQR4;     // 0x3C
    volatile uint32_t DR;       // 0x40
    uint32_t reserved4[2];      // 0x44-0x48
    volatile uint32_t JSQR;     // 0x4C
    uint32_t reserved5[4];      // 0x50-0x5C
    volatile uint32_t OFR1;     // 0x60
    volatile uint32_t OFR2;     // 0x64
    volatile uint32_t OFR3;     // 0x68
    volatile uint32_t OFR4;     // 0x6C
    uint32_t reserved6[4];      // 0x70-0x7C
    volatile uint32_t JDR1;     // 0x80
    volatile uint32_t JDR2;     // 0x84
    volatile uint32_t JDR3;     // 0x88
    volatile uint32_t JDR4;     // 0x8C
    uint32_t reserved7[4];      // 0x90-0x9C
    volatile uint32_t AWD2CR;   // 0xA0
    volatile uint32_t AWD3CR;   // 0xA4
    uint32_t reserved8[2];      // 0xA8-0xAC
    volatile uint32_t DIFSEL;   // 0xB0
    volatile uint32_t CALFACT;  // 0xB4
} ADCType;

typedef struct {
    volatile uint32_t CSR;      // 0x00
    uint32_t reserved1;         // 0x04
    volatile uint32_t CCR;      // 0x08
    volatile uint32_t CDR;      // 0x0C
} ADCCommon;

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;
    volatile uint32_t BRR;
    uint32_t reserved;
    volatile uint32_t SECCFGR;
} GPIO_Struct;

// Register Pointers for Structs
#define GPIOA_PTR       ((GPIO_Struct *) 0x42020000)
#define GPIOB_PTR       ((GPIO_Struct *) 0x42020400)
#define GPIOC_PTR       ((GPIO_Struct *) 0x42020800)
#define ADC1            ((ADCType *) 0x42028000)
#define ADC_COMMON      ((ADCCommon *) 0x42028300)


// TIM15 Structure
typedef struct {
    volatile uint32_t CR1;      //0
    volatile uint32_t CR2;      //4
    volatile uint32_t SMCR;     //8
    volatile uint32_t DIER;     //C
    volatile uint32_t SR;       //10
    volatile uint32_t EGR;      //14
    volatile uint32_t CCMR1;    //18
    uint32_t reserved1;         //1C
    volatile uint32_t CCER;     //20
    volatile uint32_t CNT;      //24
    volatile uint32_t PSC;      //28
    volatile uint32_t ARR;      //2C
    volatile uint32_t RCR;      //30
    volatile uint32_t CCR1;     //34
    volatile uint32_t CCR2;     //38
    uint32_t reserved2[2];      //3C 40
    volatile uint32_t BDTR;     //44
    volatile uint32_t DCR;      //48
    volatile uint32_t DMAR;     //4C
    volatile uint32_t OR1;      //50
    uint32_t reserved3[3];      //54 58 5C
    volatile uint32_t OR2;      //60
} TIM15_General_Purpose_Type;

#define TIM15           ((TIM15_General_Purpose_Type *) 0x40014000)

// --- PI 1 (for PI 3): TIM7 (ADC Trigger Timer) Registers ---
// Base: 0x4000 1400
#define RCC_APB1ENR1_TIM7EN (1 << 5) // TIM7 Clock Enable Bit
#define TIM7_CR1        *((volatile uint32_t *) 0x40001400)
#define TIM7_SR         *((volatile uint32_t *) 0x40001410)
#define TIM7_PSC        *((volatile uint32_t *) 0x40001428)
#define TIM7_ARR        *((volatile uint32_t *) 0x4000142C)

// --- PI 2: TIM16 (Buzzer PWM Timer) Registers ---
// Base: 0x4001 4400
#define RCC_APB2ENR_TIM16EN (1 << 17) // TIM16 Clock Enable Bit

// TIM16 Structure (different from TIM15)
typedef struct {
    volatile uint32_t CR1;      //0
    volatile uint32_t CR2;      //4
    uint32_t reserved1;         //8
    volatile uint32_t DIER;     //C
    volatile uint32_t SR;       //10
    volatile uint32_t EGR;      //14
    volatile uint32_t CCMR1;    //18
    uint32_t reserved2;         //1C
    volatile uint32_t CCER;     //20
    volatile uint32_t CNT;      //24
    volatile uint32_t PSC;      //28
    volatile uint32_t ARR;      //2C
    volatile uint32_t RCR;      //30
    volatile uint32_t CCR1;     //34
    uint32_t reserved3[3];      //38 3C 40
    volatile uint32_t BDTR;     //44
    volatile uint32_t DCR;      //48
    volatile uint32_t DMAR;     //4C
    volatile uint32_t OR1;      //50
    uint32_t reserved4[3];      //54 58 5C
    volatile uint32_t OR2;      //60
} TIM16_General_Purpose_Type;

#define TIM16           ((TIM16_General_Purpose_Type *) 0x40014400)

// --- NVIC (Nested Vectored Interrupt Controller) Registers ---
#define NVIC_ISER0      *((volatile uint32_t *) 0xE000E100) // Interrupt Set Enable Register 0
#define ISER2           *((volatile uint32_t *) 0xE000E108) // Interrupt Set Enable Register 2
#define TIM15_IRQn      5   // TIM15 interrupt number (bit 5 in ISER2)
#define TIM6_IRQn		17
#define NVIC_ISER1      *((volatile uint32_t *) 0xE000E104)

 //=============================================================================
 // 2. Global Variables
 //=============================================================================

 // Defines the operational states of the alarm logic
 typedef enum {
     ALARM_STATE_IDLE,       // Waiting for an initial trigger (vibration)
     ALARM_STATE_MONITORING  // Vibration detected, now checking for acceleration
 } AlarmState;

 typedef enum {
     KEYPAD_STATE_IDLE,       // Waiting for an initial trigger (vibration)
     KEYPAD_STATE_FIRST_CHECK,  // Vibration detected, now checking for acceleration
	 KEYPAD_STATE_WAIT_RELEASE,
 } KeypadState;

 // A structure to hold the raw 3-axis data from the accelerometer
 typedef struct {
     int x;
     int y;
     int z;
 } AccelerometerData;

 char g_last_read_key = '\0';
 volatile uint32_t g_keypad_tick_counter = 0;
 KeypadState g_keypad_state = KEYPAD_STATE_IDLE;

 volatile uint16_t adc_raw_values[3] = {0, 0, 0};
 volatile float acceleration_values[3] = {0.0, 0.0, 0.0};
 volatile uint8_t adc_seq_index = 0;
 volatile uint32_t adc_tick_counter = 0;

 // --- PI 1: Keypad Mapping ---
 // Defines the character for each [row][col] intersection
 // Rows: PF3, PF5, PF13, PF14
 // Cols: PF0, PF1, PF2
 const char g_keymap[4][3] = {
     {'1', '2', '3'}, // Row 1 (PF3)
     {'4', '5', '6'}, // Row 2 (PF5)
     {'7', '8', '9'}, // Row 3 (PF13)
     {'*', '0', '#'}  // Row 4 (PF14)
 };

 // --- PI 1: Keypad pin lookup arrays ---
 const uint16_t g_col_pins[3] = { (1 << 0), (1 << 1), (1 << 2) }; // PF0, PF1, PF2
 const uint16_t g_row_pins[4] = { (1 << 3), (1 << 5), (1 << 13), (1 << 14) }; // PF3, PF5, PF13, PF14

 // The current state of the alarm's logic state machine
 AlarmState g_current_alarm_state = ALARM_STATE_IDLE;

 // Represents the current state of the main alarm (true if enabled/armed, false otherwise)
 bool g_is_alarm_enabled = false;

 // --- PI 1: Keypad buffer ---
 // Stores incoming key presses.
 char g_keypad_buffer[PASSWORD_BUFFER_SIZE];

 // Current index in the keypad input buffer
 int g_keypad_buffer_index = 0;

 // Timer for vibration count reset interval
 uint32_t g_vibration_timer_s = 0;

 // Buffer to store timestamps of vibration pulses for IC analysis
 uint32_t g_vibration_timestamps[VIBRATION_INTERVAL_BUFFER_SIZE] = {0};

 // Current index in the vibration timestamp buffer
 int g_vibration_timestamp_index = 0;

 // Computed irregularity value from vibration pulse intervals
 float g_vibration_irregularity = 0.0f;

 // Number of valid entries in the vibration timestamp buffer (0 to VIBRATION_INTERVAL_BUFFER_SIZE)
 int g_vibration_timestamp_count = 0;

 // Timer for accelerometer reading intervals
 uint32_t g_accelerometer_timer_ms = 0;

 // Sliding window array to store the last N raw acceleration data (x,y,z)
 AccelerometerData g_acceleration_window[ACCELERATION_WINDOW_SIZE] = {{0, 0, 0}};

 // Current index in the acceleration sliding window array
 int g_acceleration_window_index = 0;

 // Number of valid entries in the acceleration window (0 to ACCELERATION_WINDOW_SIZE)
 int g_acceleration_window_count = 0;

 // The computed pattern value from the acceleration window
 float g_acceleration_pattern_value = 0.0f;

 // Stores the raw 3-axis data from the accelerometer
 AccelerometerData g_raw_accel_data = {0, 0, 0};

 // Stores the latest reading from the acceleration sensor
 int g_acceleration_value = 0;

 // System tick counter for timing operations (incremented each main loop)
 uint32_t g_system_tick_ms = 0;

 // Flag to track if buzzer is currently active (alarm triggered)
 bool g_buzzer_active = false;

 // Flag to track if buzzer is currently high pitch or low pitch
 bool high = false;

 //=============================================================================
 // 3. PI 1&2: IMPLEMENTATION - Initialization Functions
 //=============================================================================

 /**
  * @brief Initializes ADC1 for 3-channel sequence (PA1, PB1, PC2).
  * Enables interrupts to update adc_raw_values in background.
  */
 void init_ADC(void)
 {
     // 1. Setup GPIOs for Analog Mode
     RCC_AHB2ENR |= (1 << 0) | (1 << 1) | (1 << 2); // Enable GPIO A, B, C
     GPIOA_PTR->MODER |= (3 << 2);       // PA1 Analog
     GPIOB_PTR->MODER |= (3 << 2);       // PB1 Analog
     GPIOC_PTR->MODER |= (3 << 4);       // PC2 Analog

     // 2. Enable ADC Clock
     RCC_AHB2ENR |= (1 << 13);

     // 3. Power up ADC (Deep Power Down exit)
     ADC1->CR &= ~(1 << 29); // DEEPPWD = 0
     ADC1->CR |= (1 << 28);  // ADVREGEN = 1

     // 4. Clock Configuration
     RCC_CCIPR1 |= (3 << 28); // ADCSEL
     ADC_COMMON->CCR |= (3 << 16); // CKMODE

     // 5. Sampling Time Configuration
     ADC1->SMPR1 |= (7 << 18); // Ch 6 (PA1)
     ADC1->SMPR1 |= (7 << 9);  // Ch 3 (PC2)
     ADC1->SMPR2 |= (7 << 18); // Ch 16 (PB1)

     // 6. Sequence Configuration (SQR1)
     ADC1->SQR1 &= ~(0xF);
     ADC1->SQR1 |= 2; // Length = 3 conversions (L=2 means 3)

     ADC1->SQR1 &= ~(0x1FFFF << 6);
     ADC1->SQR1 |= (6 << 6);   // SQ1 = 6 (PA1)
     ADC1->SQR1 |= (16 << 12); // SQ2 = 16 (PB1)
     ADC1->SQR1 |= (3 << 18);  // SQ3 = 3 (PC2)

     // 7. Calibration
     ADC1->CR |= (1 << 31); // ADCAL
     while ((ADC1->CR & (1 << 31)) != 0);

     // 8. Enable ADC
     ADC1->CR |= (1 << 0); // ADEN
     while ((ADC1->ISR & (1 << 0)) == 0); // Wait for ADRDY

     // 9. Start & Interrupts
     ADC1->IER |= (1 << 2);              // Enable EOC (End of Conversion) interrupt
     NVIC_ISER1 |= (1 << 5);             // Enable ADC IRQ in NVIC (IRQ 18?)

     ADC1->CR |= (1 << 2);               // ADSTART - Start first conversion sequence
 }


 /**
  * @brief PI 1: Initializes TIM6 to generate an overflow event every 1ms.
  * Assumes a 4MHz default MSI clock.
  * (4,000,000 Hz / (PSC+1)) / (ARR+1) = 1000 Hz (1ms)
  * (4,000,000 / 40) / 100 = 1000
  */
 void init_TIM6_for_1ms(void) {
     RCC_APB1ENR1 |= RCC_APB1ENR1_TIM6EN; // Enable TIM6 clock
     TIM6_PSC = 39;            // Set Prescaler to 39 (divides clock by 40)
     TIM6_ARR = 99;            // Set Auto-Reload to 99 (counts 100 ticks)
     TIM6_DIER |= (1 << 0);
     NVIC_ISER1 |= (1 << (TIM6_IRQn));
     TIM6_CR1 &= ~(1 << 1);    // Enable Update Event (UDIS=0)
     TIM6_CR1 |= (1 << 0);     // Enable Counter (CEN=1)
 }

/**
 * @brief PI 2: Initializes TIM15 for Input Capture (IC).
 * Configures vibration sensor monitoring with interrupt-driven timestamping.
 * This timer is STARTED for Input Capture.
 */
void init_PI2_Timer(void) {
    // 1. Enable TIM15 clock (Advanced Timer)
    RCC_APB2ENR |= RCC_APB2ENR_TIM15EN;

    // 2. Configure TIM15 for Input Capture
    // Set prescaler for good resolution (4MHz / 4000 = 1kHz = 1ms resolution)
    TIM15->PSC = 3999;      // Prescaler: divide by 4000
    TIM15->ARR = 1999;      // 1 second period for overflow reset

    //
    TIM15->CCR2 = 999;

    // 3. Configure Channel 1 for Input Capture
    // CC1S = 01 (IC1 mapped to TI1)
    TIM15->CCMR1 &= ~(0b11 << 0);    // Clear CC1S bits
    TIM15->CCMR1 |= (0b01 << 0);     // CC1S = 01 (input capture)

    // 4. Configure capture on both edges for maximum sensitivity
    TIM15->CCER &= ~(0b1111 << 0);   // Clear all CC1 bits
    TIM15->CCER |= (1 << 0);         // CC1E = 1 (enable capture)
    TIM15->CCER |= (1 << 1);         // CC1P = 1 (falling edge)
    TIM15->CCER |= (1 << 3);         // CC1NP = 1 (both edges when combined with CC1P)

    // 5. Enable capture and overflow interrupts
    TIM15->DIER |= (1 << 1);         // CC1IE = 1 (capture interrupt)
    TIM15->DIER |= (1 << 0);         // UIE = 1 (update interrupt for overflow)

    TIM15->DIER |= (1 << 2);     // CC2IE = 1 for frequency toggle

    // 6. Clear all status flags
    TIM15->SR = 0;

    // 7. Enable TIM15 interrupt in NVIC (IRQ 37, bit 5 in ISER2)
    ISER2 |= (1 << TIM15_IRQn);

    // 8. Start the timer for Input Capture
    TIM15->CR1 |= (1 << 0);          // CEN = 1 (enable counter)
}

/**
 * @brief PI 1: Initializes TIM7 for PI 3 (ADC Sampling Trigger).
 * Enables the clock and sets values for ACCELEROMETER_READ_INTERVAL_MS.
 * This timer is NOT STARTED (CEN=0).
 */
void init_PI3_Timer(void) {
    // 1. Enable TIM7 clock (Basic Timer)
    RCC_APB1ENR1 |= RCC_APB1ENR1_TIM7EN;

    // 2. Set PSC/ARR for 200ms (ACCELEROMETER_READ_INTERVAL_MS)
    // 0.2s = (PSC+1) * (ARR+1) / 4,000,000 Hz
    // (PSC+1) * (ARR+1) = 800,000
    // Let PSC = 3999 (div by 4000), ARR = 199 (count 200)
    // (3999+1) * (199+1) = 4000 * 200 = 800,000. Correct.
    TIM7_PSC = 3999;
    TIM7_ARR = 199;

    // 3. Ensure Update Event is enabled (UDIS=0)
    TIM7_CR1 &= ~(1 << 1);

    // 4. DO NOT START THE TIMER (CEN remains 0).
    // PI 3 will enable this (or link it to ADC) when ready.
}

/**
 * @brief PI 2: Initializes TIM16 for buzzer PWM output.
 * Configures TIM16_CH1 for PWM generation at 1kHz frequency.
 */
void init_buzzer_timer(void) {
    // 1. Enable TIM16 clock (Advanced Timer)
    RCC_APB2ENR |= RCC_APB2ENR_TIM16EN;

    // 2. Configure TIM16 for 1kHz PWM (audible buzzer frequency)
    // 4MHz / (PSC+1) / (ARR+1) = 1kHz
    // 4MHz / 4 / 1000 = 1kHz
    TIM16->PSC = 3;     // Prescaler: divide by 4 (1MHz timer frequency)
    TIM16->ARR = 999;   // Auto-reload: 1000 counts (1kHz PWM frequency)

    // 3. Configure Channel 1 for PWM Mode 1
    TIM16->CCMR1 &= ~(0b11 << 0);   // Clear CC1S (output mode)
    TIM16->CCMR1 |= (0b110 << 4);   // PWM mode 1 (OC1M = 110)

    // 4. Set 50% duty cycle for buzzer tone
    TIM16->CCR1 = 500;  // 50% duty cycle (500/1000)

    // 5. Configure output polarity (active high)
    TIM16->CCER &= ~(1 << 1);       // CC1P = 0 (active high)

    // 6. Enable main output (required for advanced timers)
    TIM16->BDTR |= (1 << 15);       // MOE = 1 (Main Output Enable)

    // 7. Channel 1 output is initially disabled (buzzer off)
    TIM16->CCER &= ~(1 << 0);       // CC1E = 0 (disable output)

    // 8. Start the timer (but output is disabled)
    TIM16->CR1 |= (1 << 0);         // CEN = 1 (enable counter)
}

// PI 2: Initializes the buzzer on PA6 for TIM16_CH1 PWM Output
void init_buzzer_gpio(void) {
    // PA6 clock is already enabled in init_rgb_led() (GPIOA clock)

    // Configure PA6 as alternate function for TIM16_CH1
    GPIOA_MODER &= ~(3 << (6*2));  // Clear mode bits for PA6
    GPIOA_MODER |= (2 << (6*2));   // Set PA6 to alternate function mode (10)

    // Set PA6 alternate function to TIM16 (AF14)
    // GPIOA_AFRL for pins 0-7, PA6 uses bits 24-27
    GPIOA_AFRL &= ~(0xF << (6*4));  // Clear AF bits for PA6
    GPIOA_AFRL |= (0xE << (6*4));   // Set AF14 for TIM16

}

 /**
  * @brief PI 1: Initializes GPIOF for the 4x3 Keypad.
  * - Cols (PF0, PF1, PF2) as Output
  * - Rows (PF3, PF5, PF13, PF14) as Input with Pull-up
  */
 void init_keypad(void) {
     RCC_AHB2ENR |= (1 << 5); // Enable GPIOF clock

     // 1. Set Keypad Column Pins (PF0, PF1, PF2) to Output ('01')
     GPIOF_MODER &= ~( (3 << (0*2)) | (3 << (1*2)) | (3 << (2*2)) );
     GPIOF_MODER |=  ( (1 << (0*2)) | (1 << (1*2)) | (1 << (2*2)) );

     // 2. Set Keypad Row Pins (PF3, PF5, PF13, PF14) to Input ('00')
     GPIOF_MODER &= ~( (3 << (3*2)) | (3 << (5*2)) | (3 << (13*2)) | (3 << (14*2)) );

     // 3. Enable Internal Pull-Up Resistors for Row Pins ('01')
     GPIOF_PUPDR &= ~( (3 << (3*2)) | (3 << (5*2)) | (3 << (13*2)) | (3 << (14*2)) );
     GPIOF_PUPDR |=  ( (1 << (3*2)) | (1 << (5*2)) | (1 << (13*2)) | (1 << (14*2)) );

     // 4. Initialize keypad buffer
     g_keypad_buffer[0] = '\0';
     g_keypad_buffer_index = 0;
 }

 /**
  * @brief PI 1&2: Initializes GPIOA for the status LEDs.
  * - PA3 (Green), PA4 (Blue), PA5 (Red) as Output (PA2 used for vibration sensor)
  * - Sets initial state to OFF (Red ON, Green OFF, Blue OFF)
  */
 void init_rgb_led(void) {
     RCC_AHB2ENR |= (1 << 0); // Enable GPIOA clock

     // 1. Set LED Pins (PA3, PA4, PA5) to Output ('01') - PA2 used for TIM15
     GPIOA_MODER &= ~( (3 << (3*2)) | (3 << (4*2)) | (3 << (5*2)) ); // Clear modes for PA3, PA4, PA5
     GPIOA_MODER |=  ( (1 << (3*2)) | (1 << (4*2)) | (1 << (5*2)) ); // Set PA3, PA4, PA5 to Output

     // 2. Set initial state: System OFF (Red ON, Green OFF, Blue OFF)
     // Assumes Active-LOW wiring (LOW = ON, HIGH = OFF)
     GPIOA_ODR &= ~(1 << 5); // PA5 (Red) LOW -> ON
     GPIOA_ODR |=  (1 << 3); // PA3 (Green) HIGH -> OFF
     GPIOA_ODR |=  (1 << 4); // PA4 (Blue) HIGH -> OFF
 }

 //=============================================================================
 // 4. IMPLEMENTATION & MOCK FUNCTIONS (PI 2 Complete, PI 3 Mock)
 //=============================================================================

 // MOCK (PI 3): Initializes the accelerometer sensor
 void init_accelerometer(void) {
 }

 // PI 2: Initializes the vibration sensor on PA2 for TIM15_CH1 Input Capture
void init_vibration_sensor(void) {
    // PA2 clock is already enabled in init_rgb_led() (GPIOA clock)

    // Configure PA2 as alternate function for TIM15_CH1
    GPIOA_MODER &= ~(3 << (2*2));  // Clear mode bits for PA2
    GPIOA_MODER |= (2 << (2*2));   // Set PA2 to alternate function mode (10)

    // Set PA2 alternate function to TIM15 (AF14)
    // GPIOA_AFRL for pins 0-7, PA2 uses bits 8-11
    GPIOA_AFRL &= ~(0xF << (2*4));  // Clear AF bits for PA2
    GPIOA_AFRL |= (0xE << (2*4));   // Set AF14 for TIM15

    // Enable pull-up resistor for PA2 to prevent floating input noise
    GPIOA_PUPDR &= ~(0b11 << (2 * 2));    // Clear pull bits
    GPIOA_PUPDR |= (0b01 << (2 * 2));     // Enable pull-up resistor

}

 // PI 2: Initializes the buzzer hardware (GPIO + Timer)
 void init_buzzer(void) {
     init_buzzer_gpio();   // Configure PA6 for TIM16_CH1
     init_buzzer_timer();  // Configure TIM16 for PWM
 }

 // MOCK (PI 3): Initializes the Bluetooth module
 void init_bluetooth_module(void) {
 }

 // MOCK (PI 3): read the value from the accelerometer via ADC
 // Updated to use the ADC Globals (Reading logic commented out for now)
 void read_accelerometer_adc(void) {
     // 1. Read values from ISR-updated array
     // float accelx = (float)adc_raw_values[0] / 409.6 - 5.0;
     // float accely = (float)adc_raw_values[1] / 409.6 - 5.0;
     // float accelz = (float)adc_raw_values[2] / 220.0 - 5.55;
     ADC1->CR |= (1 << 2); // ADSTART


     // 2. Update the logic struct (using mock logic for now to keep code compiling)
     g_raw_accel_data.x = adc_raw_values[0];
     g_raw_accel_data.y = adc_raw_values[1];
     g_raw_accel_data.z = adc_raw_values[2];
 }

 // PI 2: Control buzzer PWM output
 void set_buzzer_state(bool activate) {
     g_buzzer_active = activate;
     if (activate) {
         // Enable PWM output on TIM16_CH1
         TIM16->CCER |= (1 << 0);    // CC1E = 1 (enable output)
     } else {
         // Disable PWM output on TIM16_CH1
         TIM16->CCER &= ~(1 << 0);   // CC1E = 0 (disable output)
     }
 }

 // MOCK (PI 3): check for incoming commands from Bluetooth
 void check_bluetooth_commands(void) {
     // In a real implementation, this would parse serial data.
 }

 // MOCK (PI 3): Adds new raw acceleration data to the sliding window
 void add_acceleration_to_window(AccelerometerData accel_data) {
     g_acceleration_window_count++;
 }

 // MOCK (PI 3): Computes a pattern value from the raw acceleration window data
 float compute_acceleration_pattern(void) {
     return 1.0f; // Mock pattern value
 }


 // PI 2: Timestamp-based sliding window vibration detection
 float compute_vibration_irregularity(void) {

    if (g_vibration_timestamp_count < VIBRATION_PULSE_THRESHOLD) {
        return 0.0f;
    }
    // Most recent pulse timestamp
    uint32_t latest = g_vibration_timestamps[
        (g_vibration_timestamp_index - 1 + VIBRATION_INTERVAL_BUFFER_SIZE) %
        VIBRATION_INTERVAL_BUFFER_SIZE
    ];

    int pulses = 0;
    uint32_t window = VIBRATION_RESET_INTERVAL_S * 1000000; // Sliding window in µs

    // Count pulses within the window, going backward in the circular buffer
    for (int i = 0; i < VIBRATION_INTERVAL_BUFFER_SIZE; i++) {

        uint32_t t = g_vibration_timestamps[
            (g_vibration_timestamp_index - 1 - i + VIBRATION_INTERVAL_BUFFER_SIZE) %
            VIBRATION_INTERVAL_BUFFER_SIZE
        ];

        // Stop once timestamp is older than the window
        if (latest - t > window)
            break;

        pulses++;
    }

    // Trigger if enough pulses occur in the window
    return pulses >= VIBRATION_PULSE_THRESHOLD ? 1.0f : 0.0f;
}


 //=============================================================================
 // 5. PI 1&2: IMPLEMENTATION - Core Functionality
 //=============================================================================

 /**
   * @brief PI 1&2: Updates the Red (PA5), Green (PA3), and Blue (PA4) LEDs based on system state.
   * (Active-LOW: Pin LOW = ON, Pin HIGH = OFF)
   * @param is_on Is the system ON (true) or OFF (false)?
 */
 void update_system_leds(bool is_on) {
     if (is_on) {
         // System ON: Green ON, Red OFF, Blue OFF
         GPIOA_ODR &= ~(1 << 3); // PA3 (Green) LOW -> ON
         GPIOA_ODR |=  (1 << 5); // PA5 (Red) HIGH -> OFF
         GPIOA_ODR |=  (1 << 4); // PA4 (Blue) HIGH -> OFF
     } else {
         // System OFF: Green OFF, Red ON, Blue OFF
         GPIOA_ODR |=  (1 << 3); // PA3 (Green) HIGH -> OFF
         GPIOA_ODR &= ~(1 << 5); // PA5 (Red) LOW -> ON
         GPIOA_ODR |=  (1 << 4); // PA4 (Blue) HIGH -> OFF
     }
 }

 /**
   * @brief PI 2: Updates LEDs for monitoring state (Blue ON, others OFF)
   * Called when system enters ALARM_STATE_MONITORING
 */
 void set_monitoring_led_state(void) {
     // Monitoring state: Blue ON, Red OFF, Green OFF
     GPIOA_ODR |=  (1 << 3); // PA3 (Green) HIGH -> OFF
     GPIOA_ODR |=  (1 << 5); // PA5 (Red) HIGH -> OFF
     GPIOA_ODR &= ~(1 << 4); // PA4 (Blue) LOW -> ON
 }

 char read_keypad_input(void) {
     const uint16_t ALL_COLS = g_col_pins[0] | g_col_pins[1] | g_col_pins[2];

     for (int c = 0; c < 3; c++) { // Iterate through all columns
         // 1. Set all columns HIGH
         GPIOF_ODR |= ALL_COLS;
         // 2. Set the current column LOW
         GPIOF_ODR &= ~(g_col_pins[c]);

         // 3. Read all rows
         uint32_t idr_val = GPIOF_IDR;

         for (int r = 0; r < 4; r++) { // Iterate through all rows
             // 4. Check if a key is pressed (row pin is pulled LOW)
             if (!(idr_val & g_row_pins[r])) {
                     GPIOF_ODR |= ALL_COLS; // Reset columns
                     return g_keymap[r][c]; // Return the pressed key
             }
         }
     }

     GPIOF_ODR |= ALL_COLS; // Ensure all columns are HIGH before exiting
     return '\0'; // No key pressed
 }


  /**
   * @brief PI 1&2: Processes keypad input, buffers keys, and checks password.
   * This is the core logic for PI 1, matching the project flowchart.
   */
  void process_keypad_input_with_password(void) {
	 char key;
	 switch (g_keypad_state) {
	 case KEYPAD_STATE_IDLE:
			 // 1. Instant scan (no delay inside)
			 key = read_keypad_input();

			 if (key == '\0') {
				 return; // Nothing pressed, exit function immediately
			 }
			 else {
				 // Key detected!
				 g_last_read_key = key;
				 g_keypad_tick_counter = 0; // Reset timer
				 g_keypad_state = KEYPAD_STATE_FIRST_CHECK;

				 // DEBUG: Turn on Blue LED to prove we found a key
				 GPIOA_ODR &= ~(1 << 4);
			 }
			 break;

		 case KEYPAD_STATE_FIRST_CHECK:
			 // This code runs every loop now until counter >= 50

			 // Check if 50ms (debouncing time) has passed
			 // NOTE: 1000ms is 1 second, that is too long for a button press!
			 // Use 50ms for debouncing.
			 if(g_keypad_tick_counter >= 50) {

				 // Double check the key
				 key = read_keypad_input();

				 if (key == g_last_read_key) {
					  // Confirmed! It wasn't noise.
					  // Handle the password logic here...
					  // handle_password(key);

					  // Move to wait release so we don't spam input
					  g_keypad_state = KEYPAD_STATE_WAIT_RELEASE;
				 } else {
					  // It was noise, go back to IDLE
					  g_keypad_state = KEYPAD_STATE_IDLE;
					  // Turn off Blue LED
					  GPIOA_ODR |= (1 << 4);
				 }
			 }
			 break;

		 case KEYPAD_STATE_WAIT_RELEASE: // Actually "WAIT_RELEASE"
			 key = read_keypad_input();
			 if (key == '\0') {
				 // User let go of button
				 g_keypad_state = KEYPAD_STATE_IDLE;
				 // Turn off Blue LED
				 GPIOA_ODR |= (1 << 4);


				 if (g_last_read_key == '#') {
					 // ENTER ('#') was pressed, check the password
					 if (strcmp(g_keypad_buffer, CORRECT_PASSWORD) == 0) {
						 // --- PASSWORD CORRECT ---
						 g_vibration_timestamp_index = 0;
						 g_vibration_timestamp_count = 0;
						 // Clear the entire vibration timestamp circular buffer
						 for (int i = 0; i < VIBRATION_INTERVAL_BUFFER_SIZE; i++) {
							 g_vibration_timestamps[i] = 0;
						 }
						 // Toggle the system state (ON <-> OFF)
						 g_is_alarm_enabled = !g_is_alarm_enabled;
						 g_current_alarm_state = ALARM_STATE_IDLE;
						 set_buzzer_state(false);
						 // Update the LEDs to reflect the new state
						 update_system_leds(g_is_alarm_enabled);
					 }

					 // Clear the buffer regardless of password correctness
					 g_keypad_buffer_index = 0;
					 g_keypad_buffer[0] = '\0';

				 } else if (g_last_read_key == '*') {
					 // CLEAR ('*') was pressed
					 g_keypad_buffer_index = 0;
					 g_keypad_buffer[0] = '\0';

				 } else if (g_last_read_key >= '0' && g_last_read_key <= '9') {
					 // A digit was pressed, add it to the buffer
					 if (g_keypad_buffer_index < (PASSWORD_BUFFER_SIZE - 1)) {
						 g_keypad_buffer[g_keypad_buffer_index++] = g_last_read_key;
						 g_keypad_buffer[g_keypad_buffer_index] = '\0'; // Null-terminate the string
					 }
				 }
			 }
			 break;

	 }


 }

 // PI 2 & 3: Manages the alarm logic (PI 2: vibration complete, PI 3: acceleration mock)
 void update_alarm_logic(void) {
     g_system_tick_ms += 100;

     // No more artificial timer - timestamps handle the 5-second window automatically

     switch (g_current_alarm_state) {
         case ALARM_STATE_IDLE:
        	 g_vibration_irregularity = compute_vibration_irregularity(); // PI 2

             if (g_vibration_irregularity >= VIBRATION_IRREGULARITY_THRESHOLD) {
                 g_current_alarm_state = ALARM_STATE_MONITORING;
                 set_monitoring_led_state(); // Turn on blue LED
             }
             break;

         case ALARM_STATE_MONITORING:
             //read_accelerometer_adc(); // PI 3
             //add_acceleration_to_window(g_raw_accel_data); // PI 3
        	 if (adc_tick_counter >= 1000) {
        		 adc_tick_counter = 0;
        		 ADC1->CR |= (1 << 2);
				 g_acceleration_pattern_value = compute_acceleration_pattern(); // PI 3

				 if (g_acceleration_pattern_value >= 1.0 + ACCELERATION_PATTERN_THRESHOLD) {
					 //set_buzzer_state(true); // PI 2
					 // From project.pdf: Buzzer stays ON until correct password is entered
				 } else if (!g_buzzer_active) {
					 // Only return to idle if buzzer is not active
					 //g_current_alarm_state = ALARM_STATE_IDLE;
					 //update_system_leds(g_is_alarm_enabled); // Restore normal LED state
				 }
        	 }
             break;
     }
 }

 void TIM6_IRQHandler(void) {
	 if (TIM6_SR & (1 << 0)) {
	         TIM6_SR &= ~(1 << 0); // Clear Flag
	         g_keypad_tick_counter++;
	         adc_tick_counter++;
	 }
 }


 // ADC1_2: Accelerometer Reading
 // This ISR runs automatically when ADC conversion is done
 void ADC1_2_IRQHandler(void)
 {
     // Check for End of Conversion (EOC)
     if ((ADC1->ISR & (1 << 2)) != 0)
     {
         // Store the result
         adc_raw_values[adc_seq_index] = ADC1->DR;
         adc_seq_index++;

         // If we have read all 3 channels in the sequence
         if (adc_seq_index >= 3)
         {
             adc_seq_index = 0;
             acceleration_values[0] = (float)adc_raw_values[0] / 409.6 - 5.0;
             acceleration_values[1] = (float)adc_raw_values[1] / 409.6 - 5.0;
             acceleration_values[2] = (float)adc_raw_values[2] / 220.0 - 5.55;
             // Restart the sequence for continuous updating
             //ADC1->CR |= (1 << 2); // ADSTART
         }
     }
 }

//=============================================================================
// 6. PI 2: TIM15 Interrupt Handler for Input Capture
//=============================================================================

// Global interrupt counter for debugging
/**
 * @brief TIM15 Interrupt Handler - Captures vibration pulse timestamps
 * This ISR is called on every edge of the vibration sensor
 */
void TIM15_IRQHandler(void) {

    // Only process vibration if alarm is enabled
	if (!g_is_alarm_enabled) {
	    if (TIM15->SR & (1 << 1)) {        // CC1IF
	        TIM15->CCR1;
	    }
	    if (TIM15->SR & (1 << 0)) {        // UIF
	        TIM15->SR &= ~(1 << 0);
            g_vibration_timestamp_index = 0;
            g_vibration_timestamp_count = 0;
            // Clear the entire vibration timestamp circular buffer
            for (int i = 0; i < VIBRATION_INTERVAL_BUFFER_SIZE; i++) {
                g_vibration_timestamps[i] = 0;
            }
	    }
	    if (TIM15->SR & (1 << 2)) {        // CC2IF  <-- FIX HERE
	        TIM15->SR &= ~(1 << 2);
	    }
	    return;
	}
	else {

        // Check for timer overflow (UIF)
        if (TIM15->SR & (1 << 0)) {
            TIM15->SR &= ~(1 << 0);  // Clear overflow flag
            g_vibration_timestamp_index = 0;
            g_vibration_timestamp_count = 0;
            // Clear the entire vibration timestamp circular buffer
            for (int i = 0; i < VIBRATION_INTERVAL_BUFFER_SIZE; i++) {
                g_vibration_timestamps[i] = 0;
            }
        }
        // Check if CC1 interrupt flag is set (capture occurred)
        if (TIM15->SR & (1 << 1)) {
            // Clear the interrupt flag by reading CCR1
            uint32_t timestamp = TIM15->CCR1;

            g_vibration_timestamp_count++;

            // Add timestamp to circular buffer
            g_vibration_timestamps[g_vibration_timestamp_index] = timestamp;
            g_vibration_timestamp_index = (g_vibration_timestamp_index + 1) % VIBRATION_INTERVAL_BUFFER_SIZE;
        }
        // ---- NEW: CC2 Output Compare interrupt (1-second toggle) ----
        if (TIM15->SR & (1 << 2)) {      // CC2IF triggered?
            TIM15->SR &= ~(1 << 2);      // Clear CC2 flag

            // Toggle buzzer frequency
            high = !high;

            if (high) {
                // Set 1000 Hz
                TIM16->ARR  = 999;
                TIM16->CCR1 = 500;
            } else {
                // Set 500 Hz
                TIM16->ARR  = 1999;
                TIM16->CCR1 = 1000;
            }
        }

    }
}


 //=============================================================================
 // 7. Main Program Loop
 //=============================================================================

 int main() {
	 CPACR |= (0xF <<20);
     // --- PI 1&2: Initialize Hardware ---
     init_keypad();         // PI 1: Init Keypad (GPIOF)
     init_rgb_led();        // PI 1: Init LEDs (GPIOA)
     init_TIM6_for_1ms();   // PI 1: Init Timer (TIM6)
     init_PI2_Timer();      // PI 2: Init Timer (TIM15 for vibration IC)
     init_PI3_Timer();      // PI 3: Init Timer (TIM7 for ADC trigger)

     // --- Initialize Peripherals ---
     init_accelerometer();    // PI 3: Mock
     init_vibration_sensor(); // PI 2: Complete
     init_buzzer();           // PI 2: Complete
     init_ADC();
     init_bluetooth_module(); // PI 3: Mock

     // --- PI 1: Set Initial State ---
     g_is_alarm_enabled = false;             // Start in OFF state
     update_system_leds(g_is_alarm_enabled); // (Red ON, Green OFF)

     // Main system loop (Bare-metal superloop)
     while (1) {
    	 __asm volatile ("wfi");
         // PI 1: Poll keypad and process password logic
         process_keypad_input_with_password();

         // PI 2 & 3: Run sensor logic if the project's alarm is armed
         if (g_is_alarm_enabled) {
             update_alarm_logic();
         }

         // PI 3 Mock: Check for bluetooth commands
         check_bluetooth_commands();
     }

     return 0;
 }
