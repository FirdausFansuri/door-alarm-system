// Minimal definitions for registers (using absolute addresses)
// Base addresses for peripherals
#define RCC_BASE       0x40023800UL
#define GPIOA_BASE     0x40020000UL
#define GPIOB_BASE     0x40020400UL

// RCC register: AHB1 peripheral clock enable register (offset 0x30)
#define RCC_AHB1ENR    (*(volatile unsigned int *)(RCC_BASE + 0x30))

// Offsets for GPIO registers
#define GPIO_MODER_OFFSET    0x00  // Mode register
#define GPIO_PUPDR_OFFSET    0x0C  // Pull-up/pull-down register
#define GPIO_IDR_OFFSET      0x10  // Input data register
#define GPIO_ODR_OFFSET      0x14  // Output data register
#define GPIO_BSRR_OFFSET     0x18  // Bit set/reset register

// GPIOA registers
#define GPIOA_MODER    (*(volatile unsigned int *)(GPIOA_BASE + GPIO_MODER_OFFSET))
#define GPIOA_PUPDR    (*(volatile unsigned int *)(GPIOA_BASE + GPIO_PUPDR_OFFSET))
#define GPIOA_IDR      (*(volatile unsigned int *)(GPIOA_BASE + GPIO_IDR_OFFSET))

// GPIOB registers
#define GPIOB_MODER    (*(volatile unsigned int *)(GPIOB_BASE + GPIO_MODER_OFFSET))
#define GPIOB_ODR      (*(volatile unsigned int *)(GPIOB_BASE + GPIO_ODR_OFFSET))
#define GPIOB_BSRR     (*(volatile unsigned int *)(GPIOB_BASE + GPIO_BSRR_OFFSET))

// NVIC Application Interrupt and Reset Control Register for system reset
#define NVIC_AIRCR     (*(volatile unsigned int *)0xE000ED0CUL)

// Pin definitions
// Input pins on GPIOA
#define TRIGGER_PIN    0    // PA0
#define RESET_PIN      1    // PA1

// Output pins on GPIOB
#define RED_LED_PIN    0    // PB0
#define GREEN_LED_PIN  1    // PB1
#define SPEAKER_PIN    2    // PB2

// delay function 
void delay_ms(unsigned int ms)
{
    volatile unsigned int count;
    while (ms--)
    {
        count = 16000;  // Adjust this constant based on your system clock frequency.
        while (count--)
        {
            __asm__("nop");
        }
    }
}

// System reset function using NVIC_AIRCR
void system_reset(void)
{
    // Write the key (0x5FA) to bits 31:16 and set SYSRESETREQ (bit 2)
    NVIC_AIRCR = (0x5FA << 16) | (1 << 2);
    while (1);  // Wait for reset to occur
}

int main(void)
{
    unsigned int temp;

    // 1. Enable the clocks for GPIOA and GPIOB.
    // In RCC_AHB1ENR, bit 0 corresponds to GPIOA, and bit 1 corresponds to GPIOB.
    RCC_AHB1ENR |= (1 << 0) | (1 << 1);

    // 2. Configure PA0 (Trigger) and PA1 (Reset) as inputs.
    //    (GPIO mode: 00 = input). Their default state is input, so no changes needed to MODER.
    // 3. Configure internal pull-ups for PA0 and PA1.
    //    For each pin, 01 in PUPDR means pull-up.
    //    - For PA0: bits 1:0; for PA1: bits 3:2.
    temp = GPIOA_PUPDR;
    temp &= ~((3 << (TRIGGER_PIN * 2)) | (3 << (RESET_PIN * 2)));  // Clear bits for PA0 and PA1.
    temp |= ((1 << (TRIGGER_PIN * 2)) | (1 << (RESET_PIN * 2)));     // Set pull-up (01) for both.
    GPIOA_PUPDR = temp;

    // 4. Configure PB0 (Red LED), PB1 (Green LED), and PB2 (Speaker) as outputs.
    //    For each pin, set mode to 01 (general purpose output).
    temp = GPIOB_MODER;
    temp &= ~((3 << (RED_LED_PIN * 2)) | (3 << (GREEN_LED_PIN * 2)) | (3 << (SPEAKER_PIN * 2)));  // Clear mode bits.
    temp |= ((1 << (RED_LED_PIN * 2)) | (1 << (GREEN_LED_PIN * 2)) | (1 << (SPEAKER_PIN * 2)));     // Set to output mode.
    GPIOB_MODER = temp;

    // 5. Ensure all outputs are initially turned off.
    //    Clearing the output bits in the ODR (or using BSRR to reset) will turn them off.
    GPIOB_ODR &= ~((1 << RED_LED_PIN) | (1 << GREEN_LED_PIN) | (1 << SPEAKER_PIN));

    // Main loop
    while (1)
    {
        // Check if the Trigger button (PA0) is pressed (active low)
        if ((GPIOA_IDR & (1 << TRIGGER_PIN)) == 0)
        {
            // Enter alarm state: continue until the Reset button is pressed.
            while (1)
            {
                // Check if the Reset button (PA1) is pressed (active low)
                if ((GPIOA_IDR & (1 << RESET_PIN)) == 0)
                {
                    // Turn off all outputs by resetting the corresponding bits using the BSRR.
                    // The upper half (bits 16-31) of the BSRR register resets the output.
                    GPIOB_BSRR = ((1 << RED_LED_PIN) << 16) | ((1 << GREEN_LED_PIN) << 16) | ((1 << SPEAKER_PIN) << 16);
                    delay_ms(100);
                    system_reset();  // Trigger a system reset.
                }

                // Blink the Red and Green LEDs.
                // For toggling, we check the current state (in ODR) and then set or reset accordingly.

                // Toggle Red LED (PB0)
                if (GPIOB_ODR & (1 << RED_LED_PIN))
                {
                    // Turn off PB0 using the reset half of BSRR.
                    GPIOB_BSRR = (1 << RED_LED_PIN) << 16;
                }
                else
                {
                    // Turn on PB0 using the set half of BSRR.
                    GPIOB_BSRR = (1 << RED_LED_PIN);
                }

                // Toggle Green LED (PB1)
                if (GPIOB_ODR & (1 << GREEN_LED_PIN))
                {
                    GPIOB_BSRR = (1 << GREEN_LED_PIN) << 16;
                }
                else
                {
                    GPIOB_BSRR = (1 << GREEN_LED_PIN);
                }

                // Activate the speaker (PB2) for a brief beep.
                // Turn on speaker.
                GPIOB_BSRR = (1 << SPEAKER_PIN);
                delay_ms(100);
                // Turn off speaker.
                GPIOB_BSRR = (1 << SPEAKER_PIN) << 16;
                delay_ms(100);
            }  // End of alarm state loop.
        }  // End of trigger check.
    }  // End of main loop.

    return 0;
}
