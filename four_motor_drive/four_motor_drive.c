/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

#include "hardware/irq.h"


#include "four_motor_drive.pio.h"

//
// ---- quadrature encoder interface example
//
// the PIO program reads phase A/B of a quadrature encoder and increments or
// decrements an internal counter to keep the current absolute step count
// updated. At any point, the main code can query the current count by using
// the four_motor_drive_*_count functions. The counter is kept in a full
// 32 bit register that just wraps around. Two's complement arithmetic means
// that it can be interpreted as a 32-bit signed or unsigned value, and it will
// work anyway.
//
// As an example, a two wheel robot being controlled at 100Hz, can use two
// state machines to read the two encoders and in the main control loop it can
// simply ask for the current encoder counts to get the absolute step count. It
// can also subtract the values from the last sample to check how many steps
// each wheel as done since the last sample period.
//
// One advantage of this approach is that it requires zero CPU time to keep the
// encoder count updated and because of that it supports very high step rates.

/*========== PWM Function START ==========*/
// This function takes in a target frequency and gpio pin then initializes that gpio pin to said frequency.
void setFrequency(unsigned int frequency, short int gpioPin) {
    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num = pwm_gpio_to_slice_num(gpioPin);

    // Have an if statement to catch if the frequency is 0, set to 
    if(frequency == 0)
    {
        pwm_set_gpio_level(gpioPin, 0);
    }
    else if(frequency < 2000) // Now we start decreasing duty cycle since frequency can't go below 2000
    {
        double duty = frequency * 9 / 950;
        duty = (398/19) - duty;

        uint32_t clock_num = (125000000) / 2000; // Clock frequency divided by desired frequency gives # of clk cycles

        pwm_set_wrap(slice_num, clock_num - 1); // # of clks, 0 inclusive

        pwm_set_gpio_level(gpioPin, clock_num / duty); // Set to a duty cycle of 50%, hence half are high
    }
    else if (frequency > 20000)
    {
        uint32_t clock_num = (125000000) / frequency; // Clock frequency divided by desired frequency gives # of clk cycles

        pwm_set_wrap(slice_num, clock_num - 1); // # of clks, 0 inclusive
        pwm_set_gpio_level(gpioPin, clock_num - 1);
    }
    else
    {
        uint32_t clock_num = (125000000) / frequency; // Clock frequency divided by desired frequency gives # of clk cycles

        pwm_set_wrap(slice_num, clock_num - 1); // # of clks, 0 inclusive

        pwm_set_gpio_level(gpioPin, clock_num / 2); // Set to a duty cycle of 50%, hence half are high
    }
    // Set the PWM running
    pwm_set_enabled(slice_num, true);
}
/*=========== PWM Function END ===========*/

// Direction INSTRUCTIONS:
    // When facing the wheel head on, a 1 direction means to turn clockwise/right

void setMotor(bool dir, char PWM1, char PWM2, char EN, char ENB, float frequency) {
    gpio_put(EN, 0);
    gpio_put(ENB, 0);
    setFrequency(0, PWM1);
    setFrequency(0, PWM2);
    if(dir == 0) // Rotate clockwise
    {
        setFrequency(frequency, PWM1);
        setFrequency(0, PWM2);
        gpio_put(EN, 1);
    }
    else if(dir == 1) // rotate counter-clockwise
    {
        setFrequency(frequency, PWM2);
        setFrequency(0, PWM1);
        gpio_put(EN, 1);
    }
}

// int main() {
//     char PWM1[] = {5, 9};
//     char PWM2[] = {4, 8};
//     char EN[]   = {2, 6};
//     char ENB[]  = {3, 7};
//     int sm0_value, sm1_value, sm2_value, sm3_value = 0;
//     printf("Hello");

//     /* ========================== PIO/Encoder initialization section START ========================== */
//     // Base pin to connect the A phase of the encoder.
//     // The B phase must be connected to the next pin
//     const uint PIN_AB1 = 27;
//     // const uint PIN_AB1 = 10;
//     // const uint PIN_AB2 = 12;
//     const uint PIN_AB2 = 14;
//     //const uint PIN_AB3 = ;
//     //const uint PIN_AB4 = ;

//     stdio_init_all();

//     PIO pio = pio0;
//     //PIO pioo = pio1;
//     const uint sm0 = 0;
//     const uint sm1 = 1;
//     const uint sm2 = 2;
//     const uint sm3 = 3;

//     // we don't really need to keep the offset, as this program must be loaded
//     // at offset 0
//     // The following lines below initialize the pio for the interrupts
//     //four_motor_drive_program_init(pio, sm2, PIN_AB2, 0);
//     //four_motor_drive_program_init(pio, sm3, PIN_AB3, 0);
//     int ENC_vals[] = {0, 0};

//     /* =========================== PIO/Encoder initialization section END =========================== */

//     /* ========================== PID initialization START ========================== */
//     long prevT = 0;
//     float eprev[] = {0, 0};
//     float eintegral[] = {0, 0};

//     // Initializing the PWM for one wheel
//     gpio_init(2);
//     gpio_set_dir(2, GPIO_OUT);
//     gpio_put(2, 0);
//     gpio_init(3);
//     gpio_set_dir(3, GPIO_OUT);
//     gpio_put(3, 0);
    
//     gpio_init(6);
//     gpio_set_dir(6, GPIO_OUT);
//     gpio_put(6, 0);
//     gpio_init(7);
//     gpio_set_dir(7, GPIO_OUT);
//     gpio_put(7, 0);
    

//     gpio_set_function(4, GPIO_FUNC_PWM);
//     gpio_set_function(5, GPIO_FUNC_PWM);

//     gpio_set_function(8, GPIO_FUNC_PWM);
//     gpio_set_function(9, GPIO_FUNC_PWM);

//     setFrequency(0, 4);
//     setFrequency(0, 5);
//     setFrequency(0, 8);
//     setFrequency(0, 9);

//     pio_add_program(pio, &four_motor_drive_program);
//     //pio_add_program(pio1, &four_motor_drive_program);
//     four_motor_drive_program_init(pio, sm0, PIN_AB2, 0);
//     four_motor_drive_program_init27(pio, sm1, PIN_AB1, 0);
//     /* =========================== PID initialization END =========================== */    

//     while (1) {
//         // note: thanks to two's complement arithmetic delta will always
//         // be correct even when new_value wraps around MAXINT / MININT
//         // new_value0 = four_motor_drive_get_count(pio, sm+0);
//         // new_value1 = four_motor_drive_get_count(pio, sm+1);
//         // new_value2 = four_motor_drive_get_count(pio, sm+2);
//         // new_value3 = four_motor_drive_get_count(pio, sm+3);
        

//         // printf("position0 %8d, position1 %8d, position2 %8d, position3 %8d\n", new_value0, new_value1, new_value2, new_value3);
//         // sleep_ms(100);

//         /* ===================== PID loop logic ===================== */
//         int target[] = {-1200, 1200};

//         // PID constants
//         float kp = 1;
//         float kd = 0.0;
//         float ki = 0.05;

//         // time difference
//         clock_t currT_clock = clock();
//         long currT = (long)currT_clock;
//         float deltaT = ((float) (currT - prevT))/(1.0e6);
//         prevT = currT;
//         printf("\ncurrT: %d", currT);
//         printf("\ndeltaT: %f", deltaT);

//         // Read the position
//         int pos[] = {0, 0};
//         pos[0] = four_motor_drive_get_count(pio, sm0);
//         pos[1] = four_motor_drive_get_count(pio, sm1);

//         // error
//         int e[] = {0, 0};
//         float dedt[] = {0, 0};
//         float u[] = {0, 0};
//         float pwr[] = {0, 0};
//         for(int i = 0; i < 2; i++) {
//             e[i] = pos[i] - target[i];

//             // derivative
//             dedt[i] = (e[i]-eprev[i])/(deltaT);

//             // integral
//             eintegral[i] = eintegral[i] +e[i]*deltaT;

//             // control signal
//             u[i] = kp*e[i] + kd*dedt[i] + ki*eintegral[i];

//             printf("\ne: %d", e[i]);
//             printf("\ndedt: %f", dedt[i]);
//             printf("\neintegral: %f", eintegral[i]);
//             printf("\nu: %f", u[i]);

//             // motor power
//             pwr[i]= fabs(u[i]);

//             if( pwr[i] > 20000) {
//                 pwr[i] = 20000;
//             }
//             else if((pwr[i] > 0) && (pwr[i] < 100)) {
//                 pwr[i] = 100;
//             }
//         }

//         // motor direction
//         bool dir[] = {1, 1};
//         if(u[0]<0){
//             dir[0] = 0;
//         }
//         if(u[1]<0){
//             dir[1] = 0;
//         }

//         // signal the motor
//         setMotor(dir[0], PWM1[0], PWM2[0], EN[0], ENB[0], 0);
//         // signal the motor
//         setMotor(dir[1], PWM1[1], PWM2[1], EN[1], ENB[1], 0);

//         for(int i = 0; i < 2; i++) {
                        

//             // store the previous error
//             eprev[i] = e[i];

//             //sleep_ms(1);

//             printf("\nTarget: %d", target[i]);
//             printf(", Position: %d", pos[i]);
//             printf(", Power: %f", pwr[i]);
//             printf(", Direction: %d", dir[i]);
//         }
//     }
// }
/*============================= =============================*/
/*                          PWM TEST                         */
/*============================= =============================*/
// int main() {
//     int sm0_value, sm1_value, sm2_value, sm3_value = 0;
//     printf("Hello");

//     /* ========================== PID initialization START ========================== */
//     long prevT = 0;
//     float eprev = 0;
//     float eintegral = 0;

//     // Initializing the PWM for one wheel
//     int ENA1 = 7; // This is the GPIO pin number for ENA1
//     //gpio_set_function(ENA1, GPIO_FUNC_PWM);
//     gpio_init(2);
//     gpio_set_dir(2, GPIO_OUT);
//     /* =========================== PID initialization END =========================== */    

//     //gpio_set_function(0, GPIO_FUNC_PWM);
//     gpio_set_function(4, GPIO_FUNC_PWM);
//     gpio_set_function(5, GPIO_FUNC_PWM);

//     /// \end::setup_pwm[]
//     while (1) {
//         setFrequency(2000, 5);
//         //gpio_put(2, 1);
//         setFrequency(0, 4);
//         sleep_ms(5000);
//         setFrequency(0, 5);
//         //gpio_put(2, 1);
//         setFrequency(2000, 4);
//         sleep_ms(5000);
//         setFrequency(0, 5);
//         //gpio_put(2, 1);
//         setFrequency(0, 4);
//         sleep_ms(5000);
//     }
// }
/*============================= =============================*/
/*                          ENC TEST                         */
/*============================= =============================*/
// int main() {
//     int sm0_value, sm1_value, sm2_value, sm3_value = 0;
//     printf("Hello");
//     // Base pin to connect the A phase of the encoder.
//     // The B phase must be connected to the next pin
//     const uint PIN_AB1 = 0;
//     const uint PIN_AB2 = 14;
//     const uint PIN_AB3 = 28;
//     const uint PIN_AB4 = 20;

//     stdio_init_all();

//     PIO pio = pio0;
//     //PIO pioo = pio1;
//     const uint sm0 = 0;
//     const uint sm1 = 1;
//     const uint sm2 = 2;
//     const uint sm3 = 3;

//     // we don't really need to keep the offset, as this program must be loaded
//     // at offset 0
//     // The following lines below initialize the pio for the interrupts
//     pio_add_program(pio, &four_motor_drive_program);
//     //pio_add_program(pio1, &four_motor_drive_program);
//     four_motor_drive_program_init(pio, sm0, PIN_AB1, 0);
//     four_motor_drive_program_init(pio, sm1, PIN_AB2, 0);
//     four_motor_drive_program_init27(pio, sm2, PIN_AB3, 0);
//     four_motor_drive_program_init(pio, sm3, PIN_AB4, 0);

//     /// \end::setup_pwm[]
//     while (1) {
//         // Read the position
//         int pos[] = {0, 0, 0, 0};
//         pos[0] = four_motor_drive_get_count(pio, sm0);
//         pos[1] = four_motor_drive_get_count(pio, sm1);
//         pos[2] = four_motor_drive_get_count(pio, sm2);
//         pos[3] = four_motor_drive_get_count(pio, sm3);

//         printf("\nPosition 0: %d", pos[0]);
//         printf("\nPosition 1: %d", pos[1]);
//         printf("\nPosition 2: %d", pos[2]);
//         printf("\nPosition 3: %d", pos[3]);

//         sleep_ms(500);
//     }
// }
/*============================= =============================*/
/*                       WHEELS TEST                         */
/*============================= =============================*/
int main() {
    char PWM1[] = {5, 9, 13, 18};
    char PWM2[] = {4, 8, 12, 19};
    char EN[]   = {2, 6, 10, 26};
    char ENB[]  = {3, 7, 11, 22};
    printf("Hello");

    /* ========================== PIO/Encoder initialization section START ========================== */


    /* =========================== PIO/Encoder initialization section END =========================== */

    /* ========================== PID initialization START ========================== */

    // Initializing the PWM for one wheel

    for(int i = 0; i < 4; i++) {
        gpio_init(EN[i]);
        gpio_set_dir(EN[i], GPIO_OUT);
        gpio_put(EN[i], 0);
        gpio_init(ENB[i]);
        gpio_set_dir(ENB[i], GPIO_OUT);
        gpio_put(ENB[i], 0);
    }
    // gpio_init(2);
    // gpio_set_dir(2, GPIO_OUT);
    // gpio_put(2, 0);
    // gpio_init(3);
    // gpio_set_dir(3, GPIO_OUT);
    // gpio_put(3, 0);
    
    // gpio_init(6);
    // gpio_set_dir(6, GPIO_OUT);
    // gpio_put(6, 0);
    // gpio_init(7);
    // gpio_set_dir(7, GPIO_OUT);
    // gpio_put(7, 0);

    // gpio_init(10);
    // gpio_set_dir(10, GPIO_OUT);
    // gpio_put(10, 0);
    // gpio_init(11);
    // gpio_set_dir(11, GPIO_OUT);
    // gpio_put(11, 0);
    
    // gpio_init(21);
    // gpio_set_dir(21, GPIO_OUT);
    // gpio_put(21, 0);
    // gpio_init(20);
    // gpio_set_dir(20, GPIO_OUT);
    // gpio_put(20, 0);


    for(int i = 0; i < 4; i++) {
        gpio_set_function(PWM2[i], GPIO_FUNC_PWM);
        gpio_set_function(PWM1[i], GPIO_FUNC_PWM);  
    }

    for(int i = 0; i < 4; i++) {
        setFrequency(0, PWM2[i]);
        setFrequency(0, PWM1[i]);
    }

    // gpio_set_function(4, GPIO_FUNC_PWM);
    // gpio_set_function(5, GPIO_FUNC_PWM);

    // gpio_set_function(8, GPIO_FUNC_PWM);
    // gpio_set_function(9, GPIO_FUNC_PWM);

    // gpio_set_function(12, GPIO_FUNC_PWM);
    // gpio_set_function(13, GPIO_FUNC_PWM);

    // gpio_set_function(19, GPIO_FUNC_PWM);
    // gpio_set_function(18, GPIO_FUNC_PWM);

    // setFrequency(0, 4);
    // setFrequency(0, 5);
    // setFrequency(0, 8);
    // setFrequency(0, 9);
    // setFrequency(0, 12);
    // setFrequency(0, 13);
    // setFrequency(0, 19);
    // setFrequency(0, 18);

    /* =========================== PID initialization END =========================== */   
            // signal the motor
    // BIG ASS ARM IS THE FRONT
    setMotor(0, PWM1[0], PWM2[0], EN[0], ENB[0], 30000); //MD1 BACK LEFT Rotate left
    // signal the motor
    setMotor(1, PWM1[1], PWM2[1], EN[1], ENB[1], 30000); //MD2 FRONT LEFT Rotate right
    // signal the motor
    setMotor(0, PWM1[2], PWM2[2], EN[2], ENB[2], 30000); //MD3 BACK RIGHT Rotate left
    // signal the motor
    setMotor(1, PWM1[3], PWM2[3], EN[3], ENB[3], 30000); //MD4 FRONT RIGHT Rotate right

    sleep_ms(30000);

    // BIG ASS ARM IS THE FRONT
    setMotor(0, PWM1[0], PWM2[0], EN[0], ENB[0], 0); //MD1 BACK LEFT Rotate left
    // signal the motor
    setMotor(0, PWM1[1], PWM2[1], EN[1], ENB[1], 0); //MD2 FRONT LEFT Rotate left
    // signal the motor
    setMotor(1, PWM1[2], PWM2[2], EN[2], ENB[2], 0); //MD3 BACK RIGHT Rotate right
    // signal the motor
    setMotor(1, PWM1[3], PWM2[3], EN[3], ENB[3], 0); //MD4 FRONT RIGHT Rotate right 

    while (1) {

        /* ===================== PID loop logic ===================== */
        sleep_ms(5000);

    }
}