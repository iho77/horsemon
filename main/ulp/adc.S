/* ULP Example: using ADC in deep sleep

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This file contains assembly code which runs on the ULP.

   ULP wakes up to run this code at a certain period, determined by the values
   in SENS_ULP_CP_SLEEP_CYCx_REG registers. On each wake up, the program
   measures input voltage on the given ADC channel 'adc_oversampling_factor'
   times. Measurements are accumulated and average value is calculated.
   Average value is compared to the two thresholds: 'low_thr' and 'high_thr'.
   If the value is less than 'low_thr' or more than 'high_thr', ULP wakes up
   the chip from deep sleep.
*/

/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files 
 */
#include "soc/rtc_cntl_reg.h"
#include "soc/soc_ulp.h"

/* ADC1 channel 6, GPIO34 */
   .set adc_channel_0, 0
   .set adc_channel_1, 1
   .set adc_channel_2, 2

   /* Configure the number of ADC samples to average on each measurement.
      For convenience, make it a power of 2. */
   .set adc_oversampling_factor_log, 3
   .set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

   .set max_samples, 512
   .set max_samples_bytes, max_samples * 4

   /* Define variables, which go into .bss section (zero-initialized data) */
   .bss

    /* Counter of measurements done */
   .global sample_counter
sample_counter:
   .long 0

   .global tmp_ch_0
tmp_ch_0:
    .long 0

   .global tmp_ch_1
tmp_ch_1:
    .long 0

   .global tmp_ch_2
tmp_ch_2:
    .long 0

	.global channel_0_measurements
channel_0_measurements:
   .fill max_samples_bytes

   .global channel_1_measurements
channel_1_measurements:
   .fill max_samples_bytes

   .global channel_2_measurements
channel_2_measurements:
   .fill max_samples_bytes

   /* Code goes into .text section */
   .text
   .global entry
entry:
   // initialize the loop counter, reset r1 and r2
   move r0, 0
   move r1, 0
   move r2, 0

   stage_rst
measure:
   // store temporary measurements in r3
   // measure first channel
   adc r3, 0, adc_channel_0 + 1
   add r0, r0, r3

   adc r3, 0, adc_channel_1 + 1
   add r1, r1, r3

   adc r3, 0, adc_channel_2 + 1
   add r2, r2, r3

   // repeat if we haven't measure enough samples for oversampling
   stage_inc 1

   jumps measure, adc_oversampling_factor, lt

   // r3 can be used again

   // divide by oversampling factor. Since its a power of two, we can >> ^^
   rsh r0, r0, adc_oversampling_factor_log
   rsh r1, r1, adc_oversampling_factor_log
   rsh r2, r2, adc_oversampling_factor_log

   //store adc0 in temp
   move r3, tmp_ch_0
   st r0, r3, 0

   //store adc1 in temp
   move r3, tmp_ch_1
   st r1, r3, 0

   //store adc2 in temp
   move r3, tmp_ch_2
   st r2, r3, 0

   /*
   r0 - value of samle_counter
   r1 - addr (channel_x_measurement + sample_counter) to store adc measurement in
   r2 - value of adc measurement (tmp_ch_X)
   r3 - temp
   */

   //store adc0
   move r3, sample_counter //r3 = addr of sample_counter
   ld r0, r3, 0 // r0 = value of sample_counter

   // store measurements in respective arrays
   move r3, channel_0_measurements // load its address
   add r1, r3, r0                  //  r1 = addr ch_0_data+offset

   move r3, tmp_ch_0 // r3 = addr tmp_ch_0
   ld r2, r3, 0     // r2 = value of adc0 measurment
   st r2, r1, 0     // and store the value


   //store adc1
   move r3, channel_1_measurements // load its address
   add r1, r3, r0                  //  r1 = addr ch_0_data+offset

   move r3, tmp_ch_1 // r3 = addr tmp_ch_0
   ld r2, r3, 0     // r2 = value of adc1 measurment
   st r2, r1, 0     // and store the value

   //store adc2
   move r3, channel_2_measurements // load its address
   add r1, r3, r0                  //  r1 = addr ch_0_data+offset

   move r3, tmp_ch_2 // r3 = addr tmp_ch_0
   ld r2, r3, 0     // r2 = value of adc2 measurment
   st r2, r1, 0     // and store the value


   // increment our sample counter and store the value
   move r3, sample_counter
   add r0, r0, 1
   st r0, r3, 0

   jumpr wake_up, max_samples, ge  // wake up when we stored enough values (compares max_samples with r0)

   halt

wake_up:
   //READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
   //AND r0, r0, 1
   //JUMP wake_up, eq    // Retry until the bit is set
   WAKE                          // Trigger wake up
   REG_WR 0x006, 24, 24, 0       // Stop ULP timer (clear RTC_CNTL_ULP_CP_SLP_TIMER_EN)
   HALT
