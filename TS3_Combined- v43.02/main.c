/*
 * ======== Standard MSP430 includes ========
 */
#include <msp430.h>


/*
* ======== Grace related includes ========
*/
#include <stdint.h>
#include <ti/mcu/msp430/Grace.h>
#include <driverlib/MSP430FR5xx_6xx\timer_a.h>
#include <driverlib/MSP430FR5xx_6xx/dma.h>
#include <driverlib/MSP430FR5xx_6xx\eusci_a_uart.h>
#include <driverlib/MSP430FR5xx_6xx/inc/hw_memmap.h>
#include <driverlib/MSP430FR5xx_6xx\timer_a.h>
#include <driverlib/MSP430FR5xx_6xx\timer_b.h>


#define MASTER 0
#define SLAVE  1
#define THIS_UNIT MASTER

/*
       /////////////////////////////////////////////////////////////////////////////////////////////
       //
       // TURTLE SENSE PHASE THREE DEMO v.0.43.02
       // Integrated software that will run Communications Master Controller PROTOTYPE
       //                               and Smart Sensor Module PROTOTYPE
       //
       // Phase Two and Three Programmed By Samuel Wantman cc 4.0
       // Phase One Programmed by Thomas Zimmerman  cc 4.0
       //
       //      VERSION HISTORY OF PHASE TWO
       //      V0.01 2/21/14 First operational version for use with Phase II board version 0.14
       //            3/11/14 Comm head master controller code split from smart sensor code
       //            5/2/14 Revised to work with Phase II board version 0.25
       //            7/12/14 Timeout added if sensor does not report back after a day and 256 seconds.
       //    v0.2504 8/6/14  Battery check on start-up to keep batteries from running down if there are hardware problems
       //
       //      VERSION HISTORY OF PHASE THREE
       //      V0.37.01 4/28/15 Adapted code to work with v0.37 prototype board for Phase III using MSP430FR5949 processor
       //      V0.40.01 7/25/15 first attempt at getting new board to work had problems with Grace.  Abandoned
       //      V0.41.02 8/4/15 UART send and receive written.  Send works, receive yet to be tested
       //      V0.41.03 8/9/15 Combining sensor and comm programing into one program.
       //                      Comm will have working code for the sensor
       //                      and hopefully be able to restore and upgrade code in the sensors.
       //                      Changed power settings from 3.3 volts to 3.0 volts
       //      v0.41.04 8/22/15 Got prototype working with phone board and sending messages to sensor
       //      v0.41.05 9/14/15 Got two way communications working with phone board and sensor
       //      DEMOv0.43.01    2/07/16 Demo version for SERSTM  -- just blinks LEDs
       //      v0.43.01 3/16/16  Beginning of conversion to MSP430FR5949IRHA chip (40 pin)- sensor protocol and registration working
       //      v0.42.02         Getting the rest of Phase II code to work
        *
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////// DEFINITIONS //////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////// PROGRAM PARAMETERS (may vary for each unit)

       // REMEMBER TO CHECK THE FTP LOG-IN CREDENTIALS IN THE FOLLOWING TWO LINES!!!!
#define ISP_PROVIDER "AT+CGDCONT=1,\"IP\",\"ISP-Provider.net\"\r" // AT message to open internet connection UPDATE WITH YOUR CREDENTIALS
#define FTP_ACCOUNT   "AT#FTPOPEN=\"yourwebsite.org\",\"ftp-username\",\"ftp-pasword\"\r" // AT messages to create FTP connection UPDATE WITH YOUR CREDENTIALS


#define USER_PASSWORD "ABCDEFGH"  // 8 character password needed for program or parameter downloads.  Recompile with a different password.


#define TURTLE_SENSE_BANK 1


// The following is only necessary for compiling a SMART SENSOR slave
       // SMART SENSOR IDENTIFICATION
#define NEST_ID "123456.000,060914,M-AA0001"
       // The last 6 digits is a unique serial number for each smart sensor.
       // Update the last 6 digits before compiling for each new device.
       // The rest of the data will be updated during each new registration with a master
       // What you see here is just an example of what it could look like
#define SERIAL_NUMBER "M-AA0001"   // Unique serial number for each device.  Update before compiling for a new device
#define SERIAL_ID_LEN   8          // length of serial number string (don't change without changing all the places this is used)
       // PHASE THREE SERIAL NUMBER TEMPLATES
       // M-AA####  -- Master device
       // S-AA####  -- Slave device

       // PHASE TWO SERIAL NUMBER TEMPLATES (Now obsolete)
       // C-AA####  -- Comm unit device
       // AA####    -- Sensor device

       // SMART SENSOR TEMPERATURE CALIBRATION
#define TEMPERATURE_OFFSET 0x0000       // Default setting of 0 does not change the reading
#define TEMPERATURE_RATIO 0x1000        // The temperature reading adjusted by the offset times this number will be divided by 0x1000,
       // These values need to be manually entered for each smart sensor.
       // Eventually there will be an automated procedure for setting them.



/*
============================== HARDWARE SETTINGS====================================================

       // All unused pins should be set to Output Low to minimize current drain
        *
        *          NEW SETTINGS for PHASE THREE PROTOTYPE
        *          (M) = Master only    (S) = Slave only

  PIN HARDWARE USAGE           w/o device              w/phone(M) or sensor(S)
  MSP430FR5949IRHA             -------------------     -------------------
  PIN CONNCECTION      PORT    FUNC    HIGH    LOW     FUNC    HIGH    LOW     USAGE
   1(M)    AUX12.3     1.0     Opt     Opt     Opt     Opt     Opt     Opt     J12 connector pin 0 AUX I/O
   1(S)    Unused Pin  1.0
   2(M)    AUX12.2     1.1     Opt     Opt     Opt     Opt     Opt     Opt     J12 connector pin 1 AUX I/O
   2(S)    MOISTURE    1.1     TA1CLK INPUT                                    TA1CLK input clock counter for Moisture Sensor
   3(M)    AUX12.1     1.2     Opt     Opt     Opt     Opt     Opt     Opt     J12 connector pin 2 AUX I/O
   3(S)    COMP_OUT    1.2     COMPARTOR OUTPUT                                Comparator output for Moisture Sensor
   4(M)    BattMon     3.0     ADC     n/a     n/a     ADC     n/a     n/a     Analogue input for reading battery voltage
   4(S)    COMP-IN     3.0     COMPARATOR NEGATIVE INPUT                      Comparator minus input for moisture sensor
   5(M)    PhnReset    3.1     OUT-LO  ---     ---     OUT     Hi-Z    LOW     Phone Reset (hold low to reset)
   5(S)    COMP+IN     3.1     COMPARATOR POSITIVE INPUT                       Comparator positive input for moisture sensor
   6(M)    AUX14.1     3.2     Opt     Opt     Opt     Opt     Opt     Opt     J14 connector pin 1 AUX I/O
   6(S)    MS_ENABLE   3.2     OUT     ON      OFF     OUT     ON      OFF     Enable the Moisture Sensor
   7(M)    AUX14.2     3.3     Opt     Opt     Opt     Opt     Opt     Opt     J14 connector pin 2 AUX I/O
   7(S)    Unused Pin  3.3
   8(M)    AUX13.3     1.3     Opt     Opt     Opt     Opt     Opt     Opt     J13 connector pin 3 UCB0STE (Chip Select)
   8(S)    AUX3.5      1.3     Opt     Opt     Opt     Opt     Opt     Opt     J3 connector pin 5 UCB0STE (Chip Select)
   9(M)    AUX14.3     1.4     Opt     Opt     Opt     Opt     Opt     Opt     J14 connector pin 3 AUX I/O
   9(S)    Unused Pin  1.4
   10(M)   AUX14.4     1.5     Opt     Opt     Opt     Opt     Opt     Opt     J14 connector pin 4 AUX I/O
   10(S)   Unused Pin  1.5
   11      TDO         J.0     Opt     Opt     Opt     Opt     Opt     Opt     JTAG connector
   12      TDI         J.1     Opt     Opt     Opt     Opt     Opt     Opt     JTAG connector
   13      TMS         J.2     Opt     Opt     Opt     Opt     Opt     Opt     JTAG connector
   14      TCK         J.3     Opt     Opt     Opt     Opt     Opt     Opt     JTAG connector
   15(M)   LED1        4.0     OUT     LED ON  LED OFF OUT     LED ON  LED OFF Phone power indicator green LED  (see individual board specs for meaning of blinks)
   15(S)   INT2        4.0     n/a     n/a     n/a     IN                      Interrupt 2 from ADXL
   16(M)   LED2        4.1     OUT     LED ON  LED OFF OUT     LED ON  LED OFF Sensor communications indicator red LED (quick blink at the start of every discussion)
   16(S)   INT1        4.1     n/a     n/a     n/a     IN                      Interrupt 1 from ADXL
   17(M)   U-OUT       2.5     OUT-LO  ---     Drain   UCA1TXD DATA    DATA    UART to Phone, UCA1TXD
   17(S)   SIMO        2.5     SPI     ---     ---     SPI     Data    Data    SPI MOSI connection from ADXL (UCA1SIMO)
   18(M)   U-IN        2.6     IN      ---     Drain   UCA1RXD DATA    DATA    UART from Phone, UCA1RXD
   18(S)   SOMI        2.6     SPI     ---     ---     SPI     Data    Data    SPI MISO connection from ADXL (UCA1SOMI)
   19      TEST        TEST    ---     ---     ---     ---     ---     ---     JTAG connector
   20      #RST        #RST    ---     ---     ---     ---     ---     ---     JTAG connector, Reset Button and Switch
   21      SensorTXD   2.0     UCA0TXD DATA    DATA    UCA0TXD DATA    DATA    Smart Sensor UART UCA0
   22      SensorRXD   2.1     UCA0RXD DATA    DATA    UCA0RXD DATA    DATA    Smart Sensor UART UCA0
   23(M)   AUX13.4     2.2     Opt     Opt     Opt     Opt     Opt     Opt     J13 connector pin 4 UCB0CLK
   23(S)   AUX3.6      2.2     Opt     Opt     Opt     Opt     Opt     Opt     J3 connector pin 6 UCB0CLK
   24      SMCLK       3.4     OUT     1MHz    1MHz    OUT     1MHz    1MHz    1MHz SMCLK output -- Carrier for Sensor UART
   25(M)   SnsrEn      3.5     OUT     Power   Reset   OUT     Power   Reset   Power to smart sensor (low = off = reset)
   25(S)   AUX3.2      3.5     Opt     Opt     Opt     Opt     Opt     Opt     J3 connector pin 2
   26(M)   Aux En      3.6     OUT     ON      OFF     OUT     ON      OFF     Power to Aux devices
   26(S)   AUX3.1      3.6     Opt     Opt     Opt     Opt     Opt     Opt     J3 connector pin 1
   27(M)   PhnPower    3.7     OUT-LOW ON      ---     OUT-HI  ---     OFF     Turn on 5V Regulator to power phone card
   27(S)   AUX3.3      3.7     Opt     Opt     Opt     Opt     Opt     Opt     J3 connector pin 3
   28(M)   J13.2       1.6     Opt     Opt     Opt     Opt     Opt     Opt     J13 connector pin 2 UCB0SIMO
   28(S)   AUX3.7      1.6     Opt     Opt     Opt     Opt     Opt     Opt     J3 connector pin 7 UCB0SIMO
   29(M)   J13.1       1.7     Opt     Opt     Opt     Opt     Opt     Opt     J13 connector pin 2 UCB0SOMI
   29(S)   AUX3.8      1.7     Opt     Opt     Opt     Opt     Opt     Opt     J3 connector pin 8 UCB0SOMI
   30(M)   CheckBat    4.4     OUT-LOw ---     ---     ---     ---     ---     Switch for battery check (not in prototype)
   30(S)   AUX3.4      4.4     Opt     Opt     Opt     Opt     Opt     Opt     J3 connector pin 4
   31      DVSS        DVSS    ---     ---     ---     ---     ---     ---     Digital Ground
   32      DVCC        DVCC    ---     ---     ---     ---     ---     ---     Digital Power
   33(M)   PhnMon      2.7     IN      ---     ---     IN      Status blinks   Phone status indicator (see individual Janus board specs for meaning of blinks)
   33(S)   ADXL_En     2.7     OUT     ON      ---     OUT     ---     OFF     Control to power ADXL chip
   34(M)   PwrMon      2.3     IN      ---     OFF     IN      ON      ---     Phone power indicator
   34(S)   /CS         2.3     OUT-LOW ---     ---     OUT     Ignore  Select  Chip select for ADXL chip
   35(M)   PhnOn/Off   2.4     OUT-HI  ---     ---     OUT     Hi-Z    LOW     Phone on/off request (hold low for 3 sec)
   35(S)   CLK         2.4     SPI     ---     ---     SPI     CLOCK   CLOCK   SPI Clock connection from ADXL (UCA1CLK)
   36      AVSS        AVSS    ---     ---     ---     ---     ---     ---     Analog ground
   37      LFXIN       J.4     ---     ---     ---     ---     ---     ---     Crystal
   38      LFXOUT      J.5     ---     ---     ---     ---     ---     ---     Crystal
   39      AVSS        AVSS    ---     ---     ---     ---     ---     ---     Analog ground
   40      AVCC        AVCC    ---     ---     ---     ---     ---     ---     Analog 3.0 V


*/

// MSP430FR5949IRHA Hardware  -- Port definitions  ///////////////////////////////////////////////////////////////

// MASTER definitions  (some are used in the SLAVEs as well)
       // Janus Phone board power settings (MASTER)
#define PHONE_POWER_ENABLE      P3OUT |= BIT7           // Turn on the 5 V power supply
#define PHONE_POWER_DISABLE     P3OUT &= ~ BIT7         // Turn off the 5 V power supply
#define PHONE_ON_OFF_REQUEST    P2OUT &= ~ BIT4         // Turn the phone on or off (must be held low for a while)
#define PHONE_ON_OFF_HIZ        P2OUT |= BIT4           // When not turning the phone on or off it stays hi
#define PHONE_RESET_REQUEST     P3OUT &= ~ BIT1         // Reset the phone by holding this bit low
#define PHONE_RESET_HIZ         P3OUT |= BIT1           // When not resetting, it stays hi
#define PHONE_IS_ON             (P2IN & BIT3)           // The phone is on
#define PHONE_IS_OFF            (!(P2IN & BIT3))        // The phone is off
#define PHNMON_STATUS           (P2IN & BIT7)           // Test indicator of phone board status (the meaning of blinking is different for different Janus boards)
#define PHNMON_LED_OFF          P4OUT &= ~BIT1          // Turn off phone board status LED
#define PHNMON_LED_ON           P4OUT |= BIT1           // Turn on phone board status LED

       // Sensor Power Supply (MASTER)
#define COAX_POWER_ENABLE       P3OUT |= BIT5           // Start up the sensors
#define COAX_POWER_DISABLE      P3OUT &= ~BIT5          // Turn off the sensors

       // Auxiliary Power Supply (MASTER)
#define AUX_POWER_ENABLE        P3OUT |= BIT6           // Turn on the aux power
#define AUX_POWER_DISABLE       P3OUT &= ~BIT6          // Turn off the aux power

       // UART Settings for connection to Janus Phone board (MASTER)
       // The Phone board uses UCA1
#define PHONE_BUSY              (UCA1STATW & UCBUSY)    // The eUSCI module is transmitting or receiving
#define PHONE_READY             (!(UCA1STATW & UCBUSY)) // The eUSCI module is not transmitting or receiving
#define PHONE_RX_BUFF           UCA1RXBUF               // The receive buffer for the Master UART
#define PHONE_TX_BUFF           UCA1TXBUF               // The transmit buffer for the Master UART
#define PHONE_TX_READY          (UCA1IFG & UCTXIFG)     // Checks the TX buffer interrupt flag is set
#define PHONE_TX_NOT_READY      (!(UCA1IFG & UCTXIFG))  // checks if anything is in the TX buffer
#define PHONE_TX_DONE           (UCA1IFG & UCTXCPTIFG)  // looks to see if the byte has been sent
#define PHONE_RX_ENABLE         UCA1IE |= UCRXIE        // enable receive buffer interrupts
#define PHONE_RX_DISABLE        UCA1IE &= ~UCRXIE       // disable receive buffer interrupts
#define PHONE_RX_BUFF_EMPTY     (!(UCA1IFG & UCRXIFG))  // the received interrupt flag is not set
#define PHONE_TX_ENABLE         UCA1IE |= UCTXIE        // enable transmit buffer empty interrupts
#define PHONE_TX_DISABLE        UCA1IE &= ~UCTXIE       // disable transmit buffer empty interrupts
#define PHONE_TX_DONE_ENABLE    UCA1IE |= UCTXCPTIE     // enable transmit complete interrupts
#define PHONE_TX_DONE_DISABLE   UCA1IE &= ~UCTXCPTIE    // disable transmit complete interrupts
#define PHONE_UART_RESET        UCA1CTL1 |= UCSWRST     // Put Master's eUSCI in reset
#define PHONE_UART_ENABLE       UCA1CTL1 &= ~UCSWRST    // release Master's eUSCI from reset
#define PHONE_INTERRUPT_DISABLE UCA1IE = 0              // disable all UART interrupts
#define PHONE_USES_SMCLK        UCA1CTL1 |= UCSSEL__SMCLK //Set the UART timing based on the SMCLK



       // UART settings for Coax connection for smart sensors (MASTER and SLAVE)
       // The sensor communications uses UCA0

#define COAX_TX_BUFF            UCA0TXBUF               // The transmit buffer for the Master UART
#define COAX_TX_READY           (UCA0IFG & UCTXIFG)     // Checks the TX buffer interrupt flag is set
#define COAX_TX_NOT_READY       (!(UCA0IFG & UCTXIFG))  // checks if anything is in the TX buffer
#define COAX_TX_DONE            (UCA0IFG & UCTXCPTIFG)   // looks to see if the byte has been sent
#define COAX_TX_NOT_DONE        (!(UCA0IFG & UCTXCPTIFG))
#define COAX_RX_START_ENABLE    UCA0IE |= UCSTTIE       // enable start bit received interrupts
#define COAX_RX_START_DISABLE   UCA0IE &= ~UCSTTIE      // disable start bit received interrupts
#define COAX_TX_ENABLE          UCA0IE |= UCTXIE        // enable transmit buffer empty interrupts
#define COAX_TX_DISABLE         UCA0IE &= ~UCTXIE       // disable transmit buffer empty interrupts
#define COAX_TX_DONE_ENABLE     UCA0IE |= UCTXCPTIE     // enable transmit complete interrupts
#define COAX_TX_DONE_DISABLE    UCA0IE &= ~UCTXCPTIE    // disable transmit complete interrupts
#define COAX_RX_BUFF            UCA0RXBUF               // The receive buffer for the Master UART
#define COAX_RX_BUFF_EMPTY      (!(UCA0IFG & UCRXIFG))  // monitor UART rx buffer
#define COAX_RX_ENABLE          UCA0IE |= UCRXIE        // enable receive buffer interrupts
#define COAX_RX_DISABLE         UCA0IE &= ~UCRXIE       // disable receive buffer interrupts
#define COAX_BUSY               (UCA0STATW & UCBUSY)    // The eUSCI module is transmitting or receiving
#define COAX_READY              (!(UCA0STATW & UCBUSY)) // The eUSCI module is not transmitting or receiving
#define COAX_UART_RESET         UCA0CTL1 |= UCSWRST     // Put Master's eUSCI in reset
#define COAX_UART_ENABLE        UCA0CTL1 &= ~UCSWRST    // release Master's eUSCI from reset
#define COAX_INTERRUPT_DISABLE  UCA0IE = 0              // disable all UART interrupts
#define COAX_USES_SMCLK         UCA0CTL1 |= UCSSEL__SMCLK //Set the UART timing based on the SMCLK
#define COAX_DMA_ENABLE         DMA_enableInterrupt     // Set the DMA receive source to the UART buffer
#define COAX_DMA_DISABLE        DMA_disableInterrupt    // Set the DMA receive destination to the beginning of the data buffer
#define COAX_DMA_RX_DESTINATION0 DMA_setDstAddress(DMA_CHANNEL_0, (uint32_t) incoming.uart, DMA_DIRECTION_UNCHANGED)
#define COAX_DMA_RX_SOURCE0      DMA_setSrcAddress(DMA_CHANNEL_0, (uint32_t) &UCA0RXBUF, DMA_DIRECTION_UNCHANGED)
#define COAX_DMA_RX_DESTINATION1 DMA_setDstAddress(DMA_CHANNEL_1, (uint32_t) incoming.uart+1, DMA_DIRECTION_INCREMENT)
#define COAX_DMA_RX_SOURCE1      DMA_setSrcAddress(DMA_CHANNEL_1, (uint32_t) &UCA0RXBUF, DMA_DIRECTION_UNCHANGED)
#define DMA_SHORT_SIZE          0x0B                    // The length of a short coax message

               //Other hardware ports used (MASTER)
#define LED_ON          P4OUT |= BIT0               // Test LED (can be disabled in runtime version)
#define LED_OFF         P4OUT &= ~BIT0
#define RED_LED_ON      P4OUT |= BIT1               // Power indicator
#define RED_LED_OFF     P4OUT &= ~BIT1
#define GREEN_LED_ON    P4OUT |= BIT0               // Communications status
#define GREEN_LED_OFF   P4OUT &= ~BIT0


               // Clock settings
#define CLOCK_UNLOCK    CSCTL0 = CSKEY
#define CLOCK_LOCK      CSCTL0_H = 0                    // Lock CS registers
#define SMCLK_OFF       P3SEL1 &= ~BIT4                 // turn off the SMCLK output on P3.4
#define SMCLK_ON        P3SEL1 |= BIT4                  // turn on the SMCLK output on P3.4

#define CLOCKSPEED 0x08                        // Set the clock speed (4 MHz)
#define ONE_MHz         0x0000
#define FOUR_MHz        0x0006
#define EIGHT_MHz       0x000C

#define ENTER_LOW_POWER_MODE_1  __bis_SR_register(LPM1_bits + GIE)
#define ENTER_LOW_POWER_MODE_3  __bis_SR_register(LPM3_bits + GIE)
#define END_LOW_POWER_MODE      __bic_SR_register_on_exit(LPM0_bits)

               // Timer controls
#define START_MSEC_TIMER            TA2CTL |= 0x0010    // turn on bit 4  to start the timer in up mode
#define STOP_MSEC_TIMER             TA2CTL &= 0xFFCF    // turn off bits 4 and 5 to stop the timer
#define START_RANDOM_TIMER          TA0CTL |= 0x0010    // turn on bit 4  to start the timer in up mode
#define STOP_RANDOM_TIMER           TA0CTL &= 0xFFCF    // turn off bits 4 and 5 to stop the timer

#define RX_TIMEOUT                  10      // 10 milliseconds
#define SYNCH_TIMEOUT               96      // 96 milliseconds - if it misses the first message, it should catch the second.

//#define INTERVAL_INTERRUPT_ENABLE   TB0CTL |= TBIE
//#define INTERVAL_INTERRUPT_DISABLE  TB0CTL &= ~TBIE
#define NORMAL_INTERVAL             TB0CCR0 = 0x03FF  // divide by 8 for 1028ths of a sec (in this case it equals 1/8 sec)
#define SHORT_INTERVAL              TB0CCR0 = 0x03DF   // this is roughly 4 msec less than 1/8 sec
#define START_INTERVAL_TIMER        TIMER_B_startCounter(TIMER_B0_BASE, TIMER_B_UP_MODE)
#define STOP_INTERVAL_TIMER         TIMER_B_stop(TIMER_B0_BASE)
#define ZERO_TIMER                  TB0R = 0

                // Battery definitions
#define SHUT_DOWN_LEVEL     3   //Turn off the units at 3% of battery capacity.  The batteries should never get to zero.
#define START_UP_LEVEL      5   //This level should be more than enough to make two phone calls,
                                // a call to say the unit is running, and a later call when the batteries drop
#define CHECK_BATTERY_ON  P4OUT |= BIT4     // Turns on the battery test circuit (draws a small current)
#define CHECK_BATTERY_OFF P4OUT &= ~BIT4    // Turns off the battery test circuit to save power

// SLAVE definitions  (these settings are unique to the SLAVEs)
       // ADXL Accelerometer power (SLAVE)
#define ADXL_POWER_ON     P2OUT |= BIT7               // Pin 2.7 powers the sensor
#define ADXL_POWER_OFF    P2OUT &= ~BIT7              // All ADXL pins must be zeroed to reset the sensor

       // ADXL UART Settings for connection to Janus Phone board (MASTER)
       // The ADXL sensor uses UCA1 in SPI mode
//#define ADXL_UART_ON           EUSCI_A_UART_enable(EUSCI_A1_BASE)
//#define ADXL_UART_OFF          EUSCI_A_UART_disable(EUSCI_A1_BASE)
#define ADXL_UART_RESET        UCA1CTL1 |= UCSWRST     // Put Master's eUSCI in reset
#define ADXL_UART_ENABLE       UCA1CTL1 &= ~UCSWRST    // release Master's eUSCI from reset
#define ADXL_BUSY              (UCA1STATW & UCBUSY)    // The eUSCI module is transmitting or receiving
#define ADXL_READY             (!(UCA1STATW & UCBUSY)) // The eUSCI module is not transmitting or receiving
#define ADXL_RX_BUFF           UCA1RXBUF               // The receive buffer for the Master UART
#define ADXL_TX_BUFF           UCA1TXBUF               // The transmit buffer for the Master UART
#define ADXL_TX_READY          (UCA1IFG & UCTXIFG)     // Checks the TX buffer interrupt flag is set
#define ADXL_TX_NOT_READY      (!(UCA1IFG & UCTXIFG))  // checks if anything is in the TX buffer
#define ADXL_TX_DONE           (UCA1IFG & UCTXCPTIFG)  // looks to see if the byte has been sent
#define ADXL_RX_ENABLE         UCA1IE |= UCRXIE        // enable receive buffer interrupts
#define ADXL_RX_DISABLE        UCA1IE &= ~UCRXIE       // disable receive buffer interrupts
#define ADXL_RX_BUFF_EMPTY     (!(UCA1RXBUF))
#define ADXL_RX_NOT_READY      (!(UCA1IFG & UCRXIFG))  // But location to monitor while receiving data is in process
#define ADXL_TX_ENABLE         UCA1IE |= UCTXIE        // enable transmit buffer empty interrupts
#define ADXL_TX_DISABLE        UCA1IE &= ~UCTXIE       // disable transmit buffer empty interrupts
#define ADXL_TX_DONE_ENABLE    UCA1IE |= UCTXCPTIE     // enable transmit complete interrupts
#define ADXL_TX_DONE_DISABLE   UCA1IE &= ~UCTXCPTIE    // disable transmit complete interrupts
#define ADXL_INTERRUPT_DISABLE UCA1IE = 0              // disable all UART interrupts
#define ADXL_USES_SMCLK        UCA1CTL1 |= UCSSEL__SMCLK //Set the UART timing based on the SMCLK
#define ADXL_SELECT            P2OUT &= ~BIT3          // Select the ADXL by setting STE pin low (P2.3)
#define ADXL_DESELECT          P2OUT |= BIT3           // Deselect the ADXL by setting STE pin high (P2.3)

#define YES 1
#define NO 0
#define TRUE 1
#define FALSE 0




/////////////////////////////////////////////////////////////////////////////////
//////////////////// ADXL362 Sensor definitions /////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

        // ADXL362 Instruction set

    #define ADXL_WRITE 0x0A         // Instruction to write to a register
    #define ADXL_READ  0x0B         // Instruction to read from a register
    #define ADXL_FIFO  0x0D         // Instruction to read the FIFO stack

        // ADXL362 Addresses
    #define XDATA_LO 0x0E           // First data byte address -- Low byte of XData (can be read as 8 bytes: XLO, XHI, YLO, YHI, ZLO, ZHI, TEMPLO, TEMPHI)

    // Other defines used with ADXL362
    #define XYZ_PINS    0xC000      // 00 = x reading, 01 = y reading, 10 = z reading, 11 = temp reading
    #define X_READING   0x0000
    #define Y_READING   0x4000
    #define Z_READING   0x8000
    #define T_READING   0xC000
    #define NEG_MASK    0x3000      // These bits always indicate a negative reading.  Both should be set or unset
    #define ADXL_MASK   0x3FFF


/*
 * ****************** SENSOR HARDWARE SETTINGS **********************************************

//  //  //  //  //  ADXL362 Registers   //  //  //  //  //  //  //

    Table 11. Register Summary

    Reg     Name            Bits     Bit 7   Bit 6    Bit 5      Bit 4     Bit 3           Bit 2            Bit 1      Bit 0    Reset   RW
    0x00    DEVID_AD        [7:0]                               DEVID_AD[7:0]                                                   0xAD    R
    0x01    DEVID_MST       [7:0]                               DEVID_MST[7:0]                                                  0x1D    R
    0x02    PARTID          [7:0]                               PARTID[7:0]                                                     0xF2    R
    0x03    REVID           [7:0]                               REVID[7:0]                                                      0x01    R
    0x08    XDATA           [7:0]                               XDATA[7:0]                                                      0x00    R
    0x09    YDATA           [7:0]                               YDATA[7:0]                                                      0x00    R
    0x0A    ZDATA           [7:0]                               ZDATA[7:0]                                                      0x00    R
    0x0B    STATUS          [7:0]  ERR_USER_ REGS|AWAKE  |INACT|ACT     |FIFO_OVER-RUN  |FIFO_WATER-MARK|FIFO_READY |DATA_READY 0x40    R
    0x0C    FIFO_ENTRIES_L  [7:0]                                               FIFO_ENTRIES_L[7:0]                             0x00    R
    0x0D    FIFO_ENTRIES_H  [7:0]                                   UNUSED                                 |FIFO_ENTRIES_H[1:0] 0x00    R
    0x0E    XDATA_L         [7:0]                               XDATA_L[7:0]                                                    0x00    R
    0x0F    XDATA_H         [7:0]                           SX          |                       XDATA_H[3:0]                    0x00    R
    0x10    YDATA_L         [7:0]                               YDATA_L[7:0]                                                    0x00    R
    0x11    YDATA_H         [7:0]                           SX          |                       YDATA_H[3:0]                    0x00    R
    0x12    ZDATA_L         [7:0]                               ZDATA_L[7:0]                                                    0x00    R
    0x13    ZDATA_H         [7:0]                           SX          |                       ZDATA_H[3:0]                    0x00    R
    0x14    TEMP_L          [7:0]                               TEMP_L[7:0]                                                     0x00    R
    0x15    TEMP_H          [7:0]                           SX          |                       TEMP_H[3:0]                     0x00    R
    0x20    THRESH_ACT_L    [7:0]                               THRESH_ACT_L[7:0]                                               0x00    RW
    0x21    THRESH_ACT_H    [7:0]               UNUSED                                  |            THRESH_ACT_H[2:0]          0x00    RW
    0x22    TIME_ACT        [7:0]                               TIME_ACT[7:0]                                                   0x00    RW
    0x23    THRESH_INACT_L  [7:0]                               THRESH_INACT_L[7:0]                                             0x00    RW
    0x24    THRESH_INACT_H  [7:0]               UNUSED                                  |           THRESH_INACT_H[2:0]         0x00    RW
    0x25    TIME_INACT_L    [7:0]                               TIME_INACT_L[7:0]                                               0x00    RW
    0x26    TIME_INACT_H    [7:0]                               TIME_INACT_H[7:0]                                               0x00    RW
    0x27    ACT_INACT_CTL   [7:0]   RES                   | LINKLOOP    |  INACT_REF    |INACT_EN       |ACT_REF    |ACT_EN     0x00    RW
    0x28    FIFO_CONTROL    [7:0]               UNUSED                  |   AH          |FIFO_TEMP      |       FIFO_MODE       0x00    RW
    0x29    FIFO_SAMPLES    [7:0]                               FIFO_SAMPLES[7:0]                                               0x80    RW
    0x2A    INTMAP1         [7:0]   INT_LOW       |AWAKE  |INACT|ACT    |FIFO_OVER-RUN  |FIFO_WATER-MARK|FIFO_READY |DATA_READY 0x00    RW
    0x2B    INTMAP2         [7:0]   INT_LOW       |AWAKE  |INACT|ACT    |FIFO_OVER-RUN  |FIFO_WATER-MARK|FIFO_READY |DATA_READY 0x00    RW
    0x2C    FILTER_CTL      [7:0]        RANGE            |RES  |HALF_BW|   EXT_SAMPLE  |                 ODR                   0x13    RW
    0x2D    POWER_CTL       [7:0]   RES           |EXT_CLK|  LOW_NOISE  |       WAKEUP  |   AUTOSLEEP   |   MEASURE             0x00    RW

*/
////////////// ADXL362 Instruction set for MANUAL OPERATION (command 0x0B0) //////////////////
// The first byte sent after the command should be the instruction code for the ADXL (un-comment if using)
#define ADXL_WRITE 0x0A           // Instruction to write to a register
#define ADXL_READ  0x0B           // Instruction to read from a register
// DO NOT SEND THE following instruction, it will return an error.  To stream directly use command (0x0C)
//#define ADXL_FIFO  0x0D           // Instruction to read the FIFO stack
//

/////////////////////////////////////////////////////////////////////////////
//---- ADXL Defines  // Refer to ADXL362 documentation for complete descriptions:  http://dlnmh9ip6v2uc.cloudfront.net/datasheets/BreakoutBoards/ADXL362.pdf
// ADXL bit definitions

   //   FIFO_CONTROL BITS
#define FIFO_AH             BIT3    // This bit is the MSB of the FIFO_SAMPLES register, allowing FIFO samples a range of 0 to 511 bytes.
#define FIFO_TEMP_ON        BIT2    // Store Temperature Data to FIFO. 1 = temperature data is stored in the FIFO together with x-, y-, and z-axis acceleration data.
#define FIFO_OFF            0x00    // BIT0 and BIT1 combinations to Enable FIFO and Mode Selection.
#define FIFO_OLDEST_SAVED   BIT0    // Only the oldest samples are saved.  Once the FIFO fills, nothing else is stored until the data is read, then it empties
#define FIFO_STREAM         BIT1    // The FIFO is filled and then overwritten with new readings
#define FIFO_TRIGGERED      (BIT0|BIT1) // b01 = Oldest saved mode.  b10 = Stream mode. b11=Triggered mode.

    //  FILTER CONTROL BITS (select one range, one BW, one ODR)
#define RANGE_2G        0x00    // Default sensitivity setting of +/- 2g
#define RANGE_4G        BIT6    // +/- 4G
#define RANGE_8G        BIT7    // +/- 8G   (use only one range)
#define HALF_BW         0x00    // Default setting of half bandwidth antialiasing filtering
#define QUARTER_BW      BIT4    // More conservative antialiasing of quarter bandwidth
#define EXT_SAMPLE      BIT3    // External Sampling Trigger. 1 = the INT2 pin is used for external conversion timing control.

    //  POWER CONTROL BITS
#define EXT_CLK         BIT6    //External Clock. 1 = the accelerometer runs off the external clock provided on the INT1 pin.
                                // EXTERNAL CLOCK IS NOT SUPPORTED IN THE CURRENT VERSION OF THE SMART SENSOR
#define INT_CLK         0x00    //External Clock. 0 = the accelerometer runs off internal clock (default)
#define NORMAL_NOISE    0x00    // Bits 4 and 5 select LOW_NOISE Selects Power vs. Noise Tradeoff: 00=Normal (default and lower power consumption)
#define LOW_NOISE       BIT4    // 01 = Low noise mode.
#define ULTRA_LOW_NOISE BIT5    // 10 = Ultralow noise mode.
#define WAKEUP_MODE     BIT3    // Wake-Up Mode.  1 = the part operates in wake-up mode.
#define AUTOSLEEP       BIT2    // Autosleep. Activity and inactivity detection must be in linked mode or
                                // loop mode (LINK/LOOP bits in ACT_INACT_CTL register) to enable
                                // autosleep; otherwise, the bit is ignored. 1 = autosleep is enabled, and
                                //the device enters wake-up mode automatically upon detection of inactivity.
#define ADXL_STANDBY    0x00    // Bit 0 and Bit1 Selects Measurement Mode or Standby. Start up the chip in Standby, and then switch to Measurement
#define ADXL_ON         BIT1    // Start recording data in measurement mode


#define THRESH_ACT_L    0x00 // low byte of the activity threshold (default is 0x00) The sensor only records data when over the threshold
#define THRESH_ACT_H    0x00 // high byte (lowest three bits) of the activity threshold (default is 0x00)
#define TIME_ACT        0x00 // When this timer is used, only sustained motion can trigger activity detection (default is 0x00)
#define THRESH_INACT_L  0x00 // low byte of inactivity threshold (default is 0x00) The sensor stops recording data when under the threshold
#define THRESH_INACT_H  0x00 // high byte (lowest three bits) of the inactivity threshold (default is 0x00)
#define TIME_INACT_L    0x00 // low byte of inactivity timer (default is 0x00) The sensor goes to sleep after reading this many samples of inactivity
#define TIME_INACT_H    0x00 // high byte (lowest three bits) of the inactivity timer (default is 0x00)
#define ACT_INACT_CTL   0x00 // activity/inactivity control byte.  (default is 0x00 -- not activated)
#define FIFO_CONTROL    (FIFO_AH + FIFO_OLDEST_SAVED) // more than 256 bytes, and oldest save mode.  No temperature readings in the FIFO (once a record is enough)
#define FIFO_SAMPLES    44   //  300 FIFO samples (3 X 100) // Along with AH, this is the number of two byte samples to save in the buffer (subtract 256 if AH is set).
                             // Should be multiple of 3 unless FIFO_TEMP_ON is added to FIFO_CONTRO, then it should be a multiple of 4.  511 maximum.
                             // Optimizing this so it is not much bigger than needed will make the sensor run on a little less power.
#define INTMAP1         0x00 // no interrupts for now.  This interrupt is not connected to the MSP430
#define INTMAP2         0x00 // This interrupt is not currently used.  It can send interrupts to the MSP430, or used to sync a clock from the MSP430
#define FILTER_CTL      (RANGE_2G | HALF_BW | SAMPLE_SPEED)         // 2g sensitivity, normal anti-aliasing and 100 Hz sample rate
#define POWER_CTL_OFF   (INT_CLK | ULTRA_LOW_NOISE | ADXL_STANDBY)  // Startup in standby, and then change the ADXL_ON bit when ready
#define POWER_CTL_ON    (INT_CLK | ULTRA_LOW_NOISE | ADXL_ON)       // The byte to send when ready to read data


/////////////////////DATA RECORDING PARAMETERS//////////////////////////////////
    // SAMPLE_SPEED sets the OUTPUT DATA RATES (ODR) for the ADXL sensor (below)
    // The sample speed is the rate of sensor reads (12.5, 25, 50, 100, 200 or 400 Hz)
    // 0 = 12.5 Hz, 1 = 25 Hz, 2 = 50 Hz, 3 = 100 Hz,  4 = 200 Hz and 5 = 400 Hz
    // Bits 0, 1 and 2  of FILTER_CTL select the Output Data Rate.
    // which also configures internal filters to a bandwidth using the QUARTER_BW or the HALF_BW bit setting.
    #define ODR_12          0x00    // 12.5 Hz ODR
    #define ODR_25          BIT0    // 25 Hz ODR
    #define ODR_50          BIT1    // 50 Hz ODR (reset default)
    #define ODR_100    (BIT1|BIT0)  // 100 Hz ODR
    #define ODR_200         BIT2    // 200 Hz ODR
    #define ODR_400    (BIT2|BIT0)  // 400 Hz ODR


#define SAMPLE_SPEED    ODR_100 // Output Data Rate (see above)
#define READ_SPEED      1       // the number of sets of buffer reads per second.
#define SLEEP_INTERVALS 8       // Must be a power of 2.  REED_SPEED X SLEEP_INTERVALS should equal 8
    // The timer is set to have an interrupt every 1/8 of a second, so SLEEP_INTERVALS
    // is number of interrupts to wait before emptying the buffer.
    // Each set of sensor readings is 6 bytes.  2 bytes for each sensor read of X, Y and Z acceleration
    // Temperature is only read once for each record.
    // The ADXL FIFO buffer is set to hold a maximum of 440 readings (a maximum of 100 sensor reading sets of 4)
    // If the speed is 100 Hz or less the buffer can be emptied once per second (SLEEP_INTERVALS = 8)
    // For 200 reads, this should be set to 2 buffer downloads (SLEEP_INTERVALS = 4)
    // For 400 reads, this should be set to 4 buffer downloads. (SLEEP_INTERVALS = 2)
    // the number of samples per second times SLOWHOURSBINSEC can not be more than 64K or bins might overflow



////////////// SOFTWARE DEFINITIONS ///////////////////////////////////////////


    // Logic
#define YES 1
#define NO 0
#define TRUE 1
#define FALSE 0

#define DEFAULT_SLAVE 0xFF      //  All slave units start out as 255 (0xFF) and get redefined when connected to masters.
#define DEFAULT_MASTER 0x00     //  0 is for a master specific for the same bank.


// Message Protocol
    // Old protocol (might still be used)
#define SEND 1
#define RECEIVE 0

    //Protocol 0xFF -- basic communication between masters and slaves

#define START_BYTE 0x00         // Byte 0: This is always a zero, and is sent to wake up units if they are asleep.
#define PROTOCOL_BYTE 0xFF      // Byte 1: This identifies the protocol being used.  This one is 0xFF
#define ALL_BANKS 0xFF          // Byte 2: Identifies which bank of devices is being called.  0xFF calls all banks
#define ALL_UNITS 0xFF          // Byte 3: Identifies which device is being called.  0xFF calls all units.
// TURTLE_SENSE_BANK            // Byte 4: Identifies the bank of who is calling. 0x00 are masters.  0xFF is invalid.
//      THIS_UNIT               // Byte 5: Identifies the device that is calling. 0x00 in this case is a master.
#define CORE_COMMANDS 0x00      // Byte 6: Identifies the bank of commands being sent:
                                //          0x00 are core commands found in all devices that handle basic communication.  These do not get overwritten
                                //          0x01 are TURTLE SENSE specific commands
                                //          0x02 and up are available for other users
                                // Byte 7: Identifies the command number in the bank of commands.

    // Short and Long Command protocols differ after the first 8 bytes:

        //Short Commands (Byte 10 is zero)
                            // Byte 8: Checksum (high byte) of bytes 1 through 8
                            // Byte 9: Checksum (low byte)
                            // Byte 10: Last Byte -- Always a zero
        //Long Commands (Byte 10 is non zero)
                            // Byte 8-9: two parameters that can be sent with the data if needed
                            // Byte 10: Continuation byte -- non zero
                            // Byte 11 - 12: the number of bytes of attached data.  The number of data bytes that follow these command bytes.
                            // Byte 13 - 16: a 4 byte checksum of the command (high to low)
                            // Byte 17 to (Data length plus 16).  The data attached to this message.  If only parameters are being used,
                            //              there is no data attached and bytes 11 through 14 will be zero.
                            // Last byte (Data length plus 17) -- Always a zero

////////////  COMMANDS/////////////////////////////////////////
//  Commands   // This is the list of core commands in bank 0

#define CORE_COMMAND_BANK       0x00    // in bank 0, commands lower than 0x80 are all short commands requests or short reponses.
                                        // 0x80 and above are long commands
#define SYNCH                   0x00    // Reset timer at receipt of command using default settings (always first command sent)
#define REGISTERED              0x01    // sent by slave acknowledging successful registration
#define STATUS                  0x02    // Request status from slave
#define READY                   0x03    // Slave replies "Not busy, ready for instructions"
#define BUSY                    0x04    // Slave replies "Busy recording data"
#define DATA_READY              0x05    // Slave replies "Have data ready to send"
#define REQUEST_ID              0x06    // Please send (or resend) identification
#define REQUEST_PARAMETERS      0x07    // Please send (or resend) parameters
#define REQUEST_DATA            0x08    // Please send all records from the smart sensor
#define GOOD_TRANSMISSION       0x09    // Acknowledgement that the last transmission of data was received without error
#define DATA_ERROR              0x0A    // Format or Checksum Error with last transmission /resend
#define RESEND_LAST             0x0A    // Repeat the last transmission
#define REQUEST_ERROR_INFO      0x0B    // Send information about the last error
#define TURN_ADXL_OFF           0x0C    // Hardware power cycle -- powers down ADXL
#define TURN_ADXL_ON            0x0D    // turn on the ADXL sensor in low power standby without changing any settings
#define EMPTY_TIME_SLOT         0x0E    // sent by master to indicate the slot is available
#define MOISTURE_OFF            0x10    // turns off the moisture detection
#define MOISTURE_ON             0x11    // turns on moisture detection
#define CLEAR_LAST_REC          0x12    // erase the last record on the smart sensor
#define CLEAR_ALL               0x13    // clear the entire smart sensor memory
#define SEND_LAST_REC           0x14    // upload the last record from the smart sensor
#define SEND_BLOCK              0x15    // sends a block of records (can be defined by command 0x95)
#define MANUAL_OPERATION        0x16    // Read or write data directly to and from the ADXL sensor without processing
#define STREAM_DATA             0x17    // Streams of data from the smart sensor
#define START_RUN               0x18    // start up the smart sensor and collect integrated data (Phase III data)
#define RESUME_RUN              0x19    // continues a run after an interruption
#define RUN_BINS                0x1A    // start up the smart sensor and collect data in bins (Phase II data)
#define STOP_RUN                0x1B    // stop data collection method currently running
#define SUSPEND                 0x1C    // sleep until a wake-up command is sent (during report generation)
#define WAKE_UP                 0x1E    // wakes up all units or just one
#define RESET_ADXL              0x1F    // does a software reset of the ADXL without power cycling
#define SIGN_OFF                0x20    // communication completed without error, signing off (from master)


// Long Commands   // This is the list of core commands in bank 0 sending information
#define SYNCH_INTERVALS         0x80    // Reset timer, number of intervals, and clock parameters to adjust synch
#define SUSPEND_INTERVALS       0x81    // Do not expect any transmissions for specified number of 6 sec. intervals
#define IDENTIFICATION          0x86    // Sending nest identification about this sensor (both_directions)
#define PARAMETERS              0x87    // Sending operational parameters (both directions)
#define DATA                    0x88    // Sending data (slave-->master)
#define ERROR_INFO              0x8B    // Data Error details (not format or checksum error with last transmission)
#define CALIBRATE_ADXL          0x8C    // calibrate the ADXL chip (temperature settings)
#define REQUEST_TIME_SLOT       0x8E    // sent by unregisterred devices to claim empty time slots with IDENTIFICATION
#define REGISTRATION            0x8F    // sent by master to sensor (using sensor's ID) to register sensor with a new unit number and a time slot.
#define RTC_SYNCH               0x92    // Send RTC data after registerring sensor
#define DEFINE_BLOCK            0x95    // Defines the parameters for transferring blocks of data or programming
#define RESET_ADXL_SETTINGS     0x9F    // does a hardware and software reset of the ADXL and puts it in standby and loads settings
#define LOCK_USER_MEM           0xAA    // Locks the user memory using a password
#define PROGRAM_INFO            0xBB    // Sending program info (master-->slave) bank, lenghth, destination, etc...
#define PROGRAM_DATA            0xBC    // Sending program data
#define UNLOCK_USER_MEM         0xBD    // Unlocks the user memory using a password

// Error codes   //
#define NO_ERROR                0x00    // Everything is as expected
#define WRONG_COMMAND           0x01    // Transmission was OK, but an unexpected command was received
#define TIMEOUT_ERROR           0x02    // Timeout counter ranout before response received
#define PROTOCOL_ERROR          0x04    // Data did not conform to protocol
#define CHECKSUM_ERROR          0x08    // Bad transmission, checksum was incorrect
#define WATCHDOG_RESET_ERROR    0x10    // Program reset by watchdog timer
#define OUT_OF_RANGE_ERROR      0x20    // A parameter was out of range
#define BAD_COMMAND_ERROR       0x40    // an undefined command was called
#define ADDRESS_ERROR           0x80    // Communication was addressed to a different device



// Smart Sensor Interrupt codes sent as parameters
// These codes are returned during a manual run
// they explain the reason for generating an interrupt
#define NO_INTERRUPT          0x00      // = No interrupt was generated -- operation in progress
#define INTERRUPT_ONE         0x01      // = INT1 generated an interrupt
#define INTERRUPT_TWO         0x02      // = INT2 generated an interrupt






////////////// PROGRAM TIMINGS///////////////////////////
    // Default timings  (You can change these)
#define HOURS 4         // Default is 4  // Number of hours for data collection per call-in period once motion detected.  This must be a factor of 120.
#define SLOWHOURS 24        // Default is 24 // Number of hours in a slow day of data collection
#define SLOW_DAYS 40    // Number of days that should elapse until there is more frequent reporting

    // These are computed (don't change them)
#define BINSEC (15 * HOURS)     // The default is 60 // Number of seconds in each bin record.  Must be factor of 3600
#define SLOWHOURS_BIN_SEC (15 * SLOWHOURS)  // The default is 360 but it could be less if records are also stored in comm board or phone board.
                                // Number of seconds in each bin record on a slow day (every 6 minutes)

    // These can be changed, but the program may need to be adjusted (change with caution)
#define MAX_BIN 10              // number of histogram bins kept in each record (changing this might cause problems -- check thoroughly)
#define DATA_SIZE 16            // This is MAX_BIN plus other parameters (currently 6: Temp, X, Y, Z, Count, Max)
#define DATA_BYTES 32           // The number of bytes in one record (DATA_SIZE * 2)
#define MAX_RECORDS 240         // The default is 240 // The maximum number of Bin records assuming about 8 K and 32 bytes per record


    // System parameters
#define REPORT_HEADINGS 1       // flags if headings are printed in the reports (1=YES, 0 = NO)
#define CABLE_LENGTH 50         // the maximum cable length in feet (we roughly need 1 microsecond to discharge each foot of cat5e cable )
                                // if you want to be more precise the actual value is .75 microsecond per foot
#define CABLE_DISCHARGE_TIME (50 + CABLE_LENGTH) // The capacitor in the circuit takes 50 microseconds to discharge
#define SHUT_OFF_LEVEL 549      // Battery capacity left at shut-off (2%)
                                // 12V NiMH (8 AA * 1.2V) = 549 which is about 8 volts
                                // 9V  NiMH = (7 * 1.2V) = 480 which is about 7 volts

    //  Computed Default timings (Don't change these, change the ones above)
#define SECS_IN_DAY 86400       // Number of seconds in a day
#define TIME_OUT  256           // resets if no readings after this many seconds after expected
#define MAXRUN (SECS_IN_DAY-60) // The maximum length of a run before an interrupt (a day minus a minute)
#define MAXRUN4H ((MAXRUN & 0xFF000000) / 0x01000000) // The highest byte of MAXRUN
#define MAXRUN3  ((MAXRUN & 0x00FF0000) / 0x00010000)
#define MAXRUN2  ((MAXRUN & 0x0000FF00) / 0x00000100)
#define MAXRUN1L  (MAXRUN & 0x000000FF)         // The lowest byte of MAXRUN
#define BINSEC_HI ((BINSEC & 0xFF00) / 0x0100)  // The high byte of BIN_SEC
#define BINSEC_LO (BINSEC & 0x00FF)             // The low byte of BIN_SEC
#define SLOWBIN_HI ((SLOWHOURS_BIN_SEC & 0xFF00) / 0x0100)  // The high byte of SLOWHOURS_BIN_SEC
#define SLOWBIN_LO (SLOWHOURS_BIN_SEC & 0x00FF) // The low byte of SLOWHOURS_BIN_SEC
#define MAX_RECORDS_HI ((MAX_RECORDS & 0xFF00) / 0x0100)    // The high byte of MAX_RECORDS
#define MAX_RECORDS_LO  (MAX_RECORDS & 0x00FF)  // The low byte of MAX_RECORDS


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////  FRAM VARIABLES /////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

//  start FRAM persistent data storage section

    #pragma SET_DATA_SECTION(".TI.persistent")

// constants but both sets of code should be on both slaves and masters
    volatile unsigned char not_master = THIS_UNIT;         // 0 is a master, 1 is a slave.  Behaves differently with each
    volatile unsigned char not_slave = !THIS_UNIT;         // 0 is a slave, 1 is a master.
//  variables
    volatile unsigned char not_ready, coax_char_received, in_conversation, message_in_progress, waiting_for_coax_data,
                sensor_ready, tamper_count, transmission_done, transmitting, time_to_phone_in, sec_interrupt,
                sending_msg, receiving_msg, new_parameters_loaded, time_to_wakeup, urgent_alert, not_sleeping, waiting,
                force_shut_down, wd_reset, starting_up, sensor_plugged_in, coax_buffer_full;

    volatile unsigned char led_count, led_start_count, ping_count, ping_start_count;      // counts clock interrupts every eighth of a second, wakes up every 8 counts
    volatile unsigned char bad_transmission, processing_command, coax_quiet;     //status flags for error and ready to process
    volatile unsigned int bytecount;                    // counts the number of bytes in the current received command
    volatile unsigned int received_datalength;          // max 64K?
    volatile unsigned long int running_checksum, received_checksum; // keep a checksum to look for conversation errors
    volatile unsigned char parameter_one, parameter_two;    // optional parameters for long commands
    volatile unsigned char command_type;    // 0 is a short command, 1 is a long command
    volatile unsigned char *data_ptr;                   // pointer to the buffer
    volatile unsigned char *message_ptr;
    volatile unsigned char transmit_byte, last_transmit_byte;       // pointer to the message we are sending, and the byte being sent
    volatile unsigned int message_count, message_length; // count the chars sent in the message up to the message length
    volatile unsigned int data_count, data_length;      // count the chars sent in the data up to the data length
    volatile unsigned char not_timed_out;                     // timeout flag
    volatile unsigned char pause;                       // flag if we are sleeping or waiting
    volatile unsigned char there_is_data;               // flag set when there is data in the buffer
    volatile unsigned char going_to_bank;
    volatile unsigned char going_to_unit;    //received device is two bytes to identify who we are sending a message to -- Bank and number
    volatile unsigned char this_bank;
    volatile unsigned char this_unit;
    volatile unsigned char current_command_bank;
    volatile unsigned char received_command_bank, received_command;
    volatile unsigned char received_caller_bank, received_caller_unit;
    volatile unsigned char received_recipient_bank, received_recipient_unit;
    volatile unsigned char received_protocol; // data from the last message received
    volatile unsigned char received_command_type;
    volatile unsigned int coax_tx_buff_len;          // The length of the buffer for transmitting to UCA0
    volatile unsigned char *coax_tx_buff_ptr;        // The pointer to the transmit buffer.
    volatile unsigned char coax_char_received;
    volatile unsigned char interval_count;       // Clock timing.  Each interval (1/8 sec) creates an interrupt
    volatile unsigned char interval_limit;       // The intervals are counted and counting restarts when the limit is reached.
    volatile unsigned char wait_until_cycle;     // the interval count we are waiting for
    volatile unsigned char timeout_during_rx ; // flags for synching
    volatile unsigned char tx_error;            // error code for last transmission
    volatile unsigned char error_count;
    volatile unsigned char status;                // the status of the sensor (ready, busy, etc...)
    volatile unsigned char connection_code;
    volatile unsigned char suspended;               //if the sensor should not expect messages
    volatile unsigned char intervals_on;
    volatile unsigned char reporting_data, new_census, new_device_count, data_ready, time_to_resume;
    volatile unsigned int  random;
    volatile unsigned char random_wait;
    volatile unsigned char registering;


#define SENSOR_SLOTS 0x30   // start out seeing

     unsigned char slot_count;                      // a counter to find slots
     unsigned char sensor_slot[SENSOR_SLOTS];       //0 = empty, 1 = taken, 2 = suspended
     unsigned char sensor_status[SENSOR_SLOTS];     // 0 = no unreported change, 1 = recently connected, 2 = recently disconnected
#define NO_UNREPORTED_CHANGE 0
#define RECENTLY_CONNECTED 1
#define RECENTLY_DISCONNECTED 2
#define NEW_PARAMETERS_FOUND 3
     unsigned char sensor_ids[SENSOR_SLOTS][9];     //sensor serial numbers in each slot (or most recent ID)
     unsigned char device_count;                    // the number of devices connected to the master
     unsigned int battery_level;                    // 0 = battery dead, 100 = fully charged
     unsigned int battery_percents[2][36];// = {
     // {0,1,2,3,4,5,6,7,8,9,10,12,15,20,25,30,40,50,60,70,75,80,85,88,90,91,92,93,94,95,96,97,98,99,100,101},
     // {650,670,675,etc...| }};
//    unsigned int recUploads = 0;                  // The number of records that have been uploaded
//    unsigned long int recReadings = 0;            // The number of readings that have been processed in the current record
    volatile unsigned int recCount = 0;                      // The count of how many records have been collected since last upload
//    unsigned int recCount_here = 0;               // The number of records stored on this device
//    unsigned int sensor_recCount = 0;             // The number of records stored on the smart sensor
//    unsigned int recCount_phone = 0;              // The number of records stored on the phone module
//    unsigned int room_here = MAX_RECORDS;         // The room remaining on this device for records
    volatile unsigned long int secCount = SECS_IN_DAY + TIME_OUT; // time out counter -- reset after each data upload
    volatile unsigned long int maxTime = SECS_IN_DAY;         // default time out is a day and 256 seconds
    volatile unsigned long int secMax = SECS_IN_DAY + 60;     // a minute more than a day
    volatile unsigned int day_count = 0;                     // Number of days the nest has been active
    volatile unsigned int last_day_count = 0;                // Number of days the nest was active when last reported
//    unsigned int recSecs = 0;                     // The number of seconds that the current record has been active
//    unsigned const char active = NO;              // "Yes" if the turtles are starting to move,  "No" if they are not
    volatile unsigned char error_code = 0x00;                // for future error handling

    // SENSOR VALUES

    // TEMPERATURE CALIBRATION SETTINGS
    // During Calibration, the reading at 0 degrees C becomes the temperature offset, and the ratio of 0x0200
    // divided by the reading at room temperature (minus the offset) becomes the temperature ratio.
#define CALIBRATE_TEMP YES                  // The default is to calibrate the sensors


    // NEST IDENTIFICATION and SENSOR ID

    // This has the UTC time followed by the date of activation and serial number
    // See the top of the program to set the serial number.

//#define NEST_ID "123456.000,060914,S-AA0001"
    volatile char serial_ID[] = SERIAL_NUMBER;  // This won't actually be used so it should never be altered
    unsigned char serial_num[] = SERIAL_NUMBER;     // Store a copy of a SERIAL_NUMBER that cannot be altered

    unsigned char nest_ID[] = NEST_ID;          // The unique ID of each nest based on time, date, and sensor ID
    // an example of the NEST_ID is "123456.890,060914,S-AA0001"
    // it breaks down as follows:
#define NEST_ID_BYTES 26    // the number of bytes in the nest_ID above
    // Starts with the UTC time, and GPS location copied directly from the GPS output.
    // The UTC is in the format: hhmmss.sss
#define UTC 0               // nest_ID[UTC] - The offset for the GPS UTC time code
#define UTC_BYTES 6         // The last byte is nest_ID[9], but the last 3 digits always seem to be zeros
    // The DATE is the date the sensor was activated in the format: ddmmyy  where:
    // dd - day 01..31
    // mm - month 01..12
    // yy - year 00..99 - 2000 to 2099
#define DATE 11             // nest_ID[DATE] - The date the sensor was activated by this comm tower
#define DATE_BYTES 6        // The last byte is nest_ID[16]
#define DAY 11
#define DAY_BYTES 2
#define MONTH 13
#define MONTH_BYTES 2
#define YEAR 15
#define YEAR_BYTES 2
    // The SERIAL NUMBER is a unique id for each Smart Sensor produced
    // The first letter is the hardware version
    // The second letter is the software version
    // The next 4 digits is a unique production number for each device
#define SERIAL 18           // nest_ID[SERIAL] - The offset for the sensor serial number nest_ID[SERIAL]
#define SERIAL_BYTES 8      // The last byte is nest_ID[25]

    // SMART SENSOR PARAMETERS
    // These 40 bytes can be downloaded from the comm tower to change the behavior of the sensor.
unsigned char parameters[40] = {
        THRESH_ACT_L, THRESH_ACT_H, TIME_ACT, THRESH_INACT_L,
        THRESH_INACT_H, TIME_INACT_L, TIME_INACT_H,
        ACT_INACT_CTL, FIFO_CONTROL, FIFO_SAMPLES,
        INTMAP1, INTMAP2, FILTER_CTL, POWER_CTL_OFF, // parameters[0] to [13] - the first 14 are all the ADXL reset parameters.
        SLEEP_INTERVALS,                // parameters[14] - The number of clock interrupts between each FIFO buffer read.
        READ_SPEED,                     // parameters[15] - The number of interrupts per second (default is 8 which equals 1/8 second)
        SLOWBIN_LO, SLOWBIN_HI,         // parameters[16] and [17] - The current number of seconds to accumulate readings in each record (set of bins)
        MAXRUN1L, MAXRUN2, MAXRUN3, MAXRUN4H,   // parameters[18] to [21] - The maximum time the sensor can run without reporting back to comm head.
        MAX_RECORDS_LO, MAX_RECORDS_HI, // parameters[22] and [23] - The maximum number of records to collect
        CALIBRATE_TEMP,                 // parameters[24] - Is the temperature calibration active (YES=calibrate)
        REPORT_HEADINGS,                // parameters[25] - (comm board) Flag if we are sending headings in report (YES=headings)
        BINSEC_LO, BINSEC_HI,           // parameters[26] and [27] timing settings for active days
        SLOW_DAYS,                      // parameters[28] - Number of days of low activity
        SLOWBIN_LO, SLOWBIN_HI,         // parameters[29] and [30] - The number of seconds to accumulate readings in each record (set of bins)
        0,0,0,0,0,0,0,0,0 };            // parameters[31] to [39] reserved for future use
#define PARAM_BYTES 40 // the number of bytes in the parameters structure above
     // new parameters can be ftp'd into the unit from the web
unsigned char new_parameters[40]= {
        THRESH_ACT_L, THRESH_ACT_H, TIME_ACT, THRESH_INACT_L,
        THRESH_INACT_H, TIME_INACT_L, TIME_INACT_H,
        ACT_INACT_CTL, FIFO_CONTROL, FIFO_SAMPLES,
        INTMAP1, INTMAP2, FILTER_CTL, POWER_CTL_OFF,
        SLEEP_INTERVALS,
        READ_SPEED,
        SLOWBIN_LO, SLOWBIN_HI,
        MAXRUN1L, MAXRUN2, MAXRUN3, MAXRUN4H,
        MAX_RECORDS_LO, MAX_RECORDS_HI,
        CALIBRATE_TEMP,
        REPORT_HEADINGS,
        BINSEC_LO, BINSEC_HI,
        SLOW_DAYS,
        SLOWBIN_LO, SLOWBIN_HI,
        0,0,0,0,0,0,0,0,0 };

#define PARAM_THRESH_ACT_L      parameters[0]
#define PARAM_THRESH_ACT_H      parameters[1]
#define PARAM_TIME_ACT          parameters[2]
#define PARAM_THRESH_INACT_L    parameters[3]
#define PARAM_THRESH_INACT_H    parameters[4]
#define PARAM_TIME_INACT_L      parameters[5]
#define PARAM_TIME_INACT_H      parameters[6]
#define PARAM_ACT_INACT_CTL     parameters[7]
#define PARAM_FIFO_CONTROL      parameters[8]
#define PARAM_FIFO_SAMPLES      parameters[9]
#define PARAM_INTMAP1           parameters[10]
#define PARAM_INTMAP2           parameters[11]
#define PARAM_FILTER_CTL        parameters[12]
#define PARAM_POWER_CTL_OFF     parameters[13]
#define PARAM_SLEEP_INTERVALS   parameters[14]
#define PARAM_READ_SPEED        parameters[15]
#define PARAM_SLOWBIN_LO_NOW    parameters[16]
#define PARAM_SLOWBIN_HI_NOW    parameters[17]
#define PARAM_MAXRUN1L          parameters[18]
#define PARAM_MAXRUN2           parameters[19]
#define PARAM_MAXRUN3           parameters[20]
#define PARAM_MAXRUN4H          parameters[21]
#define PARAM_MAX_RECORDS_LO    parameters[22]
#define PARAM_MAX_RECORDS_HI    parameters[23]
#define PARAM_CALIBRATE_TEMP    parameters[24]
#define PARAM_REPORT_HEADINGS   parameters[25]
#define PARAM_BINSEC_LO         parameters[26]
#define PARAM_BINSEC_HI         parameters[27]
#define PARAM_SLOW_DAYS         parameters[28]
#define PARAM_SLOWBIN_LO        parameters[29]
#define PARAM_SLOWBIN_HI        parameters[30]

#define NEW_THRESH_ACT_L      new_parameters[0]
#define NEW_THRESH_ACT_H      new_parameters[1]
#define NEW_TIME_ACT          new_parameters[2]
#define NEW_THRESH_INACT_L    new_parameters[3]
#define NEW_THRESH_INACT_H    new_parameters[4]
#define NEW_TIME_INACT_L      new_parameters[5]
#define NEW_TIME_INACT_H      new_parameters[6]
#define NEW_ACT_INACT_CTL     new_parameters[7]
#define NEW_FIFO_CONTROL      new_parameters[8]
#define NEW_FIFO_SAMPLES      new_parameters[9]
#define NEW_INTMAP1           new_parameters[10]
#define NEW_INTMAP2           new_parameters[11]
#define NEW_FILTER_CTL        new_parameters[12]
#define NEW_POWER_CTL_OFF     new_parameters[13]
#define NEW_SLEEP_INTERVALS   new_parameters[14]
#define NEW_READ_SPEED        new_parameters[15]
#define NEW_SLOWBIN_LO_NOW    new_parameters[16]
#define NEW_SLOWBIN_HI_NOW    new_parameters[17]
#define NEW_MAXRUN1L          new_parameters[18]
#define NEW_MAXRUN2           new_parameters[19]
#define NEW_MAXRUN3           new_parameters[20]
#define NEW_MAXRUN4H          new_parameters[21]
#define NEW_MAX_RECORDS_LO    new_parameters[22]
#define NEW_MAX_RECORDS_HI    new_parameters[23]
#define NEW_CALIBRATE_TEMP    new_parameters[24]
#define NEW_REPORT_HEADINGS   new_parameters[25]
#define NEW_BINSEC_LO         new_parameters[26]
#define NEW_BINSEC_HI         new_parameters[27]
#define NEW_SLOW_DAYS         new_parameters[28]
#define NEW_SLOWBIN_LO        new_parameters[29]
#define NEW_SLOWBIN_HI        new_parameters[30]

    // AT codes and strings for phone  -- the back slash is for adding quotes \r = CR
    unsigned char *messages[] = {
            "\r",               // messages[0] Not used (marks end of script)
            "ATE0 V0\r",        // messages[1] Turn off echo, return error codes only 0=OK, 4=error
                // The following line should be modified for different internet providers.
            "AT#SERVINFO\r",    // messages[2] Get cellular service info
            "AT#SGACT=1,1\r",   // messages[3] Activate context
            "AT#NITZ=1,0\r",    // messages[4] Set RTC from network time automatically
            "AT#FTPTYPE=0\r",   // messages[5] File type = Binary
            "AT#FTPTYPE=1\r",   // messages[6] File type = ASCII
            FTP_ACCOUNT,        // messages[7] Update FTP account details at the beginning of this code file
            ISP_PROVIDER,       // messages[8] Update ISP_PROVIDER at the beggining of this code file
            "+++",              // messages[9] Upload escape sequence
            "AT#FTPCLOSE\r",    // messages[10] Close FTP session
            "AT#CCLK?\r",       // messages[11]  get time from RTC
            "AT$GPSP=0\r ",     // messages[12] GPS off
            "AT$GPSP=1\r",      // messages[13] power-up GPS
            " ",                // messages[14]
            "AT+CREG?\r",       // messages[15] Registration report (will return "+CREG: 0,1" when successful.)
                                //      CREG codes
                                //      0 - not registered, ME is not currently searching a new operator to
                                //          register to (bad reception?  antenna disconnected?
                                //      1 - registered, home network
                                //      2 - not registered, but ME is currently searching a new operator to
                                //           register to
                                //      3 - registration denied
                                //      4 - unknown
                                //      5 - registered, roaming
            " ",                // messages[16]
            " ",                // messages[17]
            " ",                // messages[18]
            " ",                // messages[19]
            " ",                // messages[20]
            " ",                // messages[21]
            " ",                // messages[22] LAST AT command
            "\"\r",             // messages[23] end quotes and CR (end of file name -- special case for error checking)
            "\r",               // messages[24] CR
            "AT#FTPAPP=\"",     // messages[25] Append to existing file and upload (file name must be added)
            "AT#FTPGET",        // messages[26] Read parameter file
            "AT#FTPDELE",       // messages[27] Delete parameter file
            "Report: ",         // messages[28]
            "Sensor ID#: ",             // messages[29]
            "S-AA0000",                 // messages[30] The serial number of the smart sensor
            "Today's date: ",           // messages[31]
            "2016-01-01",               // messages[32] Today's date
            "Report #: ",               // messages[33]
            "01",                       // messages[34]  TODAYS_NUM -The number of reports sent today
            " ",                        // messages[35]
            "Comm ID#: ",               // messages[36]
            SERIAL_NUMBER,              // messages[37] The serial number of the master
            "Nest location: ",          // messages[38]
            "0000.0000N",               // messages[39] Nest GPS latitude
            "00000.0000W",              // messages[40] Nest GPS longitude
            "Start date/time: ",        // messages[41]
            "2000-00-00,00:00:00",      // messages[42] Time of previous report from GPS
            "Report date/time: ",       // messages[43]
            "2001-01-01,01:01:00",      // messages[44] Time of current report from GPS
            "Secs per rec: ",           // messages[45]
            "\r# of recs: ",            // messages[46]
            "\"\r",                     // messages[47] like #23, but without waiting for a response
            "\rBattery level: ",        // messages[48]
            "Rec#,Temp,   X,   Y,   Z, Cnt, Max,Bins: (Low) to (High)\r",   // messages[49]
            ",",                        // messages[50] just a comma
            // File type codes:
            //          Used in file names to distinguish the different types of files generated
            //          TODO // finish creating header files, log files and alert files.
            "_r-",                  // messages[51]  "r-" -- a standard report
            "=\"parameters",        // messages[52] parameter file name beginning
            ".ts",                  // messages[53] parameter file name ending
            "TurtleSense 0.2504 -- CC 4.0 BY-SA NerdsWithoutBorders.Net\r", // messages[54] COPYLEFT NOTICE
            "--end of report--\r",  // messages[55]
            ",  ",                  // messages[56] comma and a space
            "_",                    // messages[57] underscore
            ".txt",                 // messages[58] text file extension
            "NEW SETTINGS LOADED.\r",       // messages[59]
            "LOW BATTERY.\r",               // messages[60]
            "RESET OCCURRED.\r",            // messages[61]
            "CONNECTED - MONITORING.\r",    // messages[62]
            "DISCONNECTED.\r",              // messages[63]
            "# ,Energy, Max, Wet,Temp,   X,   Y,   Z, Max,ODR2,ODR3,ODR4,ODR5,ODR6", // messages[64]
            "NEST DATA LOADED",             // messages[65]
            " ",                            // messages[66]
            "Energy samples per second (ODR): ",    // messages[67]
            "ALL\r",        // messages[68]
            "12.5",         // messages[69]
            " 25",          // messages[70]
            " 50",          // messages[71]
            "100",          // messages[72]
            "200",          // messages[73]
            "400",          // messages[74]
            "SENSOR_LOG_",  // messages[75]
            "ACTIVITY_LOG_",// messages[76]
            "EVENT: "       //messages[77]
            };
                                    // If the command is complete it ends with a CR (\r)

     /////////////////Script Tokens//////////////////////

#define END_OF_SCRIPT       0   // not used (marks end of script)
#define AT_NO_ECHO          1   // turn off echo, return error codes only 0=OK, 4=error
#define AT_CELL_INFO        2   // Get cellular service info
#define AT_ACTIVATE         3   // AT commandt to Activate context
#define AT_SET_RTC          4   // Set RTC from network time automatically
#define AT_BINARY_FILE      5   // File type = Binary
#define AT_ASCII_FILE       6   // File type = ASCII
#define AT_FTP_ACCOUNT      7   // The FTP account details specified at the beginning of this code file
#define AT_ISP_PROVIDER     8   // The ISP_PROVIDER specified at the beggining of this code file
#define AT_FTP_ESCAPE       9   // Upload escape sequence
#define AT_FTP_CLOSE        10  // Close FTP session
#define AT_GET_TIME         11  // Get time from RTC
#define AT_GPS_OFF          12  // GPS off
#define AT_GPS_ON           13  // GPS on
#define AT_14               14  // Unused.
#define AT_CREG             15  // Registration report (will return "+CREG: 0,1" when successful.) Codes listed below
#define AT_16               16  // Unused.
#define AT_17               17  // Unused.
#define AT_18               18  // Unused.
#define AT_19               19  // Unused.
#define AT_20               20  // Unused.
#define AT_21               21  // Unused.
#define AT_22               22  // Unused.  LAST AT command
#define END_OF_FILENAME     23  // End quotes and CR (end of file name -- special case for error checking)
#define CR                  24  // Carriage return
#define AT_FTP_APPEND       25  // Append to existing file and upload (file name must be added)
#define AT_FTP_GET          26  // Read parameter  file
#define AT_FTP_DELETE       27  // Delete parameter file
#define TXT_REPORT          28  // "Report: "
#define TXT_SENSOR_ID       29  // "Sensor ID#: "
#define SENSOR_ID           30  // "000000" -- The serial number of the smart sensor
#define TXT_TODAYS_DATE     31  // "Today's date: "
#define DATE_TODAY          32  // "2014-05-01" -- The day the sensor was installed
#define TXT_REPORT_NUM      33  // "Report #: "
#define TODAYS_NUM          34  // "01" -- The number of reports sent today
#define SPACE               35  //  " " -- Blank single space.
#define TXT_COMM_ID         36  //  "Comm ID#: "
#define COMM_ID             37  //  SERIAL_NUMBER
#define TXT_NEST_GPS        38  //  "Nest location: "
#define NEST_LATITUDE       39  //  "0000.0000N" -- Nest GPS latitude
#define NEST_LONGITUDE      40  //  "00000.0000W" -- Nest GPS longitude
#define TXT_START_TIME      41  //  "Start date/time: "
#define START_TIME          42  //  "2000-00-00,00:00:00"  Time that data started being collected in this report
#define TXT_REPORT_TIME     43  //  "Report date/time: "
#define REPORT_TIME         44  //  "2001-01-01,01:01:00"  Time of current report from GPS
#define TXT_SECS_PER_REC    45  //  "Secs per rec: "
#define TXT_NUM_RECS        46  //  "\r# of recs: "
#define CLOSE_QUOTE         47  //  "\"\r" like #26, but without waiting for a response
#define TXT_BATTERY_LEVEL   48  //  "\rBattery level: "
#define TXT_HEADINGS        49  //  "Rec#,Temp,   X,   Y,   Z, Cnt, Max,Bins: (Low) to (High)\r"
#define COMMA               50  //  "," -- just a comma
#define TXT_R_DASH          51  //  "_r-" -- a standard report
#define PARAMETER_FILE      52  //  "=\"parameters" -- parameter file name beginning
#define DOT_TS              53  //  ".ts" -- parameter file name ending
#define COPYLEFT            54  //  "TurtleSense 0.2504 -- CC 4.0 BY-SA NerdsWithoutBorders.Net\r" -- COPYLEFT NOTICE
#define TXT_END             55  //  "--end of report--\r"
#define COMMA_SPACE         56  //  ",  "  -- just a comma and a space
#define UNDERSCORE          57  //  "_" -- just an underscore
#define DOT_TXT             58  //  ".txt"  -- text file extension
#define TXT_NEW_SETTINGS    59  //  "NEW SETTINGS LOADED\r"
#define TXT_LOW_BATTERY     60  //  "LOW BATTERY\r"
#define TXT_RESET           61  //  "RESET OCCURRED\r"
#define TXT_MONITORING      62  //  "CONNECTED -- MONITORING\r"
#define TXT_DISCONNECTED    63  //  "DISCONNECTED\r"
#define TXT_64              64  //
#define TXT_NEST_DATA_FOUND 65  //  "NEST DATA FOUND"
#define TXT_66              66  //
#define TXT_ENERGY_SAMPLES  67  //  "Energy samples per second (ODR): "
#define TXT_ALL             68  //  "ALL\r"
#define TXT_12              69  //  "12.5"
#define TXT_25              70  //  "25"
#define TXT_50              71  //  "50"
#define TXT_100             72  //  "100"
#define TXT_200             73  //  "200"
#define TXT_400             74  //  "400"
#define TXT_SENSOR_LOG      75  //  "SENSOR_LOG_"
#define TXT_ACTIVITY_LOG    76  //  "ACTIVITY_LOG_"
#define TXT_EVENT           77  //  "EVENT: "

     //      CREG codes
#define CELL_NOT_REGISTERED 0       //      0 - not registered, ME is not currently searching a new operator to
                                    //          register to (bad reception?  antenna disconnected?
#define CELL_REGISTERED     1       //      1 - registered, home network
#define SEARCHING           2       //      2 - not registered, but ME is currently searching a new operator to
                                    //           register to
#define REJECTED            3       //      3 - registration denied
#define UNKNOWN             4       //      4 - unknown
#define ROAMING             5       //      5 - registered, roaming

    // Cell message error codes
     //------------
     //  0 OK
     //  1 CONNECT
     //  2 RING
     //  3 NO CARRIER
     //  4 ERROR
     // Counters

     /// ASCII CHARACTERS
#define ASCII_CR            13  // Carriage Return
#define ASCII_LF            10  // Line Feed

//////////////////////////////////////////////////////////////

      /// Script macros
#define OPEN_AND_APPEND         AT_ISP_PROVIDER, AT_ACTIVATE, AT_FTP_ACCOUNT, AT_BINARY_FILE, AT_FTP_APPEND
#define REPORT_NAME             DATE_TODAY, UNDERSCORE, SENSOR_ID, TXT_R_DASH, TODAYS_NUM, DOT_TXT
#define FILE_NAME               REPORT_NAME, END_OF_FILENAME
#define SENSOR_LOG_FILE_NAME    TXT_SENSOR_LOG, SENSOR_ID, DOT_TXT, END_OF_FILENAME
#define ACTIVITY_LOG_FILE_NAME  TXT_ACTIVITY_LOG, DATE_TODAY, DOT_TXT, END_OF_FILENAME

    /// SCRIPTS
    static const char report_script_noheading[] = {
            OPEN_AND_APPEND, FILE_NAME,     // Open ftp and append information into a file
            REPORT_NAME, CR,                // Start Report heading
            SENSOR_ID, CR,
            DATE_TODAY, CR,
            TODAYS_NUM, CR,                   // Report Heading (report version)
            COMM_ID, CR,                    // Report Heading (IDs and Installation date)
 //           NEST_LATITUDE, COMMA, NEST_LONGITUDE, CR,   // Report Heading (nest GPS location)
            START_TIME, CR,
            REPORT_TIME, CR,               // Report Heading (Timings: start, end, and interval)
            END_OF_SCRIPT
            };

    static const char report_script[] = {
            OPEN_AND_APPEND, FILE_NAME,         // Open ftp and append information into a file (included CR)
            TXT_REPORT, REPORT_NAME, CR,        // Start Report heading with file name
            TXT_SENSOR_ID, SENSOR_ID, CR,
            TXT_TODAYS_DATE, DATE_TODAY, CR,
            TXT_REPORT_NUM, TODAYS_NUM, CR,         // Report Heading (report version)
            TXT_COMM_ID, COMM_ID, CR,           // Report Heading (IDs and Installation date)
//            TXT_NEST_GPS, NEST_LATITUDE, COMMA, NEST_LONGITUDE, CR,     // Report Heading (nest GPS location)
            TXT_START_TIME, START_TIME, CR,
            TXT_REPORT_TIME, REPORT_TIME, CR,
            TXT_SECS_PER_REC,                   // Report Heading (Timings: start, end, and interval)
            END_OF_SCRIPT
            };

    static const char end_report[] = {
            CR, COPYLEFT, TXT_END,              // end of report text
            END_OF_SCRIPT
            };

    static const char read_parameters[] = {
            AT_FTP_ACCOUNT, AT_BINARY_FILE,     // Read in paramtetersXX####.ts
            AT_FTP_GET, PARAMETER_FILE,  SENSOR_ID, DOT_TS, CLOSE_QUOTE,
            END_OF_SCRIPT
            };

    static const char delete_parameters[] = {
            AT_FTP_ACCOUNT,                     // Delete paramtetersXX####.txt
            AT_FTP_DELETE, PARAMETER_FILE,  SENSOR_ID, DOT_TS, END_OF_FILENAME,
            AT_FTP_CLOSE,
            END_OF_SCRIPT
            };

    static const char start_log[] = {          // log in to server
            AT_ISP_PROVIDER, AT_ACTIVATE,
            END_OF_SCRIPT
            };

    static const char next_sensor_log[] = {     // Opens the next log file
            AT_FTP_ACCOUNT, AT_BINARY_FILE,
            AT_FTP_APPEND, SENSOR_LOG_FILE_NAME,
            END_OF_SCRIPT
            };

    static const char activity_log[] = {    // Open ftp and append information into an activity log
            AT_FTP_ACCOUNT, AT_BINARY_FILE,
            AT_FTP_APPEND, ACTIVITY_LOG_FILE_NAME,
            END_OF_SCRIPT
            };

    static const char log_line[] = {      // A line of log data (Date, time, Comm and SensorIDs and "Event: ")
            REPORT_TIME, COMMA_SPACE,
            TXT_COMM_ID, COMM_ID, COMMA_SPACE,
            TXT_SENSOR_ID, SENSOR_ID, COMMA_SPACE,
            TXT_EVENT,
            END_OF_SCRIPT
            };



    unsigned long int x;        //all purpose 4 byte
    unsigned int y;             //all purpose 2 byte
    unsigned int z;             //all purpose 2 byte

    static const char days_in_month[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
                                // used to calculate elapsed days
    static const unsigned int powers_of_10[] = { 1, 10, 100, 1000, 10000};
                                // used for coverting HEX to ASCII

/////////////////////////////////////////////////////////////////////////////////////
///////////////////////// DATA STORAGE //////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
    // DATA STORAGE DEFINITIONS
    // These are the usage for each byte in each record of the data.bins[][] array
    // see SENSOR DATA immediately below
    #define TEMPS       0
    #define X_POS       1
    #define Y_POS       2
    #define Z_POS       3
    #define READINGS    4
    #define HIGHEST_BIN 5
    #define BINS        6          // The first bin stored
    #define JOLT_BINS   25
    #define JOLT_BITS   32
    #define JOLT_MASK   0x80000000
    #define MAX_SENSORS     4      // perhaps one sensor's data can be uploaded while the other is transmitted out the phone.
    #define ODRS_WATCHED    6


    ///  COMMUNICATIONS DATA BUFFER
    ///  This buffer holds incoming data temporarily when it is moved by UART and SPI routines
    ///  The same buffer is used by the Slave and Master for communications and
    ///  the slave uses this buffer to store data from the ADXL sensor
    #define DATA_BUFFER_SIZE 0x0200         // this needs to be large enough to handle the largest command of
                                            // the most reads by the ADXL sensor in 1/8 of a second at the highest ODR
                                            // with temperature, that would be 400, so x0200 is more than enough

    union Incoming
    {   unsigned int spi[DATA_BUFFER_SIZE];     // ADXL data comes in ints
        unsigned char uart[DATA_BUFFER_SIZE/2]; // UART data comes in bytes
    }incoming;

    union Buffer
    {   unsigned int bins[JOLT_BITS];  // Temporary storage for ADXL data until a record is completed and compressed.
                                                // 28 bins will be created, but only MAX_BIN of them will be stored in bins[].
                                                // All bins lower than the MAX_BIN highest will be discarded.
                                                // The bins increase exponentially by a factor of the square root of two (about 3db amplitude per bin)
        unsigned char bytes[JOLT_BITS+JOLT_BITS]; //Temporary storage for UART data
    } buffer;

    //volatile unsigned int temp_bins[JOLT_BITS] ;
    volatile unsigned char starting_up = YES;       // "Yes" after a reset or power-up, "No" otherwise
    volatile unsigned char force_shut_down = NO;    // Set to "yes" after a bad battery reading
    volatile unsigned int adc_read;                 // The data read in the adc interrupt routine
    volatile signed int aXold=0;                    // previous acc value, to determine delta acc
    volatile signed int aYold=0;
    volatile signed int aZold=0;
    volatile unsigned char not_first_read = NO;     // reset everytime a new integration starts
    volatile unsigned long long int e_integrated;   // A running total
    volatile unsigned int e_sample_count;           // the number of samples added together so far
    volatile unsigned char e_sample_bits;           // The power of 2 for number of 12.5 Hz sampling,
        // +1 for 25 Hz, +2 for 50, etc... Minimum is 1, maximum is 11
        // e_sample_bits   12.5  25     50    100       200     400     Time
        //  1               1     2      4      8       16      32      ~.08 seconds
        //  2               2     4      8     16       32      64      ~.16 seconds
        //  3               4     8     16     32       64     128      ~.32 seconds
        //  4               8    16     32     64      128     256      ~.64 seconds
        //  5              16    32     64    128      256     512      ~1.3 seconds
        //  6              32    64    128    256      512    1024      ~2.5 seconds
        //  7              64   128    256    512     1024    2048      ~5.1 seconds
        //  8             128   256    512   1024     2048    4096       ~10 seconds (default)
        //  9             256   512   1024   2048     4096    8192       ~20 seconds
        // 10             512  1024   2048   4096     8182   16384       ~41 seconds
        // 11            1024  2048   4096   8182    16385   32768       ~82 seconds

    volatile unsigned int e_sample_limit;           // The number of samples we'll take (64K max)
    volatile unsigned int bin=0;         // bin counter
    volatile char slave_registered;       // the interval assigned by the master during registration

    union Records
    {    unsigned char bytes[MAX_RECORDS][(MAX_BIN+6)*2];    // data storage as bytes
         unsigned int bins[MAX_RECORDS][MAX_BIN+6];          // data storage as ints when used for bin storage
         unsigned char energy[MAX_SENSORS][MAX_RECORDS][12 + (ODRS_WATCHED * 2)]; // data storage as ints when used for energy storage
        // other formats are possible for other data arrangments, but the reports are expecting bytes.
    } data;


 //    unsigned char data.bytes[MAX_RECORDS][32];   (original code in PHASE TWO)

    //  (used by master for data storage to generate reports)
    //  [MAX_RECORDS][0-1]   -- Temperature reading (the average of 16 readings)
    //  [MAX_RECORDS][2-3]   -- X Position
    //  [MAX_RECORDS][4-5]   -- Y Position
    //  [MAX_RECORDS][6-7]   -- Z Position
    //  [MAX_RECORDS][8-9]   -- Total number of accumulated readings for each record
    //  [MAX_RECORDS][10-11] -- Highest Bin: the highest bin corresponds to bincount
    //                          (plus one because zero readings are possible) of the highest bit
    //                          triggered in any reading.  If there are high readings (>0x0A)
    //                          the lowest bins are dropped.  EG:  0x0A = the highest is bin 0x0A,
    //                          the lowest bin correspondsto the lowest readings;  0x10 = the highest is bin 0x10
    //                          the lowest 6 bins are dropped.
    //  [MAX_RECORDS][22-31] -- The ten highest bin readings.  Bytes 30 and 31 are the highest bin.

    //    unsigned char energy[MAX_SENSORS][MAX_RECORDS][14];  (after merging with slave for PHASE THREE)
    //  (used by master for data storage to generate reports)
    //  [MAX_RECORDS][0-1]   -- Average energy reading (ODR = 100 Hz) or selected rate
    //  [MAX_RECORDS][2-3]   -- Highest energy reading
    //  [MAX_RECORDS][4-5]   -- Average moisture reading
    //  [MAX_RECORDS][6-7]   -- Temperature reading (the average of 16 readings)
    //  [MAX_RECORDS][8-9]   -- X Position
    //  [MAX_RECORDS][10-11] -- Y Position
    //  [MAX_RECORDS][12-13] -- Z Position
    //  ----------------------------------- the following fields are optional ---------
    //  [MAX_RECORDS][14-15] -- Average energy reading (ODR = 400 Hz)
    //  [MAX_RECORDS][16-17] -- Average energy reading (ODR = 200 HZ)
    //  [MAX_RECORDS][18-19] -- Average energy reading (ODR = 50 Hz)
    //  [MAX_RECORDS][20-21] -- Average energy reading (ODR = 25 HZ)
    //  [MAX_RECORDS][22-23] -- Average energy reading (ODR = 12.5 HZ)

    // Depending upon how many ODR you want, you can restructure the memory using ODRS_WATCHED


// unsigned int data.bins[MAX_RECORDS][MAX_BIN + 6];
    // (used by sensors for running bins -- PHASE 2)
    // each record is 32 bytes with 10 bins
    // data.bins[][0] = data.bins[][TEMPS] = the temperature reading at the completion of each record
    // data.bins[][1] = data.bins[][X_POS] = one (first) reading per record of absolute orientation x, y and z
    // data.bins[][2] = data.bins[][Y_POS]
    // data.bins[][3] = data.bins[][Z_POS]
    // data.bins[][4] = data.bins[][READINGS] = the total number of accumulated readings for each record
    // data.bins[][5] = data.bins[][HIGHEST_BIN] = the highest bin that is not zero.  This corresponds to the highest reading in the record.
    //                                  This is also the highest in the record if the highest bin is bigger than MAX_BIN
    //                                  if so, than low bins are dropped.
    //                                  eg.  if this is 15 and MAX_BIN is 10, then bins below level 5 are dropped.
    // data.bins[][6] = data.bins[][BINS] = first bin count... and continuing to data.bins[][15] = last bin count(highest bin)



    // SENSOR COUNTERS AND FLAGS
    unsigned int recUploads = 0;                // The number of records that have been uploaded
    //unsigned int recCount = 0;                  // The count of how many records have been collected since last upload
    //unsigned long int secCount = 0;             // seconds since last data upload
    unsigned int recSecs = 0;                   // The number of seconds that the current record has been active
    unsigned char active = NO;                  // "Yes" if the turtles are starting to move,  "No" if they are not
    unsigned char first_start = YES;            // Only "Yes" the first time the sensor is used. Might not be needed
    unsigned char command;                      // The command in process
    //unsigned char error_code = NO_ERROR;                // see interrupt reporting routine for error codes

    // TEMPERATURE CALIBRATION SETTINGS
    // During Calibration, the reading at 0 degrees C becomes the temperature offset, and the ratio of 0x0200
    // divided by the reading at room temperature (minus the offset) becomes the temperature ratio.
    signed int temperature_offset = TEMPERATURE_OFFSET;     // Default setting does not change the reading
    unsigned int temperature_ratio = TEMPERATURE_RATIO;     // The temperature reading adjusted by the offset times this number will be divided by 0x1000,


    //unsigned int secMax = SLOWDAY_BIN_SEC;           // The number of seconds to accumulate readings in each record (set of bins)
    unsigned long int max_run_time = SECS_IN_DAY-60; // The maximum time the sensor can run without reporting back to comm head.
    unsigned int maxRecords = MAX_RECORDS;           // The maximum number of records to collect     (can be changed at some future point)
    unsigned const char copyleft[] = "TurtleSense Phase III -- CC 4.0 BY-SA NerdsWithoutBorders.net";


#pragma SET_DATA_SECTION()                  // end of FRAM data section

void watchdog_reset()       //reset the watchdog timer
{

}


// Wait a specified number of milliseconds (int)
// Actually almost a millisecond, the exact value is 1/1028 of a second
// note, seems to be 1/8 what it should be for unknown reason
void set_timeout(unsigned int wait_msecs)
{   /* Stops the timer */
    STOP_MSEC_TIMER;
     /* Sets the value of the capture-compare register */
    TIMER_A_setCompareValue(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0, wait_msecs);
    TA2R = 0;
   /* Starts timer counter */
    START_MSEC_TIMER;
    waiting = YES;
}


// Wait a specified number of milliseconds (int)
// Actually almost a millisecond, the exact value is 1/1028 of a second
void wait(unsigned int wait_millisecs)
{ unsigned char old_intervals_on;
    /* Stops the timer */
   STOP_MSEC_TIMER;                 // Sets the value of the capture-compare register
   TIMER_A_setCompareValue(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0, wait_millisecs);
                                    // sets timer counter compare value
   TA2R = 0;                        // reset timer register
   old_intervals_on = intervals_on; // remember how this was set for later
   intervals_on = NO;               // stop the 1/8 sec clock wake-ups
   START_MSEC_TIMER;                // start the timer
   waiting = YES;                   // set the flag so it turns off
   ENTER_LOW_POWER_MODE_3;          // sleep until the timer has counted up to the value just loaded
   STOP_MSEC_TIMER;
   intervals_on = old_intervals_on; // intervals_on restored to previous setting
}


void wait_a_sec(unsigned int wait_seconds)
{
   for (; wait_seconds; wait_seconds--) wait(1027);
   // this will take about a second.
}


// turn on red LED, wait specified amount of time, turn off LED
void blink (unsigned int millisecs)
{   if (not_slave)
    {   RED_LED_ON;                 // Red LED on
        wait(millisecs);
        RED_LED_OFF;
    }
}

// turn on red LED, wait specified amount of time, turn off LED
void blink_green (unsigned int millisecs)
{   if (not_slave)
    {   GREEN_LED_ON;                 // Red LED on
        wait(millisecs);            // wait
        GREEN_LED_OFF;                // Red LED off
    }
}


    // MOVE_STRING
    // moves characters from one string (origin) to another (destination), overwriting the destination
    // the first parameter is the pointer to the string that is the origin of the characters
    // the second parameter  is the offset into the offset into that string for the first byte to move
    // the third parameter is the number of characters to move
    // the fourth parameter is the pointer to the string that is the destination of the characters
    // the fifth parameter is the offset into that string where the first byte gets put
    // CAUTION!  There is no error checking in this routine.  BE CAREFUL!
void move_string(unsigned char *origin_ptr,   unsigned char origin_offset,  unsigned char bytes2move,
                                 unsigned char *destination_ptr,  unsigned char destination_offset)
{   unsigned char temp_count;       // counter
    for (temp_count=0; temp_count < destination_offset; temp_count++) destination_ptr++;
                            // adjust the destination pointer to its actual destination
    for (temp_count=0; bytes2move; temp_count++)    // count through all of them
    {   if (temp_count >= origin_offset)            //only move the data once we're at the offset
        {   bytes2move--;                           // decrement the number of bytes left to move
            *destination_ptr =  *origin_ptr;        // move a character from the origin string to the destination string
            destination_ptr++ ;                     // increment the destination pointer only after a move
        }
        origin_ptr++;                               // always increment the pointer to the origin
    }
}



// TRANSFER A STRING
// sends or receives a character array of known length using the UART
// 1st parameter is the pointer to the array, the second is the length of the array
// the third is whether sending or receiving 0 = RECEIVE, 1 = SEND
// two more bytes are received to indicate the starting offset in the array
// and how many bytes to transfer (255 maximum)
void transfer_string(unsigned char *string_ptr, unsigned char string_cnt, unsigned char sending)
{

    // TODO // replace this with long coax messages wherever it was used
    /*
    unsigned char temp_count, start_byte, bytes2move;   // counters
    while (COAX_RX_BUFF_EMPTY) ;             // wait for a full buffer
    start_byte = UCA0RXBUF;             // the next byte received is the starting byte
    for (temp_count = 0; temp_count < start_byte; temp_count++) string_ptr++;   // point to the starting byte
    while (COAX_RX_BUFF_EMPTY) ;             // wait for a full buffer
    bytes2move = UCA0RXBUF;             // the next byte received is how may bytes will move
    if (sending)
    {   while (bytes2move--)            // count through all of them
        {   while (COAX_TX_NOT_READY);       //wait for an empty buffer
        UCA0TXBUF= *string_ptr;     // send a byte
        string_ptr++;               // increment the pointer
        }
        return;
    }
    for (; bytes2move; bytes2move--)    // count through all of them
    {   while (COAX_RX_BUFF_EMPTY) ;         // wait for a full buffer
        *string_ptr = UCA0RXBUF;        // get a byte
        string_ptr++;                   // increment the pointer
    }
    */
}


// coax_reset
void coax_reset(void)
{   while(COAX_BUSY);               // let things finish
    COAX_UART_RESET;                // Put eUSCI in reset
    __no_operation();
    COAX_UART_ENABLE;               // release from reset
    COAX_INTERRUPT_DISABLE;
}

// coax_initialize
        // Get everything ready to transmit
        // resets the UART and turns on the 1MHz carrier
void coax_initialize(void)
{  coax_reset();                     // Put eUSCI in reset
   SMCLK_ON;                       // turn on the 1MHz carrier
}


// transmit_done
        // This routine ends the transmission process.
void transmit_done(void)
{   coax_reset();
    SMCLK_OFF;                              // Turn off the 1MHz carrier used for transmitting
}


//coax_string
                // sends a string pointed to by string_ptr
                // the number of chars send is string_len
                // out over the Master UART
unsigned char coax_string(unsigned char *string_ptr, unsigned int string_len)
{   if (string_len)
    {   coax_initialize();                      // get things ready
        coax_tx_buff_ptr = string_ptr;          // point to the char array with the command
        while (string_len--)                    // while we haven't sent the entire string
        {   COAX_TX_BUFF = *coax_tx_buff_ptr;   // get a byte of the string in the TX buffer,
            coax_tx_buff_ptr++;                 // point to the next byte
            while (COAX_TX_NOT_READY);          // wait until the char has been sent
        }
    }
    return(0);                                  // return error code (no error checking yet)
}


// coax_char
    // sends a single character out the coax line
unsigned char coax_char(unsigned char byte2send)
{   coax_initialize();                  // get things ready
    coax_tx_buff_len = 0;               // no other chars to transmit
    COAX_TX_BUFF = byte2send;           // get a byte ready
    while(COAX_TX_NOT_READY);           // wait until the char has been sent
    return(0);                          // return error code (no error checking yet)
}


////// coax_short_message (short messages always have the lowest bit of command bank equal to zero)
    // sends a message to a connected device using the UART
    // Passed parameters are the bank and number of the recipient and the bank and number of the command
char coax_short_message(unsigned char command_number)
{   unsigned int temp_checksum = 0x00;     // used to calculate the checksum of the message
    unsigned char *q = (unsigned char*)&temp_checksum;
    unsigned char short_command[] = {PROTOCOL_BYTE,      // Starts off with an 0xFF, but this could change for other protocols
                                    going_to_bank,
                                    going_to_unit,   // two bytes to identify who we are sending a message to -- Bank and number
                                    this_bank,
                                    this_unit,      // the ID of this device -- who is sending the message
                                    current_command_bank,
                                    command_number,     // two bytes for the command -- bank and number.  Banks equal to zero or even are short commands
                                    0x00, 0x00,         // two bytes for a check sum
                                    0x00 };             // a final zero indicates that this isn't a long message
    for (message_count = 0; message_count<7; message_count++)
        temp_checksum += short_command[message_count];
                                        // calculate the checksum by adding all the previous bytes
    short_command[7] = q[1];            // store the first byte of the checksum
    short_command[8] = q[0];            // and the second
    tx_error = coax_char(0x00);                        // wake things up by sending a zero
    tx_error |= coax_string(short_command, 10);        // no error checking for now
    transmit_done();
    return(tx_error == NO_ERROR);          //returns YES if success.
}


////// coax_long_message(lowest bit of command bank equals zero)
    // sends a message to a connected device using the UART
    // A header is always appended to the message
    // 1st parameter is the command number (always even),
    // the second and third are optional parameters
    // the fourth is  a pointer to the data string
    // the fifth is the length of the data string
char coax_long_message( unsigned char command_number,
                        unsigned char command_param1,
                        unsigned char command_param2,
                        unsigned char *data_pointer,
                        unsigned int data_length)
{   unsigned long temp_checksum = 0x00;     // used to calculate the checksum of the message
    unsigned char *p = (unsigned char*)&data_length;
    unsigned char *temp_pointer;
    unsigned char long_command[] = {PROTOCOL_BYTE,      // Starts off with an 0xFF, but this could change for other protocols
                                    going_to_bank,
                                    going_to_unit,    // two bytes to identify who we are sending a message to -- Bank and number
                                    this_bank,
                                    this_unit,           // the ID of this device -- who is sending the message
                                    current_command_bank,
                                    command_number,     // two bytes for the command -- bank and number.  Banks equal to zero or even are short commands
                                    command_param1,     // two exta bytes for a sub command, switch or parameter instead of the checksum
                                    command_param2,
                                    0x01,               // This byte marks the beginning of the extension of data for long messages
                                                        // Short messages would have a zero here marking the end of the message
                                    p[1], p[0],         // two bytes for the data length (high, low)
                                    0x00, 0x00,
                                    0x00, 0x00,         // space for four more bytes for the checksum of a data command
                                    //the data pointed at by temp_pointer comes next
                                    0x00 };              // a final zero

    for (message_count = 0; message_count<12; message_count++)
        temp_checksum += long_command[message_count];
        // calculate the checksum by adding all the bytes before the checksum
    temp_pointer = data_pointer;
    for (message_count = 0; message_count < data_length; message_count++)    // also add all data
    {   temp_checksum += *temp_pointer;                     // use the pointer to find the data
        temp_pointer++;                                     // increment the point to the next byte
    }
    for (message_count = 15; message_count > 11; message_count--)        // go through the 6 bytes of the checksum
    {   long_command[message_count] = (temp_checksum & 0x000000FF);    // store the bytes in the header
        temp_checksum>>=8;
    }   // the long int is stored {highest byte, 2nd highest, 2nd lowest, lowest byte}
        // send the header
    tx_error = coax_char(0x00);                    // wake things up by sending a zero;
    tx_error |= coax_string(long_command, 16);     // no error checking yet
        // send the data
    tx_error|= coax_string(data_pointer, data_length);    // each error should be set by a bit so they can be combined
        // send a zero to mark the end
    tx_error |= coax_char(0x00);                   // end with a zeror
    transmit_done();
    return(tx_error == NO_ERROR);                        // all good (for now)
}




////// receive_message()
    // receives a message over the coax line
    // this routine was written using polling instead of interrupts to be faster and more reliable.
unsigned char receive_message(void)
{   unsigned char *q = (unsigned char*)&received_checksum;
    unsigned char *p = (unsigned char*)&received_datalength;
    unsigned char last_byte;
    coax_reset();                    // Reset the UART
    COAX_RX_ENABLE;                  // Enable RX interrupts
    if (timeout_during_rx)           // Don't timeout during synch receives
        set_timeout(RX_TIMEOUT);     // Set the clock for the total time to wait for a message once synched.
              // TODO // long messages will need a longer timeout
                      // a new timeout could be set once we know how much data is coming
    else                             // Set the timeout value for synching
        set_timeout (SYNCH_TIMEOUT); // This is more than 6 seconds.
    waiting_for_coax_data = YES;     // This flag will wake us up when a byte is received.
    ENTER_LOW_POWER_MODE_3;          // or we'll sleep until the timer runs out.
                                     // If we are still waiting we've received something,
    waiting_for_coax_data = NO;      //  nothing else should wake us up if we've timed out.
    COAX_INTERRUPT_DISABLE;          // Just the wake up byte gets an interrupt, the rest is polled.
        // The first byte will be a zero.  It just wakes up the device.
        // The first byte is not included in the checksum because it is expected to contain garbage data
    while (COAX_RX_BUFF_EMPTY && waiting);                      // 2nd byte is the Protocol byte (0xFF)
    received_protocol = COAX_RX_BUFF;
    while (COAX_RX_BUFF_EMPTY && waiting);                      // 3rd byte is the id bank being called
    received_recipient_bank = COAX_RX_BUFF;
    running_checksum = received_protocol + received_recipient_bank;// start out the checksum with the 2nd and 3rd byte
    while (COAX_RX_BUFF_EMPTY && waiting);                      // 4th byte is the id number being called
    received_recipient_unit = COAX_RX_BUFF;
    running_checksum += received_recipient_unit;                // keep the checksum current
    while (COAX_RX_BUFF_EMPTY  && waiting);                     // 5th byte is the caller's bank
    received_caller_bank = COAX_RX_BUFF;                        // the fifth byte indicates the first byte of the caller's ID
    running_checksum += received_caller_bank;
    while (COAX_RX_BUFF_EMPTY && waiting);                      // 6th byte is the caller's id number
    received_caller_unit = COAX_RX_BUFF;                        // the sixth byte indicates the second byte of the caller's ID
    running_checksum += received_caller_unit;
    while (COAX_RX_BUFF_EMPTY && waiting);                      // 7th byte is the bank of commands
    received_command_bank = COAX_RX_BUFF;                       // the seventh byte indicates the bank of commands (0x00 are short commands without data)
    running_checksum += received_command_bank;
    while (COAX_RX_BUFF_EMPTY && waiting);                      // 8th byte is the command number
    received_command = COAX_RX_BUFF;                            // the eighth byte indicates the command number in that bank
    running_checksum += received_command;
    while (COAX_RX_BUFF_EMPTY  && waiting);                     // 9th byte is parameter 1, or the first checksum byte
    received_checksum = 0x00;                                   // zero this out before receiving a new number
    q[1] = parameter_one = COAX_RX_BUFF;                        // parameters are for long commands, checksum for the short commands
    while (COAX_RX_BUFF_EMPTY && waiting);                      // 10th byte is parameter 2, or the second checksum byte
    q[0] = parameter_two = COAX_RX_BUFF;                        // parameters are for long commands, checksum for the short commands
    while (COAX_RX_BUFF_EMPTY && waiting);                      // 11th byte is a zero for short commands and a one for long commands
    last_byte =received_command_type = COAX_RX_BUFF;            // store as the last byte and the command type
    if (received_command_type && waiting)                       // long commands have data attached
    {                                                           // find out how much data will be sent
        received_datalength = 0x00;                             // Clear the data length of the data to be received
        for (bytecount = 2; bytecount; bytecount--)             // only expecting 2 bytes (high, low)
        {   while (COAX_RX_BUFF_EMPTY && waiting);              // receive datalength bytes
            p[bytecount-1] = coax_char_received = COAX_RX_BUFF; // assemble the 2 bytes into the data length for data commands
            running_checksum += coax_char_received;             // keep the checksum current
        }       // done receiving datalength, start receiving checksum
                // the long int was stored {highest byte, 2nd highest, 2nd lowest, lowest byte}
        for (bytecount = 4; bytecount; bytecount--)             // only expecting 4 bytes
        {   while (COAX_RX_BUFF_EMPTY && waiting);              // these 4 bytes are the checksum
            q[bytecount-1] = COAX_RX_BUFF;                      // assemble the bytes into the checksum for the message
        }       // done receiving checksum, start receiving data
        if (received_datalength)
            for (bytecount = 0; bytecount<received_datalength; bytecount++)
            {   while (COAX_RX_BUFF_EMPTY && waiting);             // receive data
               incoming.uart[bytecount] = coax_char_received = COAX_RX_BUFF;       // store data in the buffer.
               running_checksum += coax_char_received;              // keep the checksum current
            }       // done receiving checksum, receive the final zero
        while (COAX_RX_BUFF_EMPTY && waiting);                 // get the last byte
        last_byte = COAX_RX_BUFF;                               // The last char should be a zero this time if it wasn't the last time...
        running_checksum += (parameter_one + parameter_two + received_command_type);     // these optional parameters are included in the checksum
    }
    TIMER_A_stop(TIMER_A2_BASE);                // Stop timeout clock since we are done
    tx_error = NO_ERROR;                        // default is everything is good
    if ( ( (received_recipient_unit != this_unit) && (received_recipient_unit != ALL_UNITS) )
        || ( (received_recipient_bank != this_bank) && (received_recipient_bank != ALL_BANKS) ) )
        tx_error = ADDRESS_ERROR;
        // This unit is not being called if it isn't this unit's ID or if all units are not being called
        // the third byte must be the first ID byte of this hardware in this bank, or 0xFF for all banks
    if (received_checksum != running_checksum ) tx_error = CHECKSUM_ERROR;
                // if the checksum doesn't match...
    if ( (received_protocol != PROTOCOL_BYTE) || (last_byte) ) tx_error = PROTOCOL_ERROR;
                //protocol errors negate data errors
    if (waiting == NO) tx_error = TIMEOUT_ERROR;  // timeout errors override all others
    waiting = NO;                           // reset this flag
    return (tx_error == NO_ERROR);          // returns true if all good, false if there's an error
}



////////////////////////////////////////////////////////////////////////////////
//================ Janus Phone board routines ================================//
////////////////////////////////////////////////////////////////////////////////

void reset_phone_UART(void)
{
    PHONE_UART_RESET;            // Reset the UART for the phone
    __no_operation();
    /* Port 2 Output Register */
    // Pin 2.4 only goes low to turn on the phone, so its default is high
    PHONE_ON_OFF_HIZ;           // sets the pin high
    /* Port 2 Port Select Register 1 */
    // Pins 2.5 and 2.6 used for Phone UART
    P2OUT &= ~ BIT5;            // clear the output pin (might not be needed)
    P2DIR &= ~ BIT5;            // clear this pin as use as an output (also may not be needed)
    P2SEL1 |= ( BIT5 | BIT6);   // designate these pins as the UART
    PHONE_UART_ENABLE;          // turn on the UART
    PHONE_INTERRUPT_DISABLE;    // no interrupts
}

void phone_UART_off(void)
{   PHONE_UART_RESET;
    // Pins 2.5 and 2.6 used for Phone UART
    P2SEL1 &= ~( BIT5 | BIT6 ); // remove the UART designation for these pins
    P2DIR |= BIT5;              // set this pin as an output
    P2OUT &= BIT5;              // P 2.5 will be an output low
    PHONE_UART_ENABLE;
}


        // Sends an AT code or string to the phone
void sendMessage(unsigned char message_code)    // the array number of the pointers is passed
{   unsigned char AT_tx_char;                   // The character we will send out
    unsigned int buffer_count;                  // a counter while scanning the buffer
    unsigned char cr_count;                     // counting carriage returns
    unsigned char *string2send;                 // we'll put the pointer here
    coax_quiet = NO;                            // prevent the timer interrupts from interfeering
    while(PHONE_TX_NOT_READY);                   // wait for previous byte to complete
    reset_phone_UART();
    not_timed_out = YES;        //TODO// make timeouts work!
    buffer_count = 0;
    string2send = messages[message_code];       // get the pointer out of the array
    cr_count = (message_code < CR) + (message_code==AT_CREG) + (message_code == AT_GET_TIME);
        // cr_count is set to the number of carriage returns expected in the message returned by the phone
        // equals 1 if less than 22, 0 if greater, 2 if AT_CREG or AT_GET_TIME
    if (message_code==AT_NO_ECHO) cr_count = 4; // Wait for extra message of "+PACSP0"
    while (AT_tx_char = *string2send)           // get the next character and also continue looping if it is non-zero
    {   string2send++;                          // increment the pointer so it points to the next char
        PHONE_TX_BUFF = AT_tx_char;             // Send the byte
        while(PHONE_TX_NOT_READY);              // Wait until the buffer is empty
    }
    while (cr_count)
    {   while (PHONE_RX_BUFF_EMPTY);     // wait for data
        if ((incoming.uart[buffer_count++] = PHONE_RX_BUFF) == ASCII_CR) cr_count-- ;
    }
    if ((message_code==AT_CREG) && (buffer_count > 9)) connection_code = (incoming.uart[9]-0x30) ;    // return CREG registration code
    coax_quiet = YES;
    if (AT_tx_char == ASCII_CR)     // At the end of all lines
    {   if (message_code < CR)         // At the end of all all AT command lines
        {   if (buffer_count) error_code = (incoming.uart[0]-0x30); //Error codes from AT commands 0= no error
            wait(980);                      //wait a second between AT commands
        }
    blink(1);                               // blink as we process messages
    }
}


        // process a script for the phone
void do_script(const char *script)  //pass the pointer to the script array
{   unsigned char code;             // the command codes in the script
    const char *script_ptr;         // This pointer we can change
    script_ptr = script;            // set it to point to the script
    reset_phone_UART();             // clear out the buffers before starting
    while (code = *script_ptr)      // get the code pointed to in the script array
    {   sendMessage(code);          // send the message with that code
        script_ptr++;               // point to the next character in the script
        if ((code == END_OF_FILENAME) && (error_code == 4)) script_ptr = script; // reset the pointer and restart the script if there is a problem opening files
    }
}



        // get an internet connection with the phone board
void makeConnection(void)
{
    sendMessage(AT_NO_ECHO);        // Send the AT command to turn off echo and verbose responses
                // TODO // Modify so that this will work with all Janus boards automatically.
                // The AT_NO_ECHO command waits for four CRs for use with an HE910.
                // There is a message "+PACSP0" when it connects to the network.
    sendMessage(AT_GPS_OFF);        // Turn off GPS if comm tower, turn on if hand-held unit
    sendMessage(AT_SET_RTC);        // Set phones RTC from network time automatically
    connection_code = CELL_NOT_REGISTERED;       // reset connection code
    sendMessage(AT_CREG);           // Send AT code to request a registration report
    while ( (connection_code != CELL_REGISTERED) && (connection_code !=ROAMING) )   // added roaming
        // Keep trying until there is a connection
    {   coax_quiet = YES;          // this turns on sleep timer interrupts we can wake up
        wait_a_sec(1);                   // wait a second before trying again
        reset_phone_UART();         // reset the UART if that is the problem
        coax_quiet = NO;           // turn off sleep timer interrupts
        sendMessage(AT_CREG);       // Send AT code to request a registration report
    }                                       // TODO // better error handling!
}


    // convert an integer to an ascii string
void int2ascii(unsigned int number, unsigned char *char_string, unsigned char offset, unsigned char string_length) // string_length can be 1, 2, 3 or 4
{   unsigned char test_chars[4];                        // an easier to reference array to create the result
    unsigned char power_count, string_count;            // keep track of powers of ten
    power_count = string_length - 1;                    // start with the highest power of ten
    if (number >= powers_of_10[string_length]) return;  // a 4 character int must be less than 10 to the 4th power.
    for (string_count = 0; string_count < string_length; string_count++ )   // start with the leftmost character
    {   test_chars[string_count] = 0x30;
        while(number >= powers_of_10[power_count])
        {   test_chars[string_count]++ ;
            number -= powers_of_10[power_count];        //start the char at hex30, increment it for every
        }                                               // power of ten subtracted from the number
        power_count--;                                  // do this for all the digits
    }
    move_string(test_chars, 0, string_length, char_string, offset);  // move temporary string back to substring specified;
}


    // convert an ascii string to an integer
unsigned int ascii2int(unsigned char *char_string, unsigned char offset, unsigned char string_length) // string_length can be 1, 2, 3 or 4
{   unsigned char test_chars[5];        // an easier to reference array to work with
    unsigned char result = 0;           // accumulate the result here
    unsigned char string_count;         // go character by character
    unsigned char power_count = 0;      // keep track of powers of ten
    move_string(char_string, offset, string_length, test_chars, 1);  // move substring to temporary location
    for (; string_length; string_length-- )
    {   for (string_count = test_chars[string_length]- 0x30; string_count; string_count--)
                result += powers_of_10[power_count];
        power_count++;
    }
    return(result);
}


    // set the RTC from the cell tower
    // TODO // Save day and report count on sensor so that the numbers continue if the comm tower is swapped
void set_time(void)
{   //unsigned int start_day, current_day;
    //unsigned int start_month, current_month;
    move_string(messages[REPORT_TIME], 0, 19, messages[START_TIME], 0);  // move old report time to start time
    reset_phone_UART();                                         // empty the buffers
    sendMessage(AT_GET_TIME);                                   // request network time
    move_string(incoming.uart, 8, 17, messages[REPORT_TIME], 2);// move the date and time
    move_string(incoming.uart, 8, 2, messages[DATE_TODAY], 2);// move the year
    move_string(incoming.uart, 11, 2, messages[DATE_TODAY], 5);// move the month
    move_string(incoming.uart, 14, 2, messages[DATE_TODAY], 8);// move the day
    /*
    start_month =  ascii2int(messages[INSTALLED], 5, 2);        // convert 2 ASCII month chars to a single byte
    start_day =  ascii2int(messages[INSTALLED], 8, 2);          // convert 2 ASCII day chars to a single byte
    current_month = ascii2int(messages[REPORT_TIME], 5, 2);     // convert 2 ASCII month chars to a single byte
    current_day = ascii2int(messages[REPORT_TIME], 8, 2);       // convert 2 ASCII day chars to a single byte

    if (start_month == current_month) day_count = current_day-start_day+1;  // the case of the same month
    else
    {   day_count = days_in_month[start_month] - start_day + 1;
        start_month++;
        while(start_month<current_month)
        {   day_count += days_in_month[start_month];
            start_month++;
        }
        day_count+= current_day;
    }
    int2ascii(day_count, messages[DAYS_NUM], 0, 3);           //put the count into the string
    */
}


// download a new set of parameters from the web if they are available for this sensor
void get_new_parameters(void)
{   unsigned int tries, checksum;                   // use a checksum to verify a good load
    unsigned char *message_ptr;                     // pointer to sensor ID
    unsigned char datacount;                        // counter for file data
    unsigned char bad_checksum = TRUE;              // flag for checksum results
    for (tries = 1; bad_checksum && (tries<3) ; tries++) // try 2 times for a good load
    {   wait(980);  // pause between tries
        message_ptr = messages[SENSOR_ID];          // point to the sensor ID
        do_script(read_parameters);                 // download the file
        while(PHONE_RX_BUFF_EMPTY);                 // wait for a byte
        incoming.uart[0] = PHONE_RX_BUFF;           // store the byte received in a buffer
        if (incoming.uart[0] != 0x31) continue;     //first byte should be a "1"
        for(datacount=1; datacount < 0x33; datacount++) // read in the file
        {   while(PHONE_RX_BUFF_EMPTY);             // wait for a byte
            incoming.uart[datacount] = PHONE_RX_BUFF;   // store the byte received in a buffer
        }
        checksum=0;                                 // start the checksum at zero
        for(datacount=2; datacount < 0x30; datacount++) // verify the file
        {   if (datacount < 0x08) if (incoming.uart[datacount] != *message_ptr++) continue; // verify the sensor ID
            checksum += incoming.uart[datacount];   // add to the checksum
        }
        if (checksum == ((incoming.uart[0x30]<<8) + incoming.uart[0x31])) bad_checksum = FALSE;
                                                    // confirm the check-sum
    }                                               // try again if there is an error.  Give up after 2 tries.
    wait(980);                                      // pause before delete
    if (!bad_checksum)                              // double negative if a good file load
    {   move_string(incoming.uart, 8, PARAM_BYTES, new_parameters, 0);  // save the downloaded parameters
        do_script(delete_parameters);               // delete file if successful
        new_parameters_loaded = YES;                //set flag for report message
    }
    sendMessage(AT_FTP_CLOSE);        // close FTP connection
}


// decide if reports should be daily or more frequent and adjust timings
void set_report_frequency(void)
{   // unsigned char byteCount;                 // counter for counting the 4 bytes of MAXRUN
    // calculate the new maxTime
    //  maxTime = parameters[21];                   // maxTime starts with the highest byte of MAXRUN
    //  for (byteCount = 3; byteCount; byteCount--) // set maxTime to catch sensor errors
    //  {   maxTime <<= 8;                          // Shift the bits up to get ready for the next byte
    //      maxTime += parameters[17 + byteCount];  // combine MAXRUN1L, MAXRUN2, MAXRUN3 and MAXRUN4H (parameters[18] to [21])
    //  }
    // adjust the new parameters and send them out
    NEW_SLOWBIN_LO_NOW = NEW_SLOWBIN_LO;    // reset parameters[16] and [17] to the slow day settings
    NEW_SLOWBIN_HI_NOW =NEW_SLOWBIN_HI;
    if (day_count >= NEW_SLOW_DAYS)        // first approximation, wait 40 days and then go fast
    {   NEW_SLOWBIN_LO_NOW = NEW_BINSEC_LO;    // reset parameters[16] and [17] to the slow day settings
        NEW_SLOWBIN_HI_NOW = NEW_BINSEC_HI;
    }
    //SS_command(NEW_PARAMETERS);     // send the parameters to the sensor
    wait(1);                        // don't get ahead of the sensor

    // the following line needs fixing
//    transfer_string(new_parameters, PARAM_BYTES, 0, PARAM_BYTES, SEND); // send all the parameters
    //contact_sensor();               // wait for the sensor to respond and turn on RS485
}


// send out a nibble as an ASCII hexadecimal number from 0 to F
void send_nibble(unsigned char nibble)      // pass the nibble and we'll convert it
{   if (nibble > 0x09) nibble += 7;         // add 7 if the nibble is ten or more
    while(PHONE_TX_NOT_READY);             // wait for the UART to finish
    PHONE_TX_BUFF = (nibble + 0x30);        // Send the byte converted to ASCII
}


// send out a byte in two nibbles
void send_byte(unsigned char data_byte)     // pass the byte to send
{   send_nibble(data_byte>>4);              // send high nibble of byte
    send_nibble(data_byte & 0x0F);          // send low nibble of byte
}


// sends out an integer in four nibbles
void send_integer(unsigned int data_integer)
{
    send_byte((data_integer & 0xFF00)>>8);  // print the high byte of recCount
    send_byte(data_integer & 0x00FF);       // print the low byte of recCount
}

void escape_and_close(void)
{   wait(2000);                                 // add 2 seconds before and after escape routine
    sendMessage(AT_FTP_ESCAPE);                 // send the escape message
    wait(2000);                                 //  2 seconds after escape routine
    sendMessage(AT_FTP_CLOSE);                  // close ftp connection
}


// send out records in the report
void report_record_data(void)
{   unsigned int report_count;
    unsigned char char_count;
    not_timed_out = YES;        //TODO// make timeouts work!
    for (report_count = 0; report_count < recCount; report_count++)
    {   if (PARAM_REPORT_HEADINGS)          // if report headings are desired
        {   send_integer(report_count);     // send a count
            sendMessage(COMMA);             // send a comma
        }
        coax_quiet = NO;
        reset_phone_UART();
//        PHONE_TX_ENABLE;
        for (char_count = 0; char_count < 32; char_count +=2)   // go through them two by two
        {   send_byte(data.bytes[report_count][char_count]);       // send first byte of data
            send_byte(data.bytes[report_count][char_count+1]);     // send second byte of data
            if (char_count < 30 ) sendMessage(COMMA);           // send a comma except at the end of the line
        }
        sendMessage(CR);            // send a carriage return
    }
    wait(2);                        // make certain there is enough time to finish transmission.
}


void send_report(void)
{   if (PARAM_REPORT_HEADINGS)              // If the parameter is set to print report headings
    {   do_script(report_script);           // send script commands to open FTP and create a file
                                            // also creates most of the file header
        send_byte(PARAM_SLOWBIN_HI_NOW);    // High byte of seconds per record
        send_byte(PARAM_SLOWBIN_LO_NOW);    // Low byte of seconds per record
        sendMessage(TXT_NUM_RECS);          // # of records
        send_integer(recCount);             // print the recCount
        sendMessage(TXT_BATTERY_LEVEL);     // battery level
        send_integer(battery_level );       // print the battery level
        sendMessage(CR);                    // send a carriage return
        sendMessage(CR);                    // send a carriage return
        sendMessage(TXT_HEADINGS);          // headings for record data
    }
    else                                    // Otherwise print the report without headings
    {   do_script(report_script_noheading); // which uses a different script
        send_byte(PARAM_SLOWBIN_HI_NOW);    // High byte of seconds per record
        send_byte(PARAM_SLOWBIN_LO_NOW);    // Low byte of seconds per record
        sendMessage(CR);                    // send a carriage return
        send_integer(recCount);             // print the recCount
        sendMessage(CR);                    // send a carriage return
        send_integer(battery_level );       // print the battery level
        sendMessage(CR);                    // send a carriage return
        sendMessage(CR);                    // send a carriage return
        sendMessage(CR);                    // send a carriage return
    }
    report_record_data();                   // send all the data
                                                                // conditional lines sent at the end
    if (new_parameters_loaded) sendMessage(TXT_NEW_SETTINGS);   // send new parameters loaded message
    if (force_shut_down) sendMessage(TXT_LOW_BATTERY);          // send shutdown warning
    if (wd_reset) sendMessage(TXT_RESET);                       // send reset message
    if (starting_up) sendMessage(TXT_MONITORING);               // send startup message
    if (!sensor_plugged_in) sendMessage(TXT_DISCONNECTED);     // send sensor disconnect message
    if (PARAM_REPORT_HEADINGS) do_script(end_report);           // closing messages
                                                // finish up
    new_parameters_loaded = NO;                 // clear the new parameters flag
    wd_reset = NO;                              // clear the reset flag
    starting_up = NO;                           // clear the starting up flag
    escape_and_close();
}

///////send_log_line
void send_log_line(void)
{   do_script(log_line);            // A line of log data (Date, time, Comm and SensorIDs and "Event: ")
    switch (sensor_status[slot_count])    // end the line depending upon the current status
    {   case RECENTLY_CONNECTED:                // if we just connected
            sendMessage(TXT_MONITORING);        // send startup message
            break;
        case RECENTLY_DISCONNECTED:             // if we just disconnected
            sendMessage(TXT_DISCONNECTED);      // send sensor disconnect message
            break;
        case NEW_PARAMETERS_FOUND:              // if new parameters were just loaded
            sendMessage(TXT_NEW_SETTINGS);      // send the parameters message
            break;
    }
    escape_and_close();           // this closes the current log file
}

void report_event(void)
{   do_script(next_sensor_log);         // send script commands to open FTP and create a file
    send_log_line();
    do_script(activity_log);
    send_log_line();
}

////////////////send_census()
// sends out a list of new devices connected or disconnected
// sensor_status[SENSOR_SLOTS];     // 0 = no unreported change, 1 = recently connected, 2 = recently disconnected
// sensor_ids[SENSOR_SLOTS][9];     //sensor serial numbers in each slot (or most recent ID)
// start_log[]          // log in to server
// next_sensor_log[]    // Opens the next sensor log file
// close_log[]          // this closes the current log file
// activity_log[]       // Open ftp and append information into today's activity log

void send_census(void)
{   for (slot_count = 2; slot_count < SENSOR_SLOTS; slot_count++)
    {   if (sensor_status[slot_count] > NO_UNREPORTED_CHANGE)  // If non-zero the sensor was either recently connected or disconnected
        {   move_string(sensor_ids[slot_count], 0, 9, messages[SENSOR_ID], 0);// move the sensor_id
            report_event();                 // update the logs
            get_new_parameters();           // upload a new set of paramters if they are available
            if (new_parameters_loaded)
            {   sensor_status[slot_count] = NEW_PARAMETERS_FOUND;
                report_event();   // record the success of newly loaded parameters
                //TODO// send the new parameters to the sensor
            }
            sensor_status[slot_count] = NO_UNREPORTED_CHANGE;
        }
    }
   watchdog_reset();               // reset the watchdog timer
}

unsigned char read_battery()
{   CHECK_BATTERY_ON;               // turn on the battery voltage circuit
    // read the battery and return a value from 0 to 100
    // 0 = 3.0 volts, which is the minimum acceptable battery voltage
    // 100 = 4.2 volts, which is the maximum voltage of charged batteries
    // This number should be roughly proportional to percentage of battery capacity
    CHECK_BATTERY_OFF;
    return(START_UP_LEVEL+2);      // until the routine is written, make it above the minimum
}


void phone_on(void)
{       // Prepare to turn on the phone
    PHONE_RESET_HIZ;                // Set phone reset bit high  to set the phone pins to hi-z
    PHONE_ON_OFF_HIZ;               // Set on/off bit high to set the phone pins to hi-z
    PHONE_POWER_ENABLE;             // Turn on the phone power
    wait(20);                       // wait for 20 milliseconds for the 5V power supply to power up
    // then make sure the phone is off, but the power supply is on
    while (PHONE_IS_ON)             // check if the power monitor pin is high
    {   PHONE_POWER_DISABLE;            // while it is, turn off the phone board power supply
        while (PHONE_IS_ON) wait(8);    // loop until the power monitor pin is low.  Pause a millisecond before checking again
        PHONE_POWER_ENABLE;             // Turn on the phone power
        wait(20);                       // wait for 20 milliseconds for the capacitor to charge up
    }
    // actually turn on the phone
    while (PHONE_IS_OFF)            // While the power monitor indicator bit is off
    {   PHONE_ON_OFF_REQUEST;       // switch the phone power on/off by driving the pin low by setting P3.5 low
        //force_shut_down = (readBattery() < SHUT_OFF_LEVEL) ;    // if the battery is low, set a flag to shut down after uploading data
        wait_a_sec(3);              // with the battery reads this will be over 3 seconds
        PHONE_ON_OFF_HIZ;           // return the pin to hi-z by setting 3.5 high
        wait_a_sec(4);
    }
}



// turn off the phone
void phone_off(void)
{
    PHONE_RESET_HIZ;            // Make certain that the reset bit is by making it hi-z
    wait(8);                    // wait for the chips to settle?
    while (PHONE_IS_ON)         // keep doing this while the phone power monitor stays high
    {   PHONE_ON_OFF_REQUEST;   // switch the phone power on/off by driving the pin low
        wait_a_sec(3);          // wait at least 3 seconds
        PHONE_ON_OFF_HIZ;       // return the pin to hi-z by setting pin high
        wait_a_sec(3);          // wait at least 3 seconds
    }
    PHONE_POWER_DISABLE;        // Turn off the phone power
    phone_UART_off();           // disable the UART
}


// POWER DOWN
// enter sleep mode 3 if the batteries are low
// TODO // Rewrite to wake up occasionally and check the batteries.
void power_down()
{   COAX_POWER_DISABLE;       // Power down the sensors
    // TODO // this may ultimately be improved by sending an instruction to the microprocessor to go to sleep.
    WDTCTL = ((WDTCTL &  0x00FF) | 0x5A88);     // reset the watchdog timer on hold
    //  await_connection();             // wait in low power mode until the smart sensor is plugged in.
    //  contact_sensor();               // wait for the sensor to respond and turn on RS485
    //  SS_command(MSP_SLEEP);          // send SS the command to sleep
    // this causes strange power consumption problems
//    while (TRUE) ENTER_LOW_POWER_MODE_3;    // Enter low power mode 3
}

////// report_in
// send in a status report
void report_in(void)
{   makeConnection();               // wait for the phone to establish an internet connection
    wait_a_sec(1);
    set_time();                     // reads RTC from the phone and saves it in messages
                                    //puts today's date inf the file name for the activity log
    do_script(start_log);
    send_census();                  // send the census of newly connected devices
    phone_off();                    // turn the phone module off
}


// PHONE IN DATA
// Turn on the phone and create a report
void phone_in(void)
{   //TODO// Update to send multiple reports for multiple sensors
//    phone_on() must be called before this routine.
    makeConnection();               // wait for the phone to establish an internet connection
    wait_a_sec(1);
    set_time();                     // reads RTC from the phone and saves it in messages
    // increment the report count, start over if it is a new day
//    if (day_count!= last_day_count) move_string("00", 0, 2, messages[DAYS_NUM], 4);   // move the two zeros to the string
//    int2ascii(ascii2int(messages[DAYS_NUM], 4, 2) + 1, messages[DAYS_NUM], 4, 2);   // increment report count
//    int2ascii(day_count, messages[DAYS_NUM], 0, 3);   //put the count into the string
//    last_day_count = day_count;     // save the count for next time
    send_report();                  // generate a report
    move_string(new_parameters, 0, PARAM_BYTES, parameters, 0); // save the current parameters to print out later
    get_new_parameters();      // upload a new set of paramters if they are available
    if (new_parameters_loaded)
     {   sensor_status[slot_count] = NEW_PARAMETERS_FOUND;
         //todo// send new parameters to sensor
         send_census();              // send the census of newly loaded parameters
     }
    phone_off();                    // turn the phone module off
    watchdog_reset();               // reset the watchdog timer
    // erase the previously uploaded data
    for (y = 0; y < MAX_RECORDS; y++) for (z = 0; z < 32; z++) data.bytes[y][z] = 0;  // Erase the entire array
    recCount = 0;                   // The count of how many records have been collected since last upload
    secCount = maxTime + TIME_OUT;  // Reset the secCount countdown counter
}


//////////////////////////////////////////////////////////
// START OF SENSOR PROGRAM CODE
//////////////////////////////////////////////////////////

//   SQUARE a number using MSP430FR57xx 16x16 signed Multiply
//   hardware module.
//   The calculation is automatically initiated after the second operand is
//   loaded. Results are stored in RESLO and RESHI.

unsigned long int square(signed int operand1)
{
    unsigned long int result;
    MPYS = operand1;      // Load first operand - signed
    OP2 = operand1;       // Load second operand
    __no_operation();
    __no_operation();     // wait a tiny bit
    result = RESHI;
    return(result<<16 | RESLO);
}


// Sleep for a set number of INTERVALS
void interval(unsigned char intervals)
{   while (intervals)
    {   ENTER_LOW_POWER_MODE_3;         // Enter low power mode until the next tick of the clock
        intervals--;
    }
}


void reset_SPI(void)
{   wait(1);
    ADXL_UART_RESET;                    // Reset the SPI
    __no_operation();
    ADXL_UART_ENABLE;           // confirm that SPI is enabled
    wait(1);                        // wait a millisecond
}


// RECEIVE a BYTE over the UART from the Comm board
char receive_byte(void)
{   unsigned char byte_rcvd;
    while (COAX_RX_BUFF_EMPTY) ;
    byte_rcvd = UCA0RXBUF;  //see what came in
    return (byte_rcvd);
}


// READ data from the ADXL sensor FIFO stack
void ADXL_FifoRead(void)
// Reading from the ADXL's FIFO buffer is a command structure that does not have an address.
// </CS down> <command byte (0x0D)> <data byte> <data byte> … </CS up>
{
    unsigned int temp;          // temporary storage for a read
    reset_SPI();                // confirm that SPI is enabled
    while(ADXL_BUSY);           // Make sure no activity is happening
    ADXL_SELECT;                // Select the chip by setting STE pin low
    __no_operation();           // let the ADXL settle (is this needed?)
    while(ADXL_TX_NOT_READY);   // wait for the TX buffer to empty
    ADXL_TX_BUFF= ADXL_FIFO ;   // Send command to transmit the data in the FIFO stack
    for( x = 2; x>0 ; x--)      // Ignore the first two bytes, they look like garbage
    {   while(ADXL_TX_NOT_READY) ;   // finish up transmit
        ADXL_TX_BUFF= 0xFF;     // Send two dummy bytes to push the FIFO command out of the buffer and the first byte of data off the stack
        temp = UCA1RXBUF ;      // empty the recieved buffer
    }
    for( x = 0; x < DATA_BUFFER_SIZE-1; x++ )     // Fill up the buffer with the data from the stack.  The first is junk
    {   while(ADXL_TX_NOT_READY) ;        // wait for the TX buffer to empty
        ADXL_TX_BUFF= 0xFF;            // Send another dummy byte to push data off the stack
        incoming.spi[x] = UCA1RXBUF; // get the byte from the previous send
        while(ADXL_TX_NOT_READY) ;        // wait for the TX buffer to empty
        ADXL_TX_BUFF= 0xFF;            // Send another dummy byte to push data off the stack
        temp = UCA1RXBUF ;
        incoming.spi[x] |= (temp<<8) ;   //  add the second byte * 256
    }
    while(ADXL_TX_NOT_READY) ;    // wait for SPI module to finish up
    temp = UCA1RXBUF;   // get the last byte
    ADXL_DESELECT;           // deselect
}


// PROCESS the DATA read in from the ADXL FIFO stack
// and prepare bin histogram
void Process_data(void)
{
    unsigned long int bin_mask;
    char bin_count;
    signed int tempX, tempY, tempZ;         // temp values
    signed int aX, aY, aZ;                  // value of accelerometer for each coordinate
    signed long int dX, dY, dZ;             // difference values for each coordinate
    signed long int mag_squared;            // absolute magnitude of vector squared (dX^2 + dY^2 +dZ^2)
        //Skip over bad data and synch up with the good data
    not_first_read = NO;
    for (x=0; x< DATA_BUFFER_SIZE-3 && !( ((incoming.spi[x] & X_READING) == X_READING) &&
                                        ((incoming.spi[x+1] & Y_READING) == Y_READING) &&
                                        ((incoming.spi[x+2] & Z_READING) == Z_READING)  );   x++ );
    while (x<DATA_BUFFER_SIZE)
    {           // process good data to find the "jerk" (third derivative of displacement)
        tempX=incoming.spi[x];
        tempY=incoming.spi[x+1];
        tempZ=incoming.spi[x+2];
        if (    ((tempX & XYZ_PINS) == X_READING) &&
                ((tempY & XYZ_PINS) == Y_READING) &&
                ((tempZ & XYZ_PINS) == Z_READING) )     // process only non-zero readings
        {        // make the data proper 16 bit integers
            aX= (tempX & ADXL_MASK) | ((tempX & NEG_MASK)<<2);  // mask off the top 2 bits and copy the sign bits there
            aY= (tempY & ADXL_MASK) | ((tempY & NEG_MASK)<<2);
            aZ= (tempZ & ADXL_MASK) | ((tempZ & NEG_MASK)<<2);
            if ( not_first_read )       // process the result into the bins
            {       // skip the first read because the delta will be very high
                    // analyzing the square of delta ACCELERATION (turns a 12 bit signed data into an unsigned 25 bit number)
                dX= (aX - aXold);               // compute the differences
                dY= (aY - aYold);
                dZ= (aZ - aZold);
                mag_squared = square(dX) + square(dY) + square(dZ);  // sum the square of the differences
                    // bin 29 (28 counting from bin 0) is the highest bit possible (but very unlikely)
                bin_mask = JOLT_MASK;           // Start looking at the highest bit possible
                for (bin_count=JOLT_BITS-1; (bin_count) && ((bin_mask & mag_squared) == 0); bin_count--)
                    //mask off one bit and see if it is zero
                    bin_mask >>= 1 ;            // if so move the mask over one bit to the right and look at the next bin
                buffer.bins[bin_count]++ ;      // increment the appropriate bin
                data.bins[recCount][READINGS]++ ;    // increment the number of readings
            }
            else
            {   not_first_read = TRUE ;
                        // Save the first reading off the stack in the orientation log
            // TODO // make this an average reading
                data.bins[recCount][X_POS] = aX;
                data.bins[recCount][Y_POS] = aY;
                data.bins[recCount][Z_POS] = aZ;
            }           // save the reading for next time in aXold, aYold, and aZold
            aXold = aX;
            aYold = aY;
            aZold = aZ;
        }
        x = x+3;        // increment the counter to look at the next readings
    }
}







// PROCESS the DATA read in from the ADXL FIFO stack
// and integrate the data regularly by summing squares of the delta
// The sums go into a long long (64) bits, so the maximum ODR and sample time before an overload
// is 25bits (the highest reading squared) summed 39 bit times at 400 samples per second is
// more than 20 years!
void integrate_data(void)
{
    unsigned long int bin_mask;
    char bin_count;
    signed int tempX, tempY, tempZ;         // temp values
    signed int aX, aY, aZ;                  // value of accelerometer for each coordinate
    signed long int dX, dY, dZ;             // difference values for each coordinate
    signed long int mag_squared;            // absolute magnitude of vector squared (dX^2 + dY^2 +dZ^2)

    //Skip over bad data and synch up with the good data
    for (x=0; x< DATA_BUFFER_SIZE-3 && !( ((incoming.spi[x] & X_READING) == X_READING) &&
                                        ((incoming.spi[x+1] & Y_READING) == Y_READING) &&
                                        ((incoming.spi[x+2] & Z_READING) == Z_READING)  );   x++ );
    while (x<DATA_BUFFER_SIZE)
    {           // process good data to find the "jerk" (third derivative of displacement)
        tempX=incoming.spi[x];
        tempY=incoming.spi[x+1];
        tempZ=incoming.spi[x+2];
        if (    ((tempX & XYZ_PINS) == X_READING) &&
                ((tempY & XYZ_PINS) == Y_READING) &&
                ((tempZ & XYZ_PINS) == Z_READING) )     // process only non-zero readings
        {        // make the data proper 16 bit integers
            aX= (tempX & ADXL_MASK) | ((tempX & NEG_MASK)<<2);  // mask off the top 2 bits and copy the sign bits there
            aY= (tempY & ADXL_MASK) | ((tempY & NEG_MASK)<<2);
            aZ= (tempZ & ADXL_MASK) | ((tempZ & NEG_MASK)<<2);
            if ( not_first_read )       // process the result into the bins
            {       // skip the first read because the delta will be very high
                    // analyzing the square of delta ACCELERATION (turns a 12 bit signed data into an unsigned 25 bit number)
                dX= (aX - aXold);               // compute the differences
                dY= (aY - aYold);
                dZ= (aZ - aZold);
                mag_squared = square(dX) + square(dY) + square(dZ);  // sum the square of the differences
                    // bin 29 (28 counting from bin 0) is the highest bit possible (but very unlikely)
      //          bin_mask = JOLT_MASK;           // Start looking at the highest bit possible
     //           for (bin_count=JOLT_BITS-1; (bin_count) && ((bin_mask & mag_squared) == 0); bin_count--)
                    //mask off one bit and see if it is zero
     //               bin_mask >>= 1 ;            // if so move the mask over one bit to the right and look at the next bin
     //           buffer.bins[bin_count]++ ;      // increment the appropriate bin
     //           data.bins[recCount][READINGS]++ ;    // increment the number of readings
            }
            else
            {   not_first_read = TRUE ;
                        // Save the first reading off the stack in the orientation log
            // TODO // make this an average reading
                data.bins[recCount][X_POS] = aX;
                data.bins[recCount][Y_POS] = aY;
                data.bins[recCount][Z_POS] = aZ;
            }           // save the reading for next time in aXold, aYold, and aZold
            aXold = aX;
            aYold = aY;
            aZold = aZ;
        }
        else    // once we hit zeros we're done.
        x = x+3;        // increment the counter to look at the next readings
    }
}













    // CLEAR THE TEMPORARY BINS
    // deletes all data from the temporary bins after they have been saved as a record
void clear_temp_bins(void)
{   for (bin =  0; bin < JOLT_BITS; bin++) buffer.bins[bin] = 0;
}


    // SEND A RECORD
    // send a record over the UART to the comm head
void send_a_record(int rec2send)
{   unsigned int temp = 0;                      // used to temporarily hold the data we're sending
    unsigned int data_count = 0 ;               // count how many bytes we'll send
    while (data_count <  DATA_SIZE)         // we'll send 16 integers
    {   temp=data.bins[rec2send][data_count];    // temporarily store the data to send
        while(COAX_TX_NOT_READY);                // wait for an empty buffer
        UCA0TXBUF= temp>>8 ;                // send the high byte first
        while(COAX_TX_NOT_READY);                // wait for an empty buffer
        UCA0TXBUF = temp & 0x00FF;          // send the low byte
        data_count++;                       // increment data count
    }
}


////////////// SMART SENSOR COMMANDS////////////////////////////////////////////////////


////////////// Command 0x00 -- cycles the ADXL sensor OFF and ON
    // This is a hardware reset of the ADXL by turning off all power to the chip
    // The chip starts up with its default values in standby
void ADXL_power_cycle(void)
{   unsigned char p2save[3], p4save[3];  // save the settings for Port 2
    // Save the settings for P2
    p2save[0] = P2OUT;  // Port 2 Port Select Register 1
    p2save[1] = P2SEL1; // Port 2 Direction Register
    p2save[2] = P2DIR;  // Port 2 Resistor Enable Register */
    p4save[0] = P4OUT;  // Port 4 Port Select Register 1
    p4save[1] = P4SEL1; // Port 4 Direction Register
    p4save[2] = P4DIR;  // Port 4 Resistor Enable Register */
    // Stop sending power to any of the SPI pins by setting the appropriate pins low on P2
    // The ADXL chip can only be reset if there is no voltage on ANY pin.
    P2OUT = P4OUT = 0;      // Port 2 and 4 Port Select Register 1
    P2SEL1 = P2SEL1 = 0;     // Port 2 and 4 Port Select Register 1
    P2DIR = P2DIR = 0xFF;   // Port 2 and 4 Direction Register (all outputs)
    // Turn off power to the ADXL and reset it
    ADXL_POWER_OFF;           // Setting the pin low turns off the sensor
    wait(100);                           //  Wait 100 msec. was 50
    // This may be more time than necessary,
    // but it needs to be long enough that the voltage goes to zero.
    // Then turn the ADXL back on
    // First, restore Port 2 settings
    P2OUT = p2save[0];  // Port 2 Port Select Register 1 */
    P2SEL1 = p2save[1]; // Port 2 Direction Register */
    P2DIR = p2save[2];  // Port 2 Resistor Enable Register */
    P4OUT = p4save[0];  // Port 4 Port Select Register 1 */
    P4SEL1 = p4save[1]; // Port 4 Direction Register */
    P4DIR = p4save[2];  // Port 4 Resistor Enable Register */
    ADXL_POWER_ON ;     // turn it on by setting the bit high
    wait(50);           // wait 50 ms, was 20 ms (sensor spec'd for 5)
}


////////////// Command 0x01 -- turns the ADXL sensor ON
    // This takes the sensor out of standby
    // and puts it in run mode.  Checks that it is actually on.
void ADXL_on (void)
{   unsigned char test_byte = 0;
//    ADXL_UART_ON;
    for (x= 0; x <10; x++)  // try ten times
    {       // Prepare the SPI
        reset_SPI();                // confirm that SPI is enabled
        ADXL_SELECT;          // Select the chip by setting STE pin low (P2.3)
            // Write the instruction to the ADXL to turn on
        while(ADXL_TX_NOT_READY);
        ADXL_TX_BUFF = ADXL_WRITE;     // Tell the ADXL to transmit the data in the FIFO stack
        while(ADXL_TX_NOT_READY);
        ADXL_TX_BUFF = 0x2D;           // send the address of the POWER_CTL byte
        while(ADXL_TX_NOT_READY);
        ADXL_TX_BUFF = PARAM_POWER_CTL_OFF | ADXL_ON;   // send the instructions for the POWER_CTL byte to turn on
        while(ADXL_BUSY);            // wait for it to finish transmitting
            // Reset the SPI
        ADXL_DESELECT;           // deselect
        wait(1);
        ADXL_SELECT;          // Select the chip by setting STE pin low (P2.3)
            // Confirm that the ADXL is on
        while(ADXL_TX_NOT_READY);
        ADXL_TX_BUFF = ADXL_READ;          // Tell the ADXL to transmit the data in a register
        while(ADXL_TX_NOT_READY);
        ADXL_TX_BUFF =  0x2D;              // send the address of the POWER_CTL byte
        while(ADXL_TX_NOT_READY);
        ADXL_TX_BUFF =  0xFF;              // send a dummy byte to receive a byte
        while(ADXL_BUSY);
        test_byte = UCA1RXBUF;          // read what came in
        ADXL_DESELECT;           // deselect
        if (test_byte == PARAM_POWER_CTL_OFF | ADXL_ON) return;
    }    // Time out after ten tries and report a problem with the sensor
    error_code = TIMEOUT_ERROR;
}


////////////// Command 0x02 -- RESET the ADXL sensor
    // Software and hardware reset of the ADXL sensor
    // and puts the sensor in standby mode.  It still needs to be turned on using ADXL_on().
    // returns an error code of 1 if the sensor does not turn on after 5 tries
char ADXL_reset(void)
{   unsigned char reset_tries = 5;
    unsigned char ADXL_not_operational = TRUE;
    unsigned char test_byte = 0;
    error_code = NO_ERROR;          // reset any previous error code
    while ( (ADXL_not_operational) && (reset_tries))    // keep resetting until things look good
    {  // TODO //  timing out.
        while (ADXL_BUSY);               // check to see if any other device is using the SPI line.
        ADXL_power_cycle();                     // Turn off power to the ADXL and reset it
        reset_SPI();                    // confirm that SPI is enabled
                    // do a software reset of the ADXL sensor
                    // ADXL SOFT RESET REGISTER Address: 0x1F, Name: SOFT_RESET
                    //  Writing Code 0x52 (representing the letter, R, in ASCII or
                    //  unicode) to this register immediately resets the ADXL362. All
                    //  register settings are cleared, and the sensor is placed in standby.
                    //  Interrupt pins are configured to a high output impedance mode
                    //  and held to a valid state by bus keepers.
                    //  This is a write-only register. If read, data in it is always 0x00.
        ADXL_SELECT;                    // Select the chip by setting STE pin low
        while(ADXL_TX_NOT_READY);       // wait for the SPI to finish
        ADXL_TX_BUFF = ADXL_WRITE;      // Tell the ADXL to write to its registers
        while(ADXL_TX_NOT_READY);       // wait for the SPI to finish
        ADXL_TX_BUFF= 0x1F;             // Send the address of the soft reset register
        while(ADXL_TX_NOT_READY);       // wait for the SPI to finish
        ADXL_TX_BUFF = 0x52;            // send the reset code
        while(ADXL_BUSY);               // wait for the last transmission to end
        ADXL_DESELECT;                  // Deselect the chip
        // initialize all the settings in standby mode
        reset_SPI();                    // confirm that SPI is enabled
        ADXL_SELECT;                    // Select the chip by setting STE pin low (P2.3)
        while(ADXL_TX_NOT_READY);       // wait for the SPI to finish
        ADXL_TX_BUFF = ADXL_WRITE;      // Tell the ADXL to write to its registers
        while(ADXL_TX_NOT_READY);       // wait for the SPI to finish
        ADXL_TX_BUFF= 0x20;             // Send the address of the first register (the ADXL auto increments after that)
        for( x = 0; x < 13; x++ )       // Send the first 13 parameters
        {   while(ADXL_TX_NOT_READY);   // wait for the SPI to finish
            ADXL_TX_BUFF= parameters[x] ;  // Send the setting to the register
        }
        while(ADXL_TX_NOT_READY);       // wait for the SPI to finish
        ADXL_TX_BUFF= PARAM_POWER_CTL_OFF & ~ADXL_STANDBY ;
        // Send the POWER_CTL setting to the register but make sure it is in standby
        while(ADXL_BUSY);               // wait for the SPI to finish
        ADXL_DESELECT;                  // deselect the ADXL
        wait(10);   // wait 10 ms for the chip to wake up before taking readings (spec'd for 5)
                    // Read back the FIFO register to see if it was set correctly.
                    // If so, we can read and write to the sensor.
        reset_SPI();                    // confirm that SPI is enabled
        ADXL_SELECT;                    // Select the chip by setting STE pin low (P2.3)
        while(ADXL_TX_NOT_READY);
        ADXL_TX_BUFF = ADXL_READ;       // Tell the ADXL to transmit the data in a register
        while(ADXL_TX_NOT_READY);
        ADXL_TX_BUFF =  0x29;           // send the address of the FIFO_SAMPLES byte
        while(ADXL_TX_NOT_READY);
        ADXL_TX_BUFF =  0xFF;           // send a dummy byte to receive a byte
        while(ADXL_BUSY);
        test_byte = UCA1RXBUF;          // read what came in
        ADXL_DESELECT;                  // deselect the ADXL
        wait(1);                        // wait 1 ms
        if (test_byte == PARAM_FIFO_SAMPLES) ADXL_not_operational = FALSE;
                // If is what we wrote to the register all is well
        else wait_a_sec(1);             // wait a second before trying again
        reset_tries--;                  // one less try
        if (!reset_tries) error_code = TIMEOUT_ERROR;
        //TODO// If we time out, there's a bug and/or we need to report by phone of a malfunction
    }
    return (error_code);
}


////////////// Command 0x03 -- Put the MSP430 TO SLEEP
    // Puts the microprocessor in a very low power mode and wakes it up
    // if it gets an interrupt from the ADXL sensor or the master board.
    // If the ADXL sensor has been turned off, then only an interrupt
    // from the master board on pin 3.3 or a power reset will wake it up
void msp2sleep(void)
{
    __bis_SR_register(LPM4_bits + GIE);         // Enter low power mode until an interrupt wakes up the microprocessor
}

//REWRITE
////////////// Command 0x04 -- CALIBRATE
    // Stores calibration data sent by the comm tower
    // Most of this process is controlled by the set-up board:
    // First a register check is done to upload the sensor settings to the set-up board
    // The settings are saved for later.  New settings are downloaded to stream temperature data
    // Buttons on the set-up board record the low temperature (an ice water bath at 0 degrees C)
    // and room temperature.  The reading at 0 degrees C becomes the temperature offset, and the ratio of 0x0200
    // divided by the reading at room temperature (minus the offset) becomes the temperature ratio.
    // This Calibrate command downloads the temperature ratio and the temperature offset to the
    // smart sensor and stores it for calibrating the temperature readings made by the sensor.
    // To simplify the math, the ratio is a two byte integer, which can be multiplied by the two byte
    // temperature reading after the offset is subtracted.  The result, shifted 12 bits right
    // can then be stored as the temperature reading:  0x0000  is zero degrees C;  0x0080 is 5 degrees C;
    // 0x0100 is ten degrees C;  0x0200 is twenty degrees C; etc...  Negative readings are possible.
unsigned char calibrate(void)
{   temperature_offset = receive_byte();    // get the first byte of the offset
    temperature_offset <<=8;                // make it the high byte
    temperature_offset += receive_byte();   // get the second byte
    temperature_ratio = receive_byte();     // get the first byte of the offset
    temperature_ratio <<=8;                 // make it the high byte
    temperature_ratio += receive_byte();    // get the second byte
    return (0);                             // do some error checking on the calibration data?
}


// REWRITE OR DELETE IF NOT NEEDED
////////////// Command 0x05 -- REGISTER Smart Sensor
    // Stores new registration data from comm tower (installation, time and GPS location)
    // Same logic as new_paramters() below.
unsigned char register_SS(void)
{   transfer_string(nest_ID, NEST_ID_BYTES, RECEIVE);
    return(0);
}

// REWRITE
////////////// Command 0x06 -- Load NEW PARAMTERS
    // Downloads new parameters from comm tower and stores them
    // This includes: ADXL register settings
    //                Record timings, and Record quantities
    // The format is first send the starting offset into the array, and how many bytes will be transferred.
    // followed by the value of the parameters.  Changing all 25 bytes of the parameters would require sending
    // 27 total.  It would start with a zero, and then 25 and then the parameters in order.
    // Changing just a single parameter byte would start with the offset into the array, then a one,
    // and than the value of the parameter.  Some parameters are more than one byte
    // so care must be taken to make sure all the bytes are sent.  The first parameter is number zero.
    // An error is generated if the offset plus the number of bytes goes out of range.
unsigned char set_new_parameters(void)
{   unsigned char offset;
    transfer_string(parameters, PARAM_BYTES, RECEIVE);
        // create integers out of parameter bytes (they might have changed)
    secMax =  (PARAM_SLOWBIN_HI_NOW<<8) | PARAM_SLOWBIN_LO_NOW ;    // The number of seconds to accumulate readings in each record (set of bins)
    max_run_time = PARAM_MAXRUN4H;
    for(offset=20; offset>17; offset--)
    {   max_run_time <<= 8;
        max_run_time |= parameters[offset];             // turn 4 unsigned chars into a long int
    }
    maxRecords = (PARAM_MAX_RECORDS_HI<<8) | PARAM_MAX_RECORDS_LO;  // The maximum number of records to collect
    ADXL_power_cycle();                         //  turn the sensor off!
    error_code = ADXL_reset();          // reset the ADXL and load the new parameters
    return(0);
}


////////////// Command 0x07 -- CLEAR last RECORD
    // Erases the last record and lowers the count
    // Also used by clear_all_records() to erase all the records
void clear_record(void)
{   if (recCount)       // Check that there is a record to erase!
    {   recCount--;     // Decrement the count of how many records have been collected since last upload
        for (z = 0; z < DATA_SIZE; z++) data.bins[recCount][z] = 0;  // erase the record
        recSecs = 0;    // Reset the number of seconds that the current record has been active
    }
}


////////////// Command 0x08 -- CLEAR ALL the RECORDS
    // erase all data and resets the counts
void clear_all_records(void)
{   recCount = MAX_RECORDS;         // Make sure they are all errased
while(recCount) clear_record(); // clear them all until they are all empty
recUploads = 0;                 // The number of records that have been uploaded
secCount = 0;                   // The number of seconds since last upload
clear_temp_bins();              // Erase these too.
}

//REWRITE
////////////// Command 0x09 -- SEND LAST RECORD
    // sends the most recent record created (the current recCount)
void send_last_record(void)
{       while (COAX_TX_NOT_READY);       //wait for an empty buffer
UCA0TXBUF= DATA_BYTES;      // we'll send 32 bytes
send_a_record(recCount);    // send the data
}

//REWRITE
////////////// Command 0x0A -- SEND ALL RECORDS
    // sends all the records in memory
void send_all_records(void)
{unsigned int temp_count=0;         // counts how many records have been sent
    while (COAX_TX_NOT_READY);      //wait for an empty buffer
    UCA0TXBUF= recCount >>8;        // we'll send the high byte of the number of records
    while (COAX_TX_NOT_READY);      //wait for an empty buffer
    UCA0TXBUF= recCount & 0x00FF;   // we'll send the low byte of the number of records
    while (COAX_TX_NOT_READY);      //wait for an empty buffer
    UCA0TXBUF= DATA_BYTES;          // we'll send 32 bytes in each record
    while (temp_count < recCount)   // until we've sent them all
    {   send_a_record(temp_count);      // send another record
        temp_count++;                   // increment the counter
    }
}

// REWRITE (eventually)
////////////// Command 0x0B -- MANUAL OPERATION
    // Manually reads or writes directly with the ADXL sensor without interpretation
    // Parameters for ADXL affect the result
    // can be used to change parameters or read from the X, Y, Z and Temp registers
    // This routine does not read from the ADXL FIFO buffer
    // use command 0x0C for that.
    // To use this, the read or write ADXL command should be sent, followed by the starting address of
    // the ADXL register, and then the number of bytes to read or write using SPI
void manual_operation(void)
{ unsigned char command, address, byte_count, count;
    reset_SPI();                    // confirm that SPI is enabled
    command = receive_byte();       // read, write but don't stream
    address = receive_byte();       // the address we will read from or write to
    byte_count= receive_byte();
    if ((command != ADXL_READ) && (command != ADXL_WRITE)) // it can only be a read or write
    {   error_code = OUT_OF_RANGE_ERROR;
        for (count = 0; count <byte_count; count++)
        {   if (command == ADXL_READ)       // reading from the sensor means writing to the comm tower
            {   while(COAX_TX_NOT_READY);   // wait for an empty buffer
                UCA0TXBUF = 0x0000;         // send zeros
            }
            else address = receive_byte();  // receive a byte but don't do anything with it
            count--;                        // decrement the counter
        }
        return;
    }
    // if things are in range we can continue
    if (command == ADXL_WRITE)      // If we're writing data we have to receive it first
    // read from the UART and write to the SPI
    {   for (count = 0; count <byte_count; count++) buffer.bytes[count] = receive_byte();
        // we'll count off the bytes we were told to receive,  get a byte and store it in the buffer
        ADXL_SELECT;      // Select the chip by setting STE pin low (P2.3)
            // Write the instruction to the ADXL to turn on
        while(ADXL_BUSY);            // Make sure no activity is happening
        ADXL_TX_BUFF = ADXL_WRITE;     // Tell the ADXL to read or write data
        while(ADXL_TX_NOT_READY)
        ADXL_TX_BUFF = address;        // send the start address of the register
        for (count = 0; count <byte_count; count++)
            {   while(ADXL_TX_NOT_READY);
                ADXL_TX_BUFF = buffer.bytes[count];    // send the data received
            }
        while(ADXL_BUSY);            // wait for it to finish transmitting
        ADXL_DESELECT;       // deselect
        return;
    }
    //otherwise we are reading from the SPI and writing to the UART
    reset_SPI();                    // confirm that SPI is enabled
    ADXL_SELECT;          // Select the chip by setting STE pin low (P2.3)
    __no_operation();               // let the ADXL settle
    while(ADXL_TX_NOT_READY);             // wait for the TX buffer to empty
    ADXL_TX_BUFF = ADXL_READ;          // Tell the ADXL to transmit the data in a register
    while(ADXL_TX_NOT_READY);
    ADXL_TX_BUFF= address ;            // Send the address of the first read locationn
    for (count = 0; count <byte_count; count++)
    {       // Fill up the buffer with the data from the ADXL
        while(ADXL_TX_NOT_READY) ;        // wait for the TX buffer to empty
        ADXL_TX_BUFF= 0xFF;            // Send another dummy byte to push data off the stack
        buffer.bytes[count] = UCA1RXBUF;    // get the byte from the previous send
    }
    while(ADXL_TX_NOT_READY) ;            // wait for SPI module to finish up
    address = UCA1RXBUF;            // get the last byte but ignore it
    ADXL_DESELECT;           // deselect
    for (count = 0; count <byte_count; count++)
    {   while(COAX_TX_NOT_READY);        // wait for an empty buffer
        UCA0TXBUF = buffer.bytes[count];    // send the data in the buffer
    }
}

//REWRITE
////////////// Command 0x0C -- Stream data
    // Sends out sensor data as soon as it is read off the ADXL FIFO stack
    // Parameters for ADXL affect how data is streamed and should be set before calling this routine.
    // This routine does not read the X,Y, Z and Temp registers.
    // The ADXL chip must be reset and turned on before using this routine.
void stream_data(void)
{   reset_SPI();                // confirm that SPI is enabled
//This routine needs to be rewritten
}

// REWRITE IF NEEDED
////////////// Command 0x0D -- CHECK ID
    // uploads the current nest ID registration to the comm tower
    // The send_string rountine receives two more bytes,
    // the starting byte and how many bytes are requested
    // first byte sent is the number of bytes being sent,
    // then they are sent in order.  The number of bytes being sent will be zero
    // if the starting byte and/or number of bytes requested is out of range.
void check_ID(void)
{   transfer_string(nest_ID, NEST_ID_BYTES, SEND);
}

// REWRITE
////////////// Command 0x0E -- SEND PARAMETERS
    // Uploads current parameters and counts  to comm tower
    // The send_string rountine receives two more bytes,
    // the starting byte and how many bytes are requested
    // first byte sent is the number of bytes being sent,
    // then they are sent in order.  The number of bytes being sent will be zero
    // if the starting byte and/or number of bytes requested is out of range.
void send_parameters(void)
{   transfer_string(parameters, PARAM_BYTES, SEND);
}

// LIKELY NO LONGER NEEDED
////////////// Command 0x0F -- SEND a PROGRESS REPORT (error_code)
    // Sends a one byte code that explains the reason for status of the smart sensor
    // The report is two bytes.  The first one is the last command sent, and
    // the second is the last status/error code.
    // All codes less than 0x80 are not errors.
    // 0x00 = No interrupt was generated -- operation in progress
    // 0x01 = INT1 generated an interrupt
    // 0x02 = INT2 generated an interrupt
    // 0x04 = operation completed successfully, data ready to report
    // 0x08 = operation completed successfully, no data to report
    // 0x10 = normal start-up
    // 0x20 = command received, operation underway, communication over
    // >= 0x80 = an error condition exists.  The error code is returned:
    // 0x81 = Sensor Start up failure (timed out)
    // 0x82 = Unknown command received
    // 0x84 = Bad Data received
    // 0x88 = Data transmission timed out
    // 0x90 = Parameters out of range
    // 0xA0 = Unknown Start up error


void send_progress_report(void)
{   UCA0CTL1 |= UCSWRST;            // Reset the UART
    __no_operation();
    UCA0CTL1 &= ~UCSWRST;           // confirm that UART is enabled
    wait(1);
    while(COAX_TX_NOT_READY);                // wait for an empty buffer
    UCA0TXBUF= command ;                // send the code
    wait(1);
    while(COAX_TX_NOT_READY);                // wait for an empty buffer
    UCA0TXBUF= error_code ;             // send the code
}

// KEEP AND REWRITE FOR PHASE 2 COMPATIBILITY
////////////// Command RESUME_RUN -- Resume Run and Process data in bins
    // resets the sensor and gets things going
void resume_run_bins(void)
{   volatile unsigned int temp, temp1, temp2;
    volatile unsigned char reads, out_of_time = 0;
    volatile long int temp_temp; // temporary temperature reading

            // Continue until it is time to stop
    while ((recCount < maxRecords) && (!out_of_time))  // continue if there is room for more and still time
    {                           // recCount goes from 0 to MAXRECORDS minus 1
        P2OUT &= ~BIT0;
        for (reads = PARAM_READ_SPEED; reads > 0 ; reads-- )
        {   interval(PARAM_SLEEP_INTERVALS);       // Enter low power mode 3 for a fraction of a second (the loop starts up again every 1/8 second -- restored by timer interupt)
            ADXL_FifoRead();        // Read the data off the ADXL FIFO stack
            Process_data();         // process the data and store in buffer.bins array
        }

                // increment counters
        recSecs++;                  // Increment the number of seconds in the current record
        secCount++;                 // Increment the total number of seconds since last transmission
        out_of_time = (secCount > max_run_time);
        if ((recSecs >= secMax) || out_of_time) // Check to see if it is time to start a new record or time has run out

                // Process and store the temporary bins in the record.
        {   bin = JOLT_BITS - 1;    // Start with the bin with the highest readings
            while( (bin>0) && (buffer.bins[bin] == 0) ) bin =  bin - 1 ; // Find the highest bin with a non zero count in it
            data.bins[recCount][HIGHEST_BIN]= bin + 1;   // Save it in the highest_bin array (1-25)
            if (bin < MAX_BIN-1) bin = MAX_BIN-1;   // is room for all the bins with non-zero data if not, we'll drop the lowest ones
            z = DATA_SIZE;                          // Counter for the real data storage bins in the record
            while (z > BINS)
            {   // transfer the temporary data into the record
                z =  z - 1;                             // the range of the array is MAX_BIN - 1 to zero.
                data.bins[recCount][z] = buffer.bins[bin];       // move the temp bin to the array.
                bin = bin - 1 ;                         // fill all of them.
            }
            clear_temp_bins();  // reset the buffer.bins for the next record

                // Get a temperature reading
            reset_SPI();                        // confirm that SPI is enabled
            temp_temp = 0;                      // start out with zero
            for (z=8; z>0; z--)                 // This is looped 8 times for an average reading
            {   ADXL_SELECT;          // Select the chip by setting STE pin low (P2.3)
                ADXL_TX_BUFF = ADXL_READ;          // Tell the ADXL to transmit the data in a register
                while(ADXL_TX_NOT_READY);             // wait for the TX buffer to empty
                ADXL_TX_BUFF =  0x14;              // send the address of the temperature byte
                while(ADXL_TX_NOT_READY) ;            // wait for the TX buffer to empty
                ADXL_TX_BUFF= 0xFF;                // Send a dummy byte to push the address out of the buffer
                while(ADXL_TX_NOT_READY) ;            // wait for the TX buffer to empty
                ADXL_TX_BUFF= 0xFF;                // Send another dummy byte to push data off the stack
                temp = UCA1RXBUF;               // this byte is junk -- clear the buffer
                while(ADXL_RX_NOT_READY);             // wait for the TX buffer to empty
                ADXL_TX_BUFF= 0xFF;                // Send another dummy byte to push more data off the stack
                temp1 = UCA1RXBUF;              // get the low byte of temperature byte from the previous send
                while (ADXL_RX_NOT_READY)             // get the second byte received
                temp2 = UCA1RXBUF;              // get the low byte of temperature byte from the previous send
                temp_temp += (temp2<<8)|temp1 ; // add the second byte * 256
                ADXL_DESELECT;           // deselect
                wait(1);
            }
            data.bins[recCount][TEMPS] = temp_temp>>3;   // divide 8 accumulated readings by 8
            if (parameters[24])                     // check to see if calibration is on
            {       // use calibration data to adjust the temperature reading
                  MPYS = temp = data.bins[recCount][TEMPS] - temperature_offset ;    // Load first operand - signed reading
                  OP2 = temperature_ratio;                                      // Load second operand - unsigned ratio
                  __no_operation();
                  __no_operation();     // wait a tiny bit for the multiplication to finish
                  data.bins[recCount][TEMPS] = (temp & 0x8000) |RESHI<<4 | RESLO>>12 ;   // divide result by 0x01000 and restore sign
            }

                // prepare for a new record
        recSecs = 0 ;           // reset the second counter
        recCount++ ;            // and add a new record

        }
    }
}

// KEEP AND REWRITE FOR PHASE TWO COMPATIBILITY
////////////// Command RUN_BINS -- START a new RUN and Process data in BINS
    // resets the sensor and gets things going
void start_run_bins(void)
{   clear_all_records();    // Reset the memory
    if (ADXL_reset()) return; // Reset and initialize the ADXL
    ADXL_on();              // Turn the sensor on
    wait_a_sec(1);          // Wait a second
    clear_temp_bins();      // Reset the temporary bins
    ADXL_FifoRead();        // Read the FIFO stack to empty it of initial bad reads (we lose a second of data)
    recCount = 0;           // Reset counts
    secCount = 1;           // Reset the number of seconds we have been running
    data.bins[0][READINGS] = 0;  // Reset the number of readings to zero
    if (error_code & 0x80) return;  // if there is an error, don't go any further
    resume_run_bins();              // start 'er up.
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////// NEW PHASE THREE COMMANDS ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void synch_event_clock(void)
{ //  INTERVAL_INTERRUPT_DISABLE;      // no interrupts while we're doing this
    STOP_INTERVAL_TIMER;             // Stop the 1/8 second interval timer for now.
    SHORT_INTERVAL;                  // reset the timer for the normal interval length - 48 (eighths of a msec)
    ZERO_TIMER;                      // reset the timer
    START_INTERVAL_TIMER;            // start or restart the timer counter
    interval_count = 0;              // synch the interval
   // INTERVAL_INTERRUPT_ENABLE;        // interrupts OK now
}

// send data out in small chunks that take more time than available during an interval
unsigned char report_data(void)
{
return(NO);
}

void stop_data_run(void)    // stops collecting data
{
return;
}


// initiates process of collecting data and integrating the energy as it goes
void start_integration(void)
{
return;
}


// the ongoing process of collecting and processing whatever process has begun
void collect_data(void)
{
return;
}


// sends out any data found on the device after a restart
void send_old_data(void)
{
return;
}



//generates a random int between 0 and the number of bits specified (10 bits = 1023)
void generate_random(unsigned char bits)
{   unsigned int mask = 0;
    clear_all_records();        // Reset the memory
    if (ADXL_reset()) return;   // Reset and initialize the ADXL
    ADXL_on();                  // Turn the sensor on
    wait(bits<<4);              // Wait 16 msec per bit
    ADXL_FifoRead();            // Read the data off the ADXL FIFO stack
    for( x=bits + bits + 10; x>9; x-=2 )                           // double bits to decide how many readings
    {   random = (random<<1) | (incoming.spi[x] & 0x01) ;   //Just use the low bits from the 10 readings
        mask = (mask<<1) | 1;   // create a mask to get rid of
    }
    random &= mask;             //Just the number of bits needed
    return;
}

// using the SMCLK, waits a random number of cycles
void wait_random(void)
{ unsigned char old_intervals_on;
    STOP_RANDOM_TIMER;                 // Sets the value of the capture-compare register
    TIMER_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0, random);
                                 // Starts timer counter
    TA0R = 0;
    old_intervals_on = intervals_on; // save this
    intervals_on = NO;               // stop the 1/8 sec clock wake-ups
    START_RANDOM_TIMER;              // start the timer
    ENTER_LOW_POWER_MODE_3;          // sleep until the timer has counted up to the value just loaded
    STOP_RANDOM_TIMER;               // Sets the value of the capture-compare register
    intervals_on = old_intervals_on; // intervals_on restored to previous setting
    return;
}

// sleep until interval count gets to the last interval
void wait_out_cycle(void)
{   while (interval_count < interval_limit -1)
        ENTER_LOW_POWER_MODE_3;
}

//////////////////////////////////////////////////////////////
//////////////////// Command: synch_slave ////////////////////
//////////////////////////////////////////////////////////////
    // Synchs the interval clock to restart immediately after receiving this command
    // The Master's interval clock will thereby be a millisecond or two ahead of sensor
    // because of this, the slave needs to adjust the clock to be slightly ahead of the master.
    // The Slave can be called by the master as soon as the master awakens at its scheduled interrupt and the slave will be ready
    // The synch_slave command is sent at the beginning of every clock cycle (default every 6 seconds)
    // Any or all the slaves can be synched at the same time. Typically all slaves will synch together (this_unit = 0xFF)
    // If a unit gets out of synch, it will listen for at least 2 cycles for a synch instruction before re-registerring
void synch_slave(void)
{   unsigned char connected = NO;
//    INTERVAL_INTERRUPT_DISABLE;
    STOP_INTERVAL_TIMER;                    // Stop the 1/8 second interval timer for now.
    while (!connected)                      // Loop until we find the slot zero synch command.
    {   timeout_during_rx = NO;             // This sets a very long timeout for receiving
        while(receive_message() == NO) ;    // receive until we get a command without an error.
        if ( (received_command_bank == CORE_COMMAND_BANK)
            &&(received_command == SYNCH) )     // If this is this the synch command,
        {   synch_event_clock();            // synch up the interval clock and get it started.
            connected = YES;                // Flag that we're done here.
        }                                   // Keep trying until we get the synch command.
    }                                       // We'll stop looping when the connected flag is set
    timeout_during_rx = YES;                // Once synched the timeout can be short.
}


// receives a message, checks against what is expected.
unsigned char rx_checked(unsigned char rcv_msg)
{   if (receive_message())                          // receive a message and get a reply
        {   if (received_command == rcv_msg) // if the reply is what was expected
                return (YES);                       // then everything is good
            else tx_error = WRONG_COMMAND;          // otherwise indicate the error
        }
    return (NO);                                    // everything was not good unless everything was perfect
}


// sends a message, checks the reply against what is expected.
unsigned char tx_rx_checked(unsigned char transmit_msg, unsigned char receive_msg)
{   if (coax_short_message(transmit_msg))       // if we send a message
        return(rx_checked(receive_msg));        // and get a reply
    return (NO);                                // everything was not good unless everything was perfect
}


// sends a message, receives a reply.
unsigned char tx_rx(unsigned char transmit_msg)
{   if ((coax_short_message(transmit_msg))      // send a message (aborts if there's an error)
        && (receive_message()))                 // get a reply, if we're successful
            return(YES);                        // then all is good
    return(NO);                                 // otherwise all is not good
}


// receives a message, checks it against what is expected, and send a reply
unsigned char rx_tx_checked(unsigned char rx_msg, unsigned char tx_msg)
{   if (receive_message())                      // receive a message and there was no error
    {   if (received_command == rx_msg)  // and the reply is what was expected
        {    //wait(1);
             return (coax_short_message(tx_msg));// send a message and report success
        }
        else tx_error = WRONG_COMMAND;               // indicate the erro
    }
    return (NO);                                // everything was not good unless everything was perfect
}

// empties the registration of all the slots at start-up
void clear_slots(void)
{ unsigned char slot_count = 0;
    for ( ; slot_count < SENSOR_SLOTS; slot_count++)
        sensor_slot[slot_count] = 0;
    device_count = 0;
}



// get the sensor registerred
// the sensor/master communication must complete without an error for registration to proceed
unsigned char get_registered(void)
{      // TODO // add a wait of a random amount of msec if multiple sensors
    registering = YES;     // set a flag that is reset if anyone else starts to transmit
    coax_reset();
    COAX_RX_START_ENABLE;  // receiving a start bit resets the registering flag cancelling registration
    wait_random();         // wait our random amount of time
    COAX_RX_START_DISABLE; // either the random timer has finished or we've starte to receive
    if (registering)       // indicates that we were the first to try an register
    {   if ( (tx_rx_checked(REQUEST_TIME_SLOT, REQUEST_ID))
            && (coax_long_message(IDENTIFICATION, 0, 0, serial_num, SERIAL_ID_LEN))
            && (rx_checked(REGISTRATION)) )
        {   this_unit = parameter_one;
            if ( coax_short_message(REGISTERED))
                return (YES);
        }
    }
    generate_random(11);     // if anything goes wrong generate a new random number.
    this_unit = DEFAULT_SLAVE;
    return(NO);
}



unsigned char register_device(void)
{  unsigned char slot_count;    // a counter to find an unused slot
    if (tx_rx_checked(EMPTY_TIME_SLOT, REQUEST_TIME_SLOT))  // let the devices know a slot is available
    {   if (tx_rx_checked(REQUEST_ID, IDENTIFICATION))      // if a device wants the slot, ask sensor to send the sensor_ID
        {                          //make sure the data isn't too big for the array
            if (received_datalength > NEST_ID_BYTES) received_datalength = NEST_ID_BYTES;
                                   // find the first empty slot
            for (slot_count = 2; ((slot_count < SENSOR_SLOTS) && (sensor_slot[slot_count] > 0)); slot_count++);
            if (slot_count >= SENSOR_SLOTS) slot_count = 0;   // if there are no available slots, return a zero
            // TODO // it would be possible here to check and see if this sensor were recently connected
            else                       // save the serial number (last sensor rejected gets stored in slot zero)
            {   move_string(incoming.uart, 0, received_datalength, sensor_ids[slot_count], 0);
                if  (coax_long_message(REGISTRATION, slot_count, 0, " ", 1 )) // if the sensor received the info
                {   going_to_unit = slot_count;
                    if (rx_checked(REGISTERED))
                    {   sensor_slot[slot_count] = YES;  // the success of registration is stored
                        sensor_status[slot_count] = RECENTLY_CONNECTED;  //This is used for census reporting
                        device_count++;   // increment the number of devices
                        return(YES);
                    }
                    else sensor_slot[slot_count] = NO;   // the failure of registration is stored and returned
                }
            }
        }
        return (NO);
    }
    tx_error = NO;              // an error means that nobody tried to register
    return (NO);
}





//    move_string(nest_ID,  SERIAL,SERIAL_BYTES, messages[30], 0);        // store the serial number
//    move_string(nest_ID,  DAY, DAY_BYTES, messages[32], 8);             // store the date installed
//    move_string(nest_ID,  MONTH, MONTH_BYTES, messages[32], 5);
//    move_string(nest_ID,  YEAR, YEAR_BYTES, messages[32], 2);
//    move_string(nest_ID,  LATITUDE, LATITUDE_BYTES, messages[39], 0);   // store the nest location (GPS not implemented)
//    move_string(nest_ID,  LONGITUDE, LONGITUDE_BYTES, messages[40], 0);
//    move_string("00", 0, 2, messages[DAYS_NUM], 4); // reset the report counter
//    phone_in();                     // report that the new sensor was plugged in
//    set_report_frequency();         // decide how often to report
//    coax_short_message(START_RUN);     // send SS the command to collect data and process
//}


void slave_routines()
{ unsigned char error_code;
  unsigned char processing = YES;
    while (processing)              // keep looping until the conversation is done
    {   error_code = NO;            // clear any errors from the last go round
        switch (received_command)                // process the command

        {   case SYNCH:
                slave_registered = NO;                    //  give up and re-register
                this_unit = DEFAULT_SLAVE;
                synch_slave();
                status = READY;
                processing = NO;
                break;
            case SIGN_OFF:
                processing = NO;
//            case REGISTERED:                    // slave shouldn't get this command
//                break;
            case TURN_ADXL_OFF :
                ADXL_power_cycle();                 // Hardware power cycle -- puts ADXL in default low power standby
                status = READY;
                received_command = STATUS;
                break;
            case TURN_ADXL_ON :
                ADXL_on();                  // turns the sensor on
                status = READY;
                received_command = STATUS;
                break;
            case RESET_ADXL :
                if(ADXL_reset()); // does a hardware and software reset of the ADXL and puts it in standby
                {   status = READY;
                    received_command = STATUS;
                }
                break;
           case CALIBRATE_ADXL:
                if(!(error_code = calibrate())); // Receive temperature calibration data from Comm board
                {   status = READY;
                    received_command = GOOD_TRANSMISSION;
                }
                break;
            case REGISTRATION :                 // shouldn't happen in this time slot
                break;
            case PARAMETERS:
                if(!(error_code = set_new_parameters()));         // Receive new parameters from Comm board
                {   status = READY;
                    received_command = GOOD_TRANSMISSION;
                }
                break;
            case CLEAR_LAST_REC :
                clear_record();             // Deletes the last bin record
                status = READY;
                received_command = STATUS;
                break;
            case CLEAR_ALL :
                clear_all_records();        // Deletes all the bin records
                status = READY;
                received_command = STATUS;
                break;
            case SEND_LAST_REC :
                send_last_record();         // sends out the last bin record
                status = READY;
                received_command = STATUS;
                break;
            case REQUEST_DATA :
                send_all_records();         // sends out all the collected data recods
                status = READY;
                received_command = STATUS;
                break;
            case MANUAL_OPERATION:
                status = BUSY;
                manual_operation();         // sends and receives data directly to and from the ADXL sensor
                processing = NO;
                break;
            case STREAM_DATA:
                status = BUSY;
                stream_data();              // streams data from the sensor's FIFO stack
                processing = NO;
                break;
            case REQUEST_ID :
                check_ID();                 // sends the sensor ID to the comm board
                status = READY;
                received_command = STATUS;
                break;
            case REQUEST_PARAMETERS :
                send_parameters();          // sends requested parameters to the comm board
                status = READY;
                received_command = STATUS;
                break;
            case STATUS :                   // does nothing, just returns the status
                processing = (tx_rx(status));   // if we send the status and get a message back
                                                // continue processing the reply
                break;
            case RESUME_RUN :
                status = BUSY;
                resume_run_bins();          // continues a run after an interruption
                received_command = STATUS;
                break;
            case RUN_BINS :
                start_run_bins();           // starts a new run after resetting everything
                status = BUSY;
                received_command = STATUS;
                break;
            case START_RUN :
                status = BUSY;
                start_integration();
                received_command = STATUS;
                break;
            case STOP_RUN :
                status = READY;
                stop_data_run();
                received_command = STATUS;
                break;
            default :
                tx_error = BAD_COMMAND_ERROR; // the wrong byte was sent or there was a communication error

        }
    if (tx_error) tx_rx(RESEND_LAST);
    }
}


unsigned char master_routines()       // this is where the master decides what to do next when it knows the status of the slave
{ unsigned char processing = YES;
    while (processing)              // keep looping until the conversation is done
    {   switch (received_command)                // process the command

        {   case READY:
                    // nothing yet,  this will start a run
                processing = NO;
                break;
            case REGISTERED:                    // this shouldn't happen here
                break;

            case SUSPEND :
                break;

            case BUSY:
                break;

            case GOOD_TRANSMISSION:
                break;

            case DATA_ERROR :
                break;

            case DATA :
                break;

            default :
                tx_error = BAD_COMMAND_ERROR; // the wrong byte was sent or there was a communication error

        }
        if (tx_error) tx_rx(RESEND_LAST);
    }
    return (tx_error == NO_ERROR);      //True if no error, false otherwise
}




///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// Hardware initialization ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

void master_settings(void)
{   ///////PORT SETTINGS FOR MASTER/////////
    PHONE_UART_RESET;
    COAX_UART_RESET;    // According to manual, the pins should be set while in reset
    // Grace settings are preset for the slave
        /* Port 1 Output Register */
        P1OUT = 0;
        /* Port 1 Direction Register */
        P1DIR = BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7; // All unused so set as outputs
        /* Port 1 Interrupt Edge Select  */
        P1IES = 0;
        /* Port 1 Interrupt Flag Register */
        P1IFG = 0;
        /* Port 2 Output Register */
        // Pin 2.4 only goes low to turn on the phone, so its default is high
        P2OUT = BIT4;
        /* Port 2 Port Select Register 1 */
        // Pins 2.0 and 2.1 used for COAX UART
        // Pins 2.5 and 2.6 used for Phone UART
        P2SEL1 = BIT0 | BIT1 | BIT5 | BIT6;
        /* Port 2 Direction Register */
        // Pin 2.2 unused set as output
        // Pin 2.3 is the Power Monitor of the phone, and is an input
        // Pin 2.4 turns the phone on and off, and is an output
        // Pin 2.7 is the Phone Status Monitor, and is an input
        P2DIR = BIT2 | BIT4;
        /* Port 2 Interrupt Edge Select */
        P2IES = 0;
        /* Port 2 Interrupt Flag Register */
        P2IFG = 0;
        /* Port 3 Output Register */
        P3OUT = 0;
        /* Port 3 Port Select Register 1 */
        // Pin 3.4 is the SMCLK output
        P3SEL1 = BIT4;
        /* Port 3 Direction Register */
        // Pin 3.0 is the battery monitor, so it is an input
        // Pin 3.1 is the Phone Reset, and is an output
        // Pin 3.2 and 3.3 are unused, so are set as outputs
        // Pin 3.5 is the Sensor Enable, and is an output
        // Pin 3.6 is the Aux Enable, and is set high
        P3DIR = BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
        /* Port 3 Interrupt Edge Select */
        P3IES = 0;
        /* Port 3 Interrupt Flag Register */
        P3IFG = 0;
        /* Port 4 Output Register */
        P4OUT = 0;
        /* Port 4 Direction Register */
        // Pin 4.0 is the green LED
        // Pin 4.1 is the red LED, so both are set as outputs
        // Pin 4.4 turns on the battery monitor, so it is an output
        P4DIR = BIT0 | BIT1 | BIT4;
        /* Port 4 Interrupt Edge Select */
        P4IES = 0;
        /* Port 4 Interrupt Flag Register */
        P4IFG = 0;
        /* Port J Output Register */
        PJOUT = 0;
        /* Port J Port Select Register 0 */
        PJSEL0 = BIT4 | BIT5;
        /* Port J Direction Register */
        PJDIR = 0;
        COAX_UART_ENABLE;
        PHONE_UART_ENABLE;
}


void slave_settings(void)
{   ///////PORT SETTINGS FOR SLAVE/////////
    ADXL_UART_RESET;
    COAX_UART_RESET;    // According to manual, the pins should be set while in reset
    // This does not include the settings for the moisture sensor
    /* Port 1 Output Register */
    P1OUT = 0;
    /* Port 1 Direction Register */
    P1DIR = 0;
    /* Port 1 Interrupt Edge Select */
    P1IES = 0;
    /* Port 1 Interrupt Flag Register */
    P1IFG = 0;
    /* Port 2 Output Register */
    P2OUT = 0;
    /* Port 2 Port Select Register 1 */
    P2SEL1 = BIT0 | BIT1 | BIT4 | BIT5 | BIT6;
    /* Port 2 Direction Register */
    P2DIR = BIT3 | BIT7;
    /* Port 2 Interrupt Edge Select */
    P2IES = 0;
    /* Port 2 Interrupt Flag Register */
    P2IFG = 0;
    /* Port 3 Output Register */
    P3OUT = 0;
    /* Port 3 Port Select Register 1 */
    P3SEL1 = BIT4;
    /* Port 3 Direction Register */
    P3DIR = BIT2 | BIT4;
    /* Port 3 Interrupt Edge Select */
    P3IES = 0;
    /* Port 3 Interrupt Flag Register */
    P3IFG = 0;
    /* Port 4 Output Register */
    P4OUT = 0;
    /* Port 4 Direction Register */
    P4DIR = 0;
    /* Port 4 Interrupt Edge Select */
    P4IES = 0;
    /* Port 4 Interrupt Flag Register */
    P4IFG = 0;
    /* Port J Output Register */
    PJOUT = 0;
    /* Port J Port Select Register 0 */
    PJSEL0 = BIT4 | BIT5;
    /* Port J Direction Register */
    PJDIR = 0;
    ADXL_UART_ENABLE;
    PHONE_UART_ENABLE;
}


////////////////////////
// START UP routines -- only runs after a reset or cold start
void common_startup(void)
{   WDTCTL = WDTPW | WDTHOLD;       // Stop watchdog
    PM5CTL0 &= ~LOCKLPM5;           // Disable the GPIO power-on default high-impedance mode to activate
                                    // previously configured port settings
    coax_reset();                   // Put Master's eUSCI in reset
    SMCLK_OFF;                      // Turn off the SMCLK -- only used during UART transmissions
    STOP_RANDOM_TIMER;              // Turn off the random interval timer
    STOP_MSEC_TIMER;
//     first_start = NO;            // Once the batteries are fully charged we can keep going
    watchdog_reset();               // Remove hold from watchdog
    going_to_bank = this_bank = TURTLE_SENSE_BANK;   //we'll just be using one bank of sensors
    current_command_bank = CORE_COMMAND_BANK;                   //and one bank of commands
    interval_limit = 48;
    interval_count = 0;             // reset the interval clock
    intervals_on = YES;         // The slave never is, the master starts off quiet.
}


/////////////////  slave() -- the main routine for SLAVE devices ////////////////////////////////
// receives commands from  the master and collects, processes and transmits data
//
void slave()
{ unsigned char temp = error_count = 0;
    slave_settings();               // Settings without using GRACE
    common_startup();               // Configuration settings common to both MASTER and SLAVE
    STOP_INTERVAL_TIMER;            // stop the timer for now
    wait(512);                      // wait 1 second
    generate_random(11);            // creates a random number between 0 and 4095
    slave_registered = NO;          // flag that slave is not registered
    status = READY;                 // TODO// this might not get reset after a power failure
    suspended = NO;                 // start out in not suspended mode -- master decides when to suspend
    reporting_data = NO;            // send data only when requested
    going_to_unit = DEFAULT_MASTER; // slaves only talk to masters
    this_unit = DEFAULT_SLAVE;      // start out unregistered
    synch_slave();                  // first get in synch ith what is happening
    while (TRUE)                    // just loop
    {   coax_reset();               // clear out any junk
        ENTER_LOW_POWER_MODE_3;     // wait until the next interval begins
// There are two modes, suspended mode (suspended = YES) and not suspended mode (suspended = NO)
// Suspended mode is for when the cell phone board is on and reporting data
        if (suspended)              // hopefully we won't go out of synch in a few minutes
        {   if ((interval_count == 1)  // listen for messages during interval one, but don't disconnect if there aren't any
                && (receive_message()) ) // any messages now are about sending data
            {   if (received_command == REQUEST_DATA)
                {   reporting_data = report_data();     // set up data reporting and sends a block of data
                                                           // TRUE = Still reporting, FALSE  = Done or fatal error
                }
                else if (received_command == WAKE_UP)
                    suspended = NO;
            }
            else if (reporting_data)  // the data gets sent in 1K chunks during each interval
                reporting_data = report_data();
        }
// If not suspended mode, the units handshake their status during their assigned slot
// interval zero is always just for resynching
        else switch (interval_count)
        {   case 0:
            if ( (receive_message())
                && (received_command == SYNCH)                 // if no error receiving
                && (received_command_bank == CORE_COMMAND_BANK) )
                // and the command is to synch all devices,
            {   synch_event_clock();                // if so, synchronize our watches
            }
            else
            {   slave_registered = NO;              // otherwise give up and re-register
                this_unit = DEFAULT_SLAVE;          // release the slot
                synch_slave();                      // and get in synch with what is happening
            }
            break;
// interval one is for adjusting the clock, registering the sensor and housekeeping
// Also controls when to report data and suspend handshaking
            case 1:           // adjust the interval timer back to a full 125 msec
            STOP_INTERVAL_TIMER;                // stop the timer while adjusting
            ZERO_TIMER;
            NORMAL_INTERVAL;                    // reset to normal interval length after a synch interval
            START_INTERVAL_TIMER;               // start or restart the timer counter
            // This is also be the interval for registerring
            if ((!slave_registered)             // if slave_registered is zero then no slot has been assigned
                && (receive_message())          // and there's no error receiving the message
                && (received_command == EMPTY_TIME_SLOT))    // and the message says a slot is available
            {   if (get_registered())           // register the sensor and assign it an interval
                {   status = READY;             // if the interval is non-zero we are connected and ready
                    slave_registered = YES;
                }
            }
            watchdog_reset();       // this slot can be used for other housekeeping
            interval_count = 1;     // was there an interrupt generated by resetting the clock?
            break;

            default:
            // This only happens during the registered interval
            if (interval_count == this_unit)
            {   //temp = coax_short_message(interval_count);
                if ( (receive_message())                      // receive a message and there was no error
                    && (received_command == STATUS) )  // and the reply is what was expected
                         temp = coax_short_message(READY);
                else
                {
                    slave_registered = NO;      // otherwise give up and re-register
                    this_unit = DEFAULT_SLAVE;  // un registered units are all 0xFF
                    synch_slave();              // and get in synch with what is happening
                }
            }
            break;
        }
// This is where everything should happen after synchs and handshakes, like data readings
        collect_data();
    }
}


/////////////////  master() -- the main routine for MASTER devices ////////////////////////////////
// sends commands to the smart sensor and controls reporting via
// Janus Plug-in terminus boards
void master()
{   unsigned char unable_to_register_count=0;
    master_settings();                  // I/O settings for master board
    common_startup();               // Configuration settings common to both MASTER and SLAVE
    COAX_POWER_DISABLE;             // Turn off power to the sensors.  This is to insure that they didn't
                                    // end up in an unstable state during power up or brown out.
    this_unit = DEFAULT_MASTER;     // set our address
    clear_slots();                  // reregister all slots on start-up
    PHONE_POWER_DISABLE;            // Make sure the phone is off, while we are not using the phone:
    PHONE_ON_OFF_REQUEST;           // Set the phone's on/off pin to low (Turns off LED in prototype)
    PHONE_RESET_REQUEST;            // Set the phone reset pin to low
    phone_UART_off();               // start with the phone UART deactivated.

    // Check for sufficient battery power to start up
    // SHUT_OFF_LEVEL should be sufficient to power up the uP, but not enough for a 2maH phone call.
//    while ((battery_level = readBattery()) < START_UP_LEVEL)
//    {   wait_a_sec(300);                 // 5 minutes in the sun (sleeping for 300 seconds) should be enough to get going.
//        watchdog_reset;
//    }

    //    blink LEDs for half a second to indicate startup.
    waiting = NO;
    RED_LED_ON;
    GREEN_LED_ON;
    wait(512);                      // wait a half second
    RED_LED_OFF;
    GREEN_LED_OFF;
    wait_a_sec(5);                  // allow the sensors 5 sec to reset if a power glitch
    COAX_POWER_ENABLE;              // Turn on the sensors
    wait_a_sec(1);                  // let the sensors get going
    interval_count = interval_limit - 2;  // get the timer ready to synch up
    timeout_during_rx = YES;        // the master doesn't synch up
    intervals_on = YES;             // start with the phone off
    data_ready = new_census = NO;
    time_to_wakeup = NO;
    device_count = new_device_count = 0;
    if (recCount) send_old_data();  // send old data if there is any after WD reset
    while(TRUE)                     // loop forever
    {   ENTER_LOW_POWER_MODE_3;     // wait until the next 1/8 sec clock interval
        coax_reset();               // get rid of any UART garbage
        if (interval_count == 0)                         // interval zero is always just for resynching
        {       going_to_unit = ALL_UNITS;          // everyone synchs their clock
                if (coax_short_message(SYNCH) )   //send the message
                    blink(1);              // blink every 6 seconds
                GREEN_LED_OFF;
                      //TODO// does there need to be any error checking for tx?
                      //TODO// This interval can also be used for commands going to all sensors
                            // because they are all already listening and synched.
        }
        else if (interval_count == 1)   // this is the time slot for finding newly connected sensors (one per cycle)
        {   going_to_unit = ALL_UNITS;
            if (new_census)             // if there was any change to the list of devices connected
            {   coax_short_message(SUSPEND);    // tells the sensors that they should not expect a handshake
                intervals_on = NO;              // no timing interupts for now
                phone_on();                     // turn on the phone
                report_in();                    // update status logs
                time_to_wakeup = YES;           // this flags that suspend is off.
                intervals_on = YES;
                new_census = NO;
                new_device_count = 0;           // reset the counter of new devices
                wait_out_cycle();               // wait until the last interval
            }
            else if (data_ready)       //
            {   coax_short_message(SUSPEND);
                phone_on();
                // wake up the units with data one by one
                // upload the data
                //TODO// the rest of sending a phone call
                phone_in();         // send in a report for each sensor
                time_to_wakeup = YES;
                intervals_on = YES;
                wait_out_cycle();
            }
            else if (time_to_wakeup)
            {   coax_short_message(WAKE_UP);
                time_to_wakeup = NO;
            }
            else if (register_device())
            {   GREEN_LED_ON;
                new_device_count ++;
                unable_to_register_count = 0;
            }
            else if ( (++unable_to_register_count >5) && (new_device_count))
                new_census = YES; // flag that we are done registering new devices

  //TODO// flag to report new connections and send after there is no response
  // All phone communications are set up during this interval.  All the sensors are put into suspend mode,
  // meaning that they listen for commands during interval one and don't disconnect if there isn't
  // a handshake during their own interval. The sensors all upload data during the 6 second cycle, which is more than
  // time to upload or download 64K of code or data (probably won't need close to that)
  // All the sensors keep synching during interval 0 and listen during slot one to see which sensor will be activated.
  // 10 sensors a minute can be processed.  Once the cell call is complete the master releases the suspend command
  // during interval one with a global wake-up command.
  // All devices report their data if any one of them is ready to report.  Therefore, the device that is set to report
  // most frequently will determine the frequency that all devices report.

        }
        else if (device_count)       // if devices are connectd
        {   if (interval_count > 1)    // this is the default for all connected sensors.  It checks that they are still connected
            {                          // and deregisters them if they do not respond to a status inquiry
                if (sensor_slot[interval_count] == YES)  // only check on an slot if a slave has been registered there (= 1)
                {   going_to_unit = interval_count;    // set the recipient to match the interval
                    if (tx_rx(STATUS))                  // If we are successful sending a status message and receiving a reply
                    {   GREEN_LED_OFF;                  // The Green LED will blink once when a sensor is connected and registerred
                        if (master_routines())          // decide what to do about the sensor's status
                            coax_short_message(SIGN_OFF);   // All is good, send a SIGN_OUT message
                    }
                    if (tx_error == TIMEOUT_ERROR)
                    {   sensor_slot[going_to_unit]= NO; // deregisiter whatever sensor was there
                        sensor_status[going_to_unit]= RECENTLY_DISCONNECTED;
                        device_count--;                 // one less device
                        tx_error = NO_ERROR;            // if there was no sensor a timeout error is expected
                        RED_LED_ON;                     // long red led blink indicates disconnection
                     }
                }
                else tx_error = NO_ERROR;
            }
           if (tx_error) blink(24);     // a longer blink for errors
           if (!(interval_count & 0x0F)) blink(1);  //blink every 2 seconds (every 16 loops)
        }
    }
}



/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// MAIN ROUTINE ////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void main(void)
{   Grace_init();                       // Activate Grace-generated configuration (default settings for the MASTER)
    if (not_slave)
       master() ;   // Additional configuration settings for the MASTER
    else slave();               // Configuration settings that are different in the SLAVE
    while (TRUE);                        // Main loop -- never stops
}
