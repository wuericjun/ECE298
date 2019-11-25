#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include "Board.h"
#include "string.h"
#include "stdlib.h"

#define INTERVAL_TIME 32768

char currentTime[4] = {0};
int armTimes[4] = {-1, -1,-1,-1};
int disarmTimes[4] = {-1, -1,-1,-1};

char ADCState = 0;
int16_t noiseLevel[3] = {0};
int16_t noiseLevelAvg = 0;

int rs2State = 0;
int rs3State = 0;
int rs4State = 0;

int zonenum = 0;

int alarm = 0;

zoneState_t zoneStates[4] = {zoneOk_e, zoneOk_e, zoneOk_e, zoneOk_e};

void incrementTimer();

void updateNoiseLevel(int16_t level) {
    /*
     * Formula:
     *   y = (x1 + 4 * x2 + x3)/3
     */
    int i;
    for (i = 0; i < 2; i++) {
        noiseLevel[i + 1] = noiseLevel[i];
    } // for
    noiseLevel[0] = level;
    noiseLevelAvg = (noiseLevel[0] + (4 * noiseLevel[1]) + noiseLevel[2]) / 3;
}


// Convert current time from 4 decimal digits to an int.
int getCurrentTimeInt() {
    int sum = 0;
    sum += currentTime[3];
    sum += 10 * currentTime[2];
    sum += 100 * currentTime[1];
    sum += 1000 * currentTime[0];
    return sum;
}

void setLEDsOk(void *zonenum) {
    if(zonenum == 1){
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); //green
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5); //red
        }
    if(zonenum == 2){
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); //green
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); //red
        }
    if(zonenum == 3){
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6); //green
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3); //red
           }
    if(zonenum == 4){
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7); //green
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3); //red
           }
}
void setLEDsArmed(void *zonenum) {
    if(zonenum == 1){
          GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2); //green
          GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5); //red
          }
      if(zonenum == 2){
          GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); //green
          GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); //red
          }
      if(zonenum == 3){
          GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6); //green
          GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3); //red
                }
      if(zonenum == 4){
          GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7); //green
          GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3); //red
                }
}


void setLEDsNotOk(void *zonenum) {
    if(zonenum == 1){
          GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); //green
          GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5); //red
          }
      if(zonenum == 2){
          GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); //green
          GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); //red
          }
      if(zonenum == 3){
          GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6); //green
          GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3); //red
                }
      if(zonenum == 4){
          GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7); //green
          GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3); //red
                }
}

/*
 * This project contains some code samples that may be useful.
 *
 */

int clockCount = 0;

void main(void)
{
    char buttonState = 0; //Current button press state (to allow edge detection)


    /*
     * Functions with two underscores in front are called compiler intrinsics.
     * They are documented in the compiler user guide, not the IDE or MCU guides.
     * They are a shortcut to insert some assembly code that is not really
     * expressible in plain C/C++. Google "MSP430 Optimizing C/C++ Compiler
     * v18.12.0.LTS" and search for the word "intrinsic" if you want to know
     * more.
     * */

    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    //Init_PWM();     //Sets up a PWM output
    Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display

    setLEDsOk(1);
    setLEDsOk(2);
    setLEDsOk(3);
    setLEDsOk(4);
     /*
     * The MSP430 MCUs have a variety of low power modes. They can be almost
     * completely off and turn back on only when an interrupt occurs. You can
     * look up the power modes in the Family User Guide under the Power Management
     * Module (PMM) section. You can see the available API calls in the DriverLib
     * user guide, or see "pmm.h" in the driverlib directory. Unless you
     * purposefully want to play with the power modes, just leave this command in.
     */
    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    __enable_interrupt();

    //displayScrollText("ECE 298");
    clearLCD();

    while (1) {
        rs2State = GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN1); // rs2
        rs3State = GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2); // rs3
        rs4State = GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7); // rs4
        if (zoneStates[1] == zoneArmed_e && rs2State == 1) { // rs2
                  zoneStates[1] = zoneNotOk_e;
                  setLEDsNotOk(2);
                  alarm = 1;
              } // if
        if (zoneStates[2] == zoneArmed_e && rs3State == 1) { // rs3
                         zoneStates[2] = zoneNotOk_e;
                         setLEDsNotOk(3);
                         alarm = 1;
                     } // if
        if (zoneStates[3] == zoneArmed_e && rs4State == 1) { // rs4
                         zoneStates[3] = zoneNotOk_e;
                         setLEDsNotOk(4);
                         alarm = 1;
                             } // if
        if (zoneNotOk_e == zoneStates[0] || zoneNotOk_e == zoneStates[1] || zoneNotOk_e == zoneStates[2] || zoneNotOk_e == zoneStates[3] ) {
            GPIO_toggleOutputOnPin(GPIO_PORT_P8, GPIO_PIN3);
            __delay_cycles(10);
        } // if

        //Start an ADC conversion (if it's not busy) in Single-Channel, Single Conversion Mode
        if (ADCState == 0)
        {
            ADCState = 1; //Set flag to indicate ADC is busy - ADC ISR (interrupt) will clear it
            ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
        }
    } // while


//     //Enter LPM0 - interrupts only
//     __bis_SR_register(LPM0_bits);
//     //For debugger to let it know that you meant for there to be no more code
//     __no_operation();


}

void Init_GPIO() {
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN2);
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN3);

    GPIO_setAsInputPin(
        GPIO_PORT_P2,
        GPIO_PIN5
        );

    //P2.5 interrupt enabled
    GPIO_enableInterrupt(
        GPIO_PORT_P2,
        GPIO_PIN5
        );

    //P2.5 Hi/Lo edge
    GPIO_selectInterruptEdge(
        GPIO_PORT_P2,
        GPIO_PIN5,
        GPIO_LOW_TO_HIGH_TRANSITION
        );


    //P2.5 IFG cleared
    GPIO_clearInterrupt(
        GPIO_PORT_P2,
        GPIO_PIN5
        );

    // P1.2 interrupt enabled
    GPIO_setAsInputPinWithPullUpResistor(
        GPIO_PORT_P1,
        GPIO_PIN2
        );

    //P1.2 interrupt enabled
    GPIO_enableInterrupt(
        GPIO_PORT_P1,
        GPIO_PIN2
        );

    //P1.2 Hi/Lo edge
    GPIO_selectInterruptEdge(
        GPIO_PORT_P1,
        GPIO_PIN2,
        GPIO_HIGH_TO_LOW_TRANSITION
        );


    //P1.2 IFG cleared
    GPIO_clearInterrupt(
        GPIO_PORT_P1,
        GPIO_PIN2
        );
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    //GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    //CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    //CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    //CS_turnOnXT1LF(CS_XT1_DRIVE_1);

        //Set DCO FLL reference = REFO
    CS_initClockSignal(
        CS_FLLREF,
        CS_REFOCLK_SELECT,
        CS_CLOCK_DIVIDER_1
        );

    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);


        //Initialize RTC
    RTC_init(RTC_BASE,
        INTERVAL_TIME,
        RTC_CLOCKPREDIVIDER_1);

    RTC_clearInterrupt(RTC_BASE,
        RTC_OVERFLOW_INTERRUPT_FLAG);

    //Enable interrupt for RTC overflow
    RTC_enableInterrupt(RTC_BASE,
        RTC_OVERFLOW_INTERRUPT);

    //Start RTC Clock with clock source SMCLK
    RTC_start(RTC_BASE, RTC_CLOCKSOURCE_SMCLK);
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(RTC_VECTOR)))
#endif
void RTC_ISR (void)
{
    switch (__even_in_range(RTCIV,2)){
        case 0: break;  //No interrupts
        case 2:         //RTC overflow
            clockCount++;
            if (clockCount == 32) {
                incrementTimer();
                clockCount = 0;
                if (armTimes[0] == getCurrentTimeInt() && alarm == 0) {
                    zoneStates[0] = zoneArmed_e;
                    setLEDsArmed(1);
                } // if
                if (armTimes[1] == getCurrentTimeInt() && alarm == 0) {
                    zoneStates[1] = zoneArmed_e;
                    setLEDsArmed(2);
                                } // if
                if (armTimes[2] == getCurrentTimeInt() && alarm == 0) {
                    zoneStates[2] = zoneArmed_e;
                    setLEDsArmed(3);
                                                } // if
                if (armTimes[3] == getCurrentTimeInt() && alarm == 0) {
                     zoneStates[3] = zoneArmed_e;
                     setLEDsArmed(4);
                                                } // if
                if (disarmTimes[0] == getCurrentTimeInt() && alarm == 0) {
                    zoneStates[0] = zoneOk_e;
                    setLEDsOk(1);
                } // if
                if (disarmTimes[1] == getCurrentTimeInt() && alarm == 0) {
                    zoneStates[1] = zoneOk_e;
                    setLEDsOk(2);
                            } // if
                if (disarmTimes[2] == getCurrentTimeInt() && alarm == 0) {
                    zoneStates[2] = zoneOk_e;
                    setLEDsOk(3);
                                } // if
                if (disarmTimes[3] == getCurrentTimeInt() && alarm == 0) {
                    zoneStates[3] = zoneOk_e;
                    setLEDsOk(4);
                                } // if
            } // if
            break;
        default: break;
    }
}

void incrementTimer() {
    currentTime[3]++;

    int i;
    for (i = 3; i > 0; i--) {
        if (currentTime[i] == 10) {
            currentTime[i - 1]++;
            currentTime[i] = 0;
        } // if
    } // for

    showChar(currentTime[0] + '0', pos3);
    showChar(currentTime[1] + '0', pos4);
    showChar(currentTime[2] + '0', pos5);
    showChar(currentTime[3] + '0', pos6);
}

/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

char command[50] = {0};
int commandPos = 0;

// Use this inside EUSCIA0_ISR
inline void transmitStr(char* str) {
    int i;
    for (i = 0; i < strlen(str); i++) {
        __delay_cycles(2000);
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, str[i]);
    } // for
}

// Use this inside EUSCIA0_ISR to transmit 4 decimal digits
inline void transmitDec(int dec) {
    if (dec > 9999) { return; }
    int remainder = dec;
    char printNum[4] = {'0', '0', '0', '0'};
    while (remainder >= 1000) {
        remainder -= 1000;
        printNum[0]++;
    } // while
    __delay_cycles(2000);
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, printNum[0]);
    while (remainder >= 100) {
        remainder -= 100;
        printNum[1]++;
    } // while
    __delay_cycles(2000);
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, printNum[1]);
    while (remainder >= 10) {
        remainder -= 10;
        printNum[2]++;
    } // while
    __delay_cycles(2000);
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, printNum[2]);
    while (remainder >= 1) {
        remainder -= 1;
        printNum[3]++;
    } // while
    __delay_cycles(2000);
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, printNum[3]);
}

inline void printLineBreak() {
    __delay_cycles(2000);
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
    __delay_cycles(2000);
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        char recv = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);


        if ('\'' == recv) {
            printLineBreak();
            if (0 == strcmp(command, "stat")) {
                transmitStr("Time: ");
                __delay_cycles(2000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, currentTime[0] + '0');
                __delay_cycles(2000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, currentTime[1] + '0');
                __delay_cycles(2000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, currentTime[2] + '0');
                __delay_cycles(2000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, currentTime[3] + '0');
                printLineBreak();
                if (zoneStates[0] == zoneOk_e) { transmitStr("Zone 1 OK"); }
                else if (zoneStates[0] == zoneArmed_e) { transmitStr("Zone 1 Armed"); }
                else if (zoneStates[0] == zoneNotOk_e) { transmitStr("Zone 1 Not Ok"); }
                else if (zoneStates[0] == zoneOk_e) { transmitStr("Zone 1 Ok"); }
                printLineBreak();
                if (zoneStates[1] == zoneOk_e) { transmitStr("Zone 2 OK"); }
                else if (zoneStates[1] == zoneArmed_e) { transmitStr("Zone 2 Armed"); }
                else if (zoneStates[1] == zoneNotOk_e) { transmitStr("Zone 2 Not Ok"); }
                else if (zoneStates[1] == zoneOk_e) { transmitStr("Zone 2 Ok"); }
                printLineBreak();
                if (zoneStates[2] == zoneOk_e) { transmitStr("Zone 3 OK"); }
                else if (zoneStates[2] == zoneArmed_e) { transmitStr("Zone 3 Armed"); }
                else if (zoneStates[2] == zoneNotOk_e) { transmitStr("Zone 3 Not Ok"); }
                else if (zoneStates[2] == zoneOk_e) { transmitStr("Zone 3 Ok"); }
                printLineBreak();
                if (zoneStates[3] == zoneOk_e) { transmitStr("Zone 4 OK"); }
                else if (zoneStates[3] == zoneArmed_e) { transmitStr("Zone 4 Armed"); }
                else if (zoneStates[3] == zoneNotOk_e) { transmitStr("Zone 4 Not Ok"); }
                else if (zoneStates[3] == zoneOk_e) { transmitStr("Zone 4 Ok"); }
                printLineBreak();
                transmitDec((int)noiseLevelAvg);
            } // if
            else if (0 == strncmp(command, "atimeall", strlen("atimeall"))) {
                armTimes[0] = atoi(command + strlen("atimeall") + 1);
                armTimes[1] = atoi(command + strlen("atimeall") + 1);
                armTimes[2] = atoi(command + strlen("atimeall") + 1);
                armTimes[3] = atoi(command + strlen("atimeall") + 1);
                transmitStr("Zone1 Arm time: ");
                transmitDec(armTimes[0]);
                printLineBreak();
                transmitStr("Zone2 Arm time: ");
                transmitDec(armTimes[1]);
                printLineBreak();
                transmitStr("Zone3 Arm time: ");
                transmitDec(armTimes[2]);
                printLineBreak();
                transmitStr("Zone4 Arm time: ");
                transmitDec(armTimes[3]);
            } // else if
            else if (0 == strncmp(command, "atime1", strlen("atime1"))) {
                armTimes[0] = atoi(command + strlen("atime1") + 1);
                transmitStr("Zone1 arm time: ");
                transmitDec(armTimes[0]);
                printLineBreak();
            } // else if
            else if (0 == strncmp(command, "atime2", strlen("atime2"))) {
                armTimes[1] = atoi(command + strlen("atime2") + 1);
                transmitStr("Zone2 arm time: ");
                transmitDec(armTimes[1]);
                printLineBreak();
            } // else if
            else if (0 == strncmp(command, "atime3", strlen("atime3"))) {
                armTimes[2] = atoi(command + strlen("atime3") + 1);
                transmitStr("Zone3 arm time: ");
                transmitDec(armTimes[2]);
                printLineBreak();
            } // else if
            else if (0 == strncmp(command, "atime4", strlen("atime4"))) {
                armTimes[3] = atoi(command + strlen("atime4") + 1);
                transmitStr("Zone3 arm time: ");
                transmitDec(armTimes[3]);
                printLineBreak();
            } // else if
            else if (0 == strncmp(command, "dtimeall", strlen("dtimeall"))) {
                disarmTimes[0] = atoi(command + strlen("dtimeall") + 1);
                disarmTimes[1] = atoi(command + strlen("dtimeall") + 1);
                disarmTimes[2] = atoi(command + strlen("dtimeall") + 1);
                disarmTimes[3] = atoi(command + strlen("dtimeall") + 1);
                transmitStr("Zone1 disarm time: ");
                transmitDec(disarmTimes[0]);
                printLineBreak();
                transmitStr("Zone2 disarm time: ");
                transmitDec(disarmTimes[1]);
                printLineBreak();
                transmitStr("Zone3 disarm time: ");
                transmitDec(disarmTimes[2]);
                printLineBreak();
                transmitStr("Zone4 disarm time: ");
                transmitDec(disarmTimes[3]);
                        } // else i
            else if (0 == strncmp(command, "dtime1", strlen("dtime1"))) {
                disarmTimes[0] = atoi(command + strlen("dtime1") + 1);
                transmitStr("Zone1 Disarm time: ");
                transmitDec(disarmTimes[0]);
                printLineBreak();
            } // else if
            else if (0 == strncmp(command, "dtime2", strlen("dtime2"))) {
                disarmTimes[1] = atoi(command + strlen("dtime2") + 1);
                transmitStr("Zone2 Disarm time: ");
                transmitDec(disarmTimes[1]);
                printLineBreak();
            } // else if
            else if (0 == strncmp(command, "dtime3", strlen("dtime3"))) {
                disarmTimes[2] = atoi(command + strlen("dtime3") + 1);
                transmitStr("Zone3 Disarm time: ");
                transmitDec(disarmTimes[2]);
                printLineBreak();
            } // else if
            else if (0 == strncmp(command, "dtime4", strlen("dtime4"))) {
                disarmTimes[3] = atoi(command + strlen("dtime4") + 1);
                transmitStr("Zone4 Disarm time: ");
                transmitDec(disarmTimes[3]);
                printLineBreak();
                       } // else if
            else if (0 == strcmp(command, "arm1")) {
                transmitStr("Zone 1 armed.");
                zoneStates[0] = zoneArmed_e;
                setLEDsArmed(1);
            } // else if
            else if (0 == strcmp(command, "arm2")) {
                transmitStr("Zone 2 armed.");
                zoneStates[1] = zoneArmed_e;
                setLEDsArmed(2);
            } // else if
            else if (0 == strcmp(command, "arm3")) {
                transmitStr("Zone 3 armed.");
                zoneStates[2] = zoneArmed_e;
                setLEDsArmed(3);
            } // else if
            else if (0 == strcmp(command, "arm4")) {
                 transmitStr("Zone 4 armed.");
                 zoneStates[3] = zoneArmed_e;
                 setLEDsArmed(4);
                        } // else if
            else if (0 == strcmp(command, "disarm1")) {
                transmitStr("Zone 1 disarmed.");
                zoneStates[0] = zoneOk_e;
                setLEDsOk(1);
            } // else if
            else if (0 == strcmp(command, "disarm2")) {
                transmitStr("Zone 2 disarmed.");
                zoneStates[1] = zoneOk_e;
                setLEDsOk(2);
            } // else if
            else if (0 == strcmp(command, "disarm3")) {
                transmitStr("Zone 3 disarmed.");
                zoneStates[2] = zoneOk_e;
                setLEDsOk(3);
            } // else if
            else if (0 == strcmp(command, "disarm4")) {
                 transmitStr("Zone 4 disarmed.");
                 zoneStates[3] = zoneOk_e;
                 setLEDsOk(4);
            } // else if
            else {
                transmitStr("Invalid command.");
            } // else
            printLineBreak();
            __delay_cycles(2000);
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '$');
            __delay_cycles(2000);
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            commandPos = 0;
            memset(command, 0, 50);
        } // if
        else {
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, recv);
            command[commandPos] = recv;
            commandPos++;
        } // else


    } // if
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT2_VECTOR)))
#endif
void P2_ISR (void)
{
    if (GPIO_PIN5 == GPIO_getInterruptStatus(GPIO_PORT_P2, GPIO_PIN5)) { // rs1
        if (zoneStates[0] == zoneArmed_e) {
            setLEDsNotOk(1);
            alarm = 1;
            zoneStates[0] = zoneNotOk_e;
        } // if
                }
        //P2.5 IFG cleared
        GPIO_clearInterrupt(
            GPIO_PORT_P2,
            GPIO_PIN5
            );
    } // if


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT1_VECTOR)))
#endif
void P1_ISR (void)
{
    if (GPIO_PIN2 == GPIO_getInterruptStatus(GPIO_PORT_P1, GPIO_PIN2)) { // override pb
        if (zoneStates[0] == zoneNotOk_e) {
            alarm = 0;
            zoneStates[0] = zoneOk_e;
            setLEDsOk(1);
        } // if
        if (zoneStates[1] == zoneNotOk_e) {
                   alarm = 0;
                   zoneStates[1] = zoneOk_e;
                   setLEDsOk(2);
               }
        if (zoneStates[2] == zoneNotOk_e) {
                    alarm = 0;
                    zoneStates[2] = zoneOk_e;
                    setLEDsOk(3);
               }
        if (zoneStates[3] == zoneNotOk_e) {
                    alarm = 0;
                    zoneStates[3] = zoneOk_e;
                    setLEDsOk(4);
                }
        //P1.2 IFG cleared
        GPIO_clearInterrupt(
            GPIO_PORT_P1,
            GPIO_PIN2
            );
    } // if
}



void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    //GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        int16_t level = ADC_getResults(ADC_BASE);
        updateNoiseLevel(level);
        if (zoneStates[0] == zoneArmed_e) {
            if (level > noiseLevelAvg + 50) {
                alarm = 1;
                zoneStates[0] = zoneNotOk_e;
                setLEDsNotOk(1);
            } // if
        } // if

    }
}











