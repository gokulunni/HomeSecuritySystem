/* DESCRIPTION
 * Sample code showing keypad's response to pushing buttons 1 and 2
 * Include pressedKey on debugger's "Expressions" to see the hexaKeys' value when you alternate between the two keys
 * Did not include button debouncer in this (releasing the button does not set pressedKey back to Value 0 '\x00')
 */


#include <msp430.h>
#include "driverlib.h"
#include "Board.h"
#include "hal_LCD.h"
#include "code_shell.h"

//Buzzer


//ADC
#define TIMER_PERIOD 511
int DUTY_CYCLE = 0;

char hexaKeys[4][3] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

void Key();
void GetADCReading(int zone);
void buzzer_on(void);
int zone1=0;
int zone2=0;
int zone3=0;
int zone4=0;
int zoneBeingRead;

int toggleMode=1;
int password =0;
int realPassword=1234;


char pressedKey;
int global_state = 0;

Timer_A_outputPWMParam param;

void Init_Buzzer(void);


void main (void)
{




    WDT_A_hold(WDT_A_BASE);     // Stop watchdog timer

    ////
    //Port select XT1
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P4,
        GPIO_PIN1 + GPIO_PIN2,
        GPIO_PRIMARY_MODULE_FUNCTION
        );

    //Set external frequency for XT1
    CS_setExternalClockSource(32768);

    //Select XT1 as the clock source for ACLK with no frequency divider
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);

    //Start XT1 with no time out
    CS_turnOnXT1(CS_XT1_DRIVE_0);

    //clear all OSC fault flag
    CS_clearAllOscFlagsWithTimeout(1000);

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();

    // L0~L26 & L36~L39 pins selected
    LCD_E_setPinAsLCDFunctionEx(LCD_E_BASE, LCD_E_SEGMENT_LINE_0, LCD_E_SEGMENT_LINE_26);
    LCD_E_setPinAsLCDFunctionEx(LCD_E_BASE, LCD_E_SEGMENT_LINE_36, LCD_E_SEGMENT_LINE_39);

    LCD_E_initParam initParams = LCD_E_INIT_PARAM;
    initParams.clockDivider = LCD_E_CLOCKDIVIDER_8;
    initParams.muxRate = LCD_E_4_MUX;
    initParams.segments = LCD_E_SEGMENTS_ENABLED;

    // Init LCD as 4-mux mode
    LCD_E_init(LCD_E_BASE, &initParams);

    // LCD Operation - Mode 3, internal 3.08v, charge pump 256Hz
    LCD_E_setVLCDSource(LCD_E_BASE, LCD_E_INTERNAL_REFERENCE_VOLTAGE, LCD_E_EXTERNAL_SUPPLY_VOLTAGE);
    LCD_E_setVLCDVoltage(LCD_E_BASE, LCD_E_REFERENCE_VOLTAGE_3_08V);

    LCD_E_enableChargePump(LCD_E_BASE);
    LCD_E_setChargePumpFreq(LCD_E_BASE, LCD_E_CHARGEPUMP_FREQ_16);

    // Clear LCD memory
    LCD_E_clearAllMemory(LCD_E_BASE);

    // Configure COMs and SEGs
    // L0, L1, L2, L3: COM pins
    // L0 = COM0, L1 = COM1, L2 = COM2, L3 = COM3
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_0, LCD_E_MEMORY_COM0);
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_1, LCD_E_MEMORY_COM1);
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_2, LCD_E_MEMORY_COM2);
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_3, LCD_E_MEMORY_COM3);

   // LCD_E_setMemory(LCD_E_BASE,LCD_E_MEMORY_BLINKINGMEMORY_18, 0xEF);

    //showChar('P',1);
    //LCD_E_on(LCD_E_BASE);

    Init_LCD();
    //displayScrollText("ENTER PIN");
    showChar('P',4);
    showChar('I',6);
    showChar('N',8);


    /////



/*
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2);                   //set zone 1 source high
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);


    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN2);                   //set zone 2 source high
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);


    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3);                   //set zone 3 source high
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);

*/

    //GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN3);                   //set zone 4 source high
    //GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);

    // ROWS ARE OUTPUTS

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN4);                  // Row 1: Output direction
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);

    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);                  // Row 2: Output direction
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);

    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);                  // Row 3: Output direction
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);                  // Row 4: Output direction
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);

    // COLUMNS ARE ISR TRIGGERS

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);     // Column 1: Input direction
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN5);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);     // Column 2: Input direction
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN6);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);     // Column 3: Input direction
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN3, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN3);


    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);     // zone 1 read: Input direction
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);



    _EINT();        // Start interrupt


    PMM_unlockLPM5();           // Need this for LED to turn on- in case of "abnormal off state"

    while(true)
    {
        GetADCReading(1);
        GetADCReading(2);
        GetADCReading(3);
        GetADCReading(4);
    }

    __bis_SR_register(LPM4_bits + GIE);     // Need this for interrupts or else "abnormal termination"
    __no_operation();           //For debugger



}

void Screen( int key_press){

    int state = global_state;
    if(key_press == 10){
        state = -1;
        password = 0;
        clearLCD();

    }

    switch(state){
    case -1:
        showChar('P',4);
        showChar('I',6);
        showChar('N',8);
        global_state=0;
        password =0;
        break;

    case 0:
        displayScrollText("ENTER PIN");
        global_state = 1;
        password =0;
        break;

    case 1:
        if(key_press == 11){
            if(password==realPassword){
                global_state = 3;
                showChar('1',4);
                showChar('2',6);
                showChar('3',8);
                showChar('4',10);
                password=0;
            }
            else{
                global_state = 1;
                displayScrollText("WRONG PASSWORD");
            }
            password = 0;

        }
        else{
            if(password <1000){
                password = password*10 + key_press;
            }
            clearLCD();
            showChar('X',4);
            if(password >= 10){
                showChar('X',6);
            }
            if(password >= 100){
                showChar('X',8);
            }
            if(password >= 1000){
                showChar('X',10);
            }

        }
        break;

    case 2:
        //show x for every pin entered
        // if enter check pin if right state = 4 else state =3
        //if back remove pin
        break;

    case 3:
        if(key_press == 1){
            displayScrollText("ON PRESS 1 OFF PRESS 2");
            clearLCD();
            if(zone1 == 0){

                showChar('O',4);
                showChar('F',6);
                showChar('F',8);
            }
            if(zone1 == 1){

                showChar('O',4);
                showChar('N',6);
            }
            global_state = 4;
        }
        if(key_press == 2){
            displayScrollText("ON PRESS 1 OFF PRESS 2");
            clearLCD();
            if(zone2 == 0){

                showChar('O',4);
                showChar('F',6);
                showChar('F',8);
            }
            if(zone2 == 1){

                showChar('O',4);
                showChar('N',6);
            }
            global_state = 5;
        }
        if(key_press == 3){
            displayScrollText("ON PRESS 1 OFF PRESS 2");
            clearLCD();
            if(zone3 == 0){

                showChar('O',4);
                showChar('F',6);
                showChar('F',8);
            }
            if(zone3 == 1){

                showChar('O',4);
                showChar('N',6);
            }
            global_state = 6;
        }
        if(key_press == 4){
            displayScrollText("ON PRESS 1 OFF PRESS 2");
            clearLCD();
            if(zone4 == 0){

                showChar('O',4);
                showChar('F',6);
                showChar('F',8);
            }
            if(zone4 == 1){

                showChar('O',4);
                showChar('N',6);
            }
            global_state = 7;
        }
        break;

    case 4:
        clearLCD();
        if(key_press == 1){
            zone1 = 1;
        }
        if(key_press == 2){
            zone1 = 0;
        }
        if(zone1 == 0){

            showChar('O',4);
            showChar('F',6);
            showChar('F',8);
        }
        if(zone1 == 1){

            showChar('O',4);
            showChar('N',6);
        }
        if(key_press == 11){
            global_state = 3;
            showChar('1',4);
            showChar('2',6);
            showChar('3',8);
            showChar('4',10);
        }
        break;
    case 5:
        clearLCD();
        if(key_press == 1){
            zone2 = 1;
        }
        if(key_press == 2){
            zone2 = 0;
        }
        if(zone2 == 0){

            showChar('O',4);
            showChar('F',6);
            showChar('F',8);
        }
        if(zone2 == 1){

            showChar('O',4);
            showChar('N',6);
        }
        if(key_press == 11){
            global_state = 3;
            showChar('1',4);
            showChar('2',6);
            showChar('3',8);
            showChar('4',10);
        }
        break;
    case 6:
        clearLCD();
        if(key_press == 1){
            zone3 = 1;
        }
        if(key_press == 2){
            zone3 = 0;
        }
        if(zone3 == 0){

            showChar('O',4);
            showChar('F',6);
            showChar('F',8);
        }
        if(zone3 == 1){

            showChar('O',4);
            showChar('N',6);
        }
        if(key_press == 11){
            global_state = 3;
            showChar('1',4);
            showChar('2',6);
            showChar('3',8);
            showChar('4',10);
        }
        break;
    case 7:
        clearLCD();
        if(key_press == 1){
            zone4 = 1;
        }
        if(key_press == 2){
            zone4 = 0;
        }
        if(zone4 == 0){

            showChar('O',4);
            showChar('F',6);
            showChar('F',8);
        }
        if(zone4 == 1){

            showChar('O',4);
            showChar('N',6);
        }
        if(key_press == 11){
            global_state = 3;
            showChar('1',4);
            showChar('2',6);
            showChar('3',8);
            showChar('4',10);
        }
        break;

    default:
        //
        break;
    }
}


void Init_Buzzer(void)
{
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //1000
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //500


    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
}

void buzzer_on(void)
{
    //Stop WDT
       WDT_A_hold(WDT_A_BASE);
       __disable_interrupt();

       Init_Buzzer();

       PMM_unlockLPM5();

       Timer_A_outputPWM(TIMER_A0_BASE, &param); //Turn on PWM
}

void GetADCReading(int zone)
{
    //Defining Zone4R (P8_PIN0):
    #define GPIO_PORT_ADC8          GPIO_PORT_P8
    #define GPIO_PIN_ADC0           GPIO_PIN0
    #define GPIO_FUNCTION_ADC8       GPIO_PRIMARY_MODULE_FUNCTION
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_ADC8,
            GPIO_PIN_ADC0,
            GPIO_FUNCTION_ADC8);

   //Defining Zone2R (P8_PIN1):
   #define GPIO_PIN_ADC1           GPIO_PIN1
   GPIO_setAsPeripheralModuleFunctionInputPin(
           GPIO_PORT_ADC8,
           GPIO_PIN_ADC1,
           GPIO_FUNCTION_ADC8);

   //Defining Zone1R (P1_PIN1):
    #define GPIO_PORT_ADC1          GPIO_PORT_P1
    #define GPIO_PIN_ADC1           GPIO_PIN1
    #define GPIO_FUNCTION_ADC1      GPIO_PRIMARY_MODULE_FUNCTION
    GPIO_setAsPeripheralModuleFunctionInputPin(
              GPIO_PORT_ADC1,
              GPIO_PIN_ADC1,
              GPIO_FUNCTION_ADC1);

    //Defining Zone1R (P1_PIN0):
     GPIO_setAsPeripheralModuleFunctionInputPin(
               GPIO_PORT_ADC1,
               GPIO_PIN_ADC0,
               GPIO_FUNCTION_ADC1);

    PMM_enableInternalReference();
    while (PMM_REFGEN_NOTREADY == PMM_getVariableReferenceVoltageStatus());

    //ADC Initialization:
    ADC_init(ADC_BASE, ADC_SAMPLEHOLDSOURCE_SC, ADC_CLOCKSOURCE_ADCOSC, ADC_CLOCKDIVIDER_1);
    ADC_enable(ADC_BASE);
    ADC_setupSamplingTimer(ADC_BASE, ADC_CYCLEHOLD_16_CYCLES, ADC_MULTIPLESAMPLESDISABLE);
    ADC_clearInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT);
    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT);

    zoneBeingRead = zone;

    //while (true) {    //while loop is only for testing purposes

    //Get reading:
    if(zone == 1)
        ADC_configureMemory(ADC_BASE, ADC_INPUT_A1, ADC_VREFPOS_AVCC, ADC_VREFNEG_AVSS);  //Zone 1
    if(zone == 2)
        ADC_configureMemory(ADC_BASE, ADC_INPUT_A9, ADC_VREFPOS_AVCC, ADC_VREFNEG_AVSS);  //Zone 2
    if(zone == 3)
        ADC_configureMemory(ADC_BASE, ADC_INPUT_A0, ADC_VREFPOS_AVCC, ADC_VREFNEG_AVSS);  //Zone 3
    if(zone == 4)
        ADC_configureMemory(ADC_BASE, ADC_INPUT_A8, ADC_VREFPOS_AVCC, ADC_VREFNEG_AVSS);  //Zone 4

    //Delay between conversions
    __delay_cycles(5000);

    //Enable and Start the conversion
    //in Single-Channel, Single Conversion Mode
    ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
    // ADC_startConversion(ADC_BASE, ADC_SEQOFCHANNELS);

    //??? memory buffer saves only one inputsourceselect!!! Need to switch...
    // ADC_disableConversions(ADC_BASE, ADC_COMPLETECONVERSION);
    // while(ADC_isBusy(ADC_BASE)){
    //     __delay_cycles(1000);
    // }
    // ADC_stopConversions();

    //LPM0, ADC10_ISR will force exit
    __bis_SR_register(CPUOFF + GIE);
    //For debug only
    __no_operation();

    //}

}


//ISR FOR ADC:
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(ADC_VECTOR)))
#endif
void ADC_ISR (void)
{
    int16_t reading;
    int placeholder;
    switch (__even_in_range(ADCIV,12)){
        case  0: break; //No interrupt
        case  2: break; //conversion result overflow
        case  4: break; //conversion time overflow
        case  6: break; //ADC10HI
        case  8: break; //ADC10LO
        case 10: break; //ADC10IN
        case 12:        ////ADCIFG0 is ADC interrupt flag

            //(Automatically clears ADC10IFG0 by reading memory buffer)

            reading = ADC_getResults(ADC_BASE);
            placeholder = reading;
            if(reading <= 99)
            {

                if (zone2 == 1 && zoneBeingRead == 2) {        //zone 2 on
                    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN2);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);
                    buzzer_on();
                }

                else if(zone3 == 1 && zoneBeingRead == 3) {          //zone 3 on
                    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
                    buzzer_on();
                }

                else if(zone4 == 1 && zoneBeingRead == 4) {          //zone 4 on
                    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN3);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);
                    buzzer_on();
                }
            }

            else
            {
                if (zone1 == 1 && zoneBeingRead == 1 && reading <= 685) {              //zone 1 on
                    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
                    buzzer_on();
                }

                else if (zone1 == 1 && zoneBeingRead == 1) {              //zone 1 Off
                    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);
                    Timer_A_stop(TIMER_A0_BASE);
                }

                else if (zone2 == 1 && zoneBeingRead == 2) {        //zone 2 Off
                    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN2);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
                    Timer_A_stop(TIMER_A0_BASE);
                }

                else if(zone3 == 1 && zoneBeingRead == 3) {          //zone 3 Off
                    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
                    Timer_A_stop(TIMER_A0_BASE);
                }

                else if(zone4 == 1 && zoneBeingRead == 4) {          //zone 4 Off
                    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN3);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
                    Timer_A_stop(TIMER_A0_BASE);
                }
            }

          //Clear CPUOFF bit from 0(SR)
            //Breakpoint here and watch ADC_Result
          __bic_SR_register_on_exit(CPUOFF);
          break;
        default: break;
    }
}


void Key()
{



        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5); // Row 2- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- HIGH



        if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5) == GPIO_INPUT_PIN_LOW)){     // Column 1 to GND

            //LCD_E_setMemory(LCD_E_BASE,LCD_E_MEMORY_BLINKINGMEMORY_18, 0x60); //Display 1
            Screen( 1 );
        }

        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5); // Row 2- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- LOW

        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5); // Row 2- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- HIGH

        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == GPIO_INPUT_PIN_LOW) {    // Column 2

            Screen( 2 );
        }
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5); // Row 2- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- LOW



        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5); // Row 2- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- HIGH

        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == GPIO_INPUT_PIN_LOW) {    // Column 3
            pressedKey = hexaKeys[0][2];       // Shows "3"

            Screen( 3 );
        }
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5); // Row 2- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- LOW



        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- HIGH

        if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5) == GPIO_INPUT_PIN_LOW)  ){     // Column 1 to GND
           // pressedKey = hexaKeys[0][0];        // Shows "4"

            //showChar('4',2);
            Screen( 4 );

        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4  LOW

        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- HIGH

        if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == GPIO_INPUT_PIN_LOW)  ){     // Column 2 to GND
            pressedKey = hexaKeys[0][0];        // Shows "5"
            Screen( 5 );

        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4  LOW

        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- HIGH

        if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == GPIO_INPUT_PIN_LOW)  ){     // Column 3 to GND
            pressedKey = hexaKeys[0][0];        // Shows "6"
            Screen( 6 );

        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4  LOW

        //row 3
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- HIGH

        if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5) == GPIO_INPUT_PIN_LOW)  ){     // Column 1 to GND
            pressedKey = hexaKeys[0][0];        // Shows "7"
            Screen( 7 );

        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4  LOW

        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- HIGH

        if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == GPIO_INPUT_PIN_LOW)  ){     // Column 2 to GND
            pressedKey = hexaKeys[0][0];        // Shows "8"
            Screen( 8 );

        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4  LOW

        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- HIGH

        if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == GPIO_INPUT_PIN_LOW)  ){     // Column 3 to GND
            pressedKey = hexaKeys[0][0];        // Shows "9"
            Screen( 9 );

        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4  LOW


        //row 4
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- LOW

        if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5) == GPIO_INPUT_PIN_LOW)  ){     // Column 1 to GND
            Screen(10);

        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4  HIGH

        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- LOW

        if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == GPIO_INPUT_PIN_LOW)  ){     // Column 2 to GND
            pressedKey = hexaKeys[0][0];        // Shows "0"
            //LCD_E_setMemory(LCD_E_BASE,LCD_E_MEMORY_BLINKINGMEMORY_2, 0x60); //second character set to 1
            Screen(0);

        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4  HIGH


        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- HIGH
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- LOW

        if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == GPIO_INPUT_PIN_LOW)  ){     // Column 3 to GND
            Screen( 11 );

        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- LOW
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4  HIGH
        //reset
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 1- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);  // Row 2- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); // Row 3- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 4- LOW



        LCD_E_on(LCD_E_BASE);

}
void LED()
{
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN2);
    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);

}


#pragma vector = PORT1_VECTOR       // Using PORT1_VECTOR interrupt because P1.4 and P1.5 are in port 1
__interrupt void PORT1_ISR()
{
    Key();
    //LED();


    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN3); // Column 3
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN6); // Column 2
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN4); // Row 1
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN5); // Column 1
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN5); // Row 2
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN7); // Row 3
    GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN0); // Row 4

    GPIO_clearInterrupt(GPIO_PORT_P8, GPIO_PIN2); //
    GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN2); //
    GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN3); //





}


    //GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN4);
    //GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
