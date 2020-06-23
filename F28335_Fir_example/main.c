/*
 *      @author : Sami Emre Yaðlý
 *      @project: FIR Filter Design
 */

#include "DSP2833x_Device.h"
#include "fdacoefs.h"    // filter coefficients

// User defines
#define N 52             // length of filter

// External function prototypes
extern void InitAdc(void);
extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitCpuTimers(void);
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);

// Prototype statements for functions found within this file.
void Gpio_select(void);
float firFilter(const float *h,int len);     // FIR Filter Function
interrupt void cpu_timer0_isr(void);         // Timer-0 ISR
interrupt void adc_isr(void);                // ADC - End of Sequence ISR

// Global Variables
int adcRead = 0;    //sample
int x[N] = { 0 };   //input sequence
int output = 0;   //output


void main(void)
{
    InitSysCtrl();              // Basic Core Init from DSP2833x_SysCtrl.c

    EALLOW;
    SysCtrlRegs.WDCR = 0x00AF;  // Re-enable the watchdog
    EDIS;

    DINT;                       // Disable all interrupts

    Gpio_select();

    InitPieCtrl();              // basic setup of PIE table; from DSP2833x_PieCtrl.c

    InitPieVectTable();         // default ISR's in PIE

    InitAdc();                  // Basic ADC setup, incl. calibration


    AdcRegs.ADCTRL1.all = 0;
    AdcRegs.ADCTRL1.bit.ACQ_PS = 7;         // 8 x ADCCLK
    AdcRegs.ADCTRL1.bit.SEQ_CASC =1;        // 1=cascaded sequencer
    AdcRegs.ADCTRL1.bit.CONT_RUN = 0;       // single run mode
    AdcRegs.ADCTRL2.all = 0;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;   // 1=enable SEQ1 interrupt
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;   // 0= interrupt after every end of sequence
    AdcRegs.ADCTRL3.bit.ADCCLKPS = 3;       // ADC clock: FCLK = HSPCLK / 2 * ADCCLKPS
                                            // HSPCLK = 75MHz (see DSP2833x_SysCtrl.c)
                                            // FCLK = 12.5 MHz
    AdcRegs.ADCMAXCONV.all = 0;             // 1 conversion
    AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0;    // Setup ADCINA5

    EALLOW;
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.ADCINT = &adc_isr;
    EDIS;

    InitCpuTimers();                        // basic setup CPU Timer0, 1 and 2

    ConfigCpuTimer(&CpuTimer0,150,100);     // 100 us

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // CPU Timer 0
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;      // ADC

    IER |=1;

    EINT;
    ERTM;

    CpuTimer0Regs.TCR.bit.TSS = 0;          // start timer0

    while(1)
    {
        while(AdcRegs.ADCST.bit.SEQ2_BSY)
        {
            // wait until sample is available
            EALLOW;
            SysCtrlRegs.WDKEY = 0x55;   // service WD #1
            SysCtrlRegs.WDKEY = 0xAA;   // service WD #2
            EDIS;
        }

        output = (int) firFilter(B,N);    //Then the variable 'output' can be used in DAC processes.
    }
}


float firFilter(const float* h,int len){
    int i = 0;
    float y = 0.0;

    //Shifting samples
    for(i = len-1; i>0; i++)
        x[i] = x[i-1];

    //Get new sample
    x[0] = adcRead;

    //Accumulation
    for(i=0; i<len; i++)
        y += h[i] * x[i];

    return y;
}

void Gpio_select(void)
{
    EALLOW;
    GpioCtrlRegs.GPAMUX1.all = 0;       // GPIO15 ... GPIO0 = General Puropse I/O
    GpioCtrlRegs.GPAMUX2.all = 0;       // GPIO31 ... GPIO16 = General Purpose I/O
    GpioCtrlRegs.GPBMUX1.all = 0;       // GPIO47 ... GPIO32 = General Purpose I/O
    GpioCtrlRegs.GPBMUX2.all = 0;       // GPIO63 ... GPIO48 = General Purpose I/O
    GpioCtrlRegs.GPCMUX1.all = 0;       // GPIO79 ... GPIO64 = General Purpose I/O
    GpioCtrlRegs.GPCMUX2.all = 0;       // GPIO87 ... GPIO80 = General Purpose I/O

    GpioCtrlRegs.GPADIR.all = 0;        // GPIO32-0 as inputs
    GpioCtrlRegs.GPBDIR.all = 0;        // GPIO63-32 as inputs
    GpioCtrlRegs.GPCDIR.all = 0;        // GPIO87-64 as inputs
    EDIS;
}

interrupt void cpu_timer0_isr(void)
{
    AdcRegs.ADCTRL2.bit.SOC_SEQ2 = 1;       // Start ADC SEQ2
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge interrupt to PIE
}

interrupt void  adc_isr(void)
{
    adcRead = AdcMirror.ADCRESULT10;        // Read from ADCINA5
    AdcRegs.ADCTRL2.bit.RST_SEQ2 = 1;       // Reset SEQ2
    AdcRegs.ADCST.bit.INT_SEQ2_CLR = 1;     // Clear INT SEQ2 bit
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge interrupt to PIE
}

