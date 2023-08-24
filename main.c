#include "DSP2833x_Device.h"

// external function prototypes
extern void InitAdc(void);
extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitCpuTimers(void);
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);

// Prototype statements for functions found within this file.
void Gpio_select(void);
interrupt void cpu_timer0_isr(void);
interrupt void ePWM2A_compare_isr(void);

// Global Variables
int VolRef=10;
float Voltage_VR1;
int counter=0;
float Vol;
float V[2]={0,0};
float D[2]={0,0};
float kp=0.05326,ki=0.5326;
float var;
//###########################################################################
//                      main code
//###########################################################################
void main(void)
{
    InitSysCtrl();  // Basic Core Init from DSP2833x_SysCtrl.c

    EALLOW;
    SysCtrlRegs.WDCR= 0x00AF;   // Re-enable the watchdog
    EDIS;           // 0x00AF  to NOT disable the Watchdog, Prescaler = 64

    DINT;               // Disable all interrupts

    Gpio_select();      // To initialize epwm and ADC pins

    InitPieCtrl();      // basic setup of PIE table; from DSP2833x_PieCtrl.c

    InitPieVectTable(); // default ISR's in PIE

    InitAdc();          // Basic ADC setup, incl. calibration

    AdcRegs.ADCTRL1.all = 0;
    AdcRegs.ADCTRL1.bit.ACQ_PS = 7;     // 7 = 8 x ADCCLK
    AdcRegs.ADCTRL1.bit.SEQ_CASC =1;    // 1=cascaded sequencer
    AdcRegs.ADCTRL1.bit.CPS = 0;        // divide by 1
    AdcRegs.ADCTRL1.bit.CONT_RUN = 0;   // single run mode

    AdcRegs.ADCTRL2.all = 0;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;   // 1=enable SEQ1 interrupt
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 =1;  // 1=SEQ1 start from ePWM_SOCA trigger
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;   // 0= interrupt after every end of sequence

    AdcRegs.ADCTRL3.bit.ADCCLKPS = 3;   // ADC clock: FCLK = HSPCLK / 2 * ADCCLKPS
                                        // HSPCLK = 75MHz (see DSP2833x_SysCtrl.c)
                                        // FCLK = 12.5 MHz

    AdcRegs.ADCMAXCONV.all = 0x0001;    // 2 conversions from Sequencer 1

    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0; // Setup ADCINA0 as 1st SEQ1 conv.
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 1; // Setup ADCINA1 as 2nd SEQ1 conv.

    EPwm2Regs.TBCTL.all = 0xC030;   // Configure timer control register
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1;  // HSPCLKDIV = 4
    /*
     bit 15-14     11:     FREE/SOFT, 11 = ignore emulation suspend
     bit 13        0:      PHSDIR, 0 = count down after sync event
     bit 12-10     000:    CLKDIV, 000 => TBCLK = HSPCLK/1
     bit 9-7       000:    HSPCLKDIV, 000 => HSPCLK = SYSCLKOUT/1
     bit 6         0:      SWFSYNC, 0 = no software sync produced
     bit 5-4       11:     SYNCOSEL, 11 = sync-out disabled
     bit 3         0:      PRDLD, 0 = reload PRD on counter=0
     bit 2         0:      PHSEN, 0 = phase control disabled
     bit 1-0       00:     CTRMODE, 00 = count up mode
    */

    EPwm2Regs.AQCTLA.all = 0x0024;      // set ePWM2A on CMPA up
                                            // clear ePWM2A on TBPRD
    EPwm2Regs.TBPRD = 37500; // TPPRD +1  =  TPWM / (HSPCLKDIV * CLKDIV * TSYSCLK)
                            //           =  20  s / 6.667 ns

    EPwm2Regs.CMPA.half.CMPA = EPwm2Regs.TBPRD / 2;
    EPwm2Regs.ETPS.all = 0x0100;            // Configure ADC start by ePWM2

    /*
     bit 15-14     00:     EPWMxSOCB, read-only
     bit 13-12     00:     SOCBPRD, don't care
     bit 11-10     00:     EPWMxSOCA, read-only
     bit 9-8       01:     SOCAPRD, 01 = generate SOCA on first event
     bit 7-4       0000:   reserved
     bit 3-2       00:     INTCNT, don't care
     bit 1-0       00:     INTPRD, don't care
    */

    EPwm2Regs.ETSEL.all = 0x0A0A;           // Enable SOCA to ADC and interrupt enable for
                                            // for epwm interrupt on prd match
   // EPwm2Regs.ETSEL.bit.INTEN = 1;      // interrupt enable for ePWM2
   // EPwm2Regs.ETSEL.bit.INTSEL = 4;     // interrupt on CMPA UP match
    EPwm2Regs.ETPS.bit.INTPRD = 1;      // interrupt on first event
    /*
     bit 15        0:      SOCBEN, 0 = disable SOCB
     bit 14-12     000:    SOCBSEL, don't care
     bit 11        1:      SOCAEN, 1 = enable SOCA
     bit 10-8      010:    SOCASEL, 010 = SOCA on PRD event
     bit 7-4       0000:   reserved
     bit 3         0:      INTEN, 0 = disable interrupt
     bit 2-0       000:    INTSEL, don't care
    */

    EALLOW;
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.EPWM2_INT = &ePWM2A_compare_isr;
    EDIS;

    InitCpuTimers();    // basic setup CPU Timer0, 1 and 2

    ConfigCpuTimer(&CpuTimer0,150,100000);

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // CPU Timer 0
    //PieCtrlRegs.PIEIER1.bit.INTx6 = 1;      // ADC


    IER |=1;

    // Enable EPWM2A INT in the PIE: Group 3 interrupt 1
        PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
        IER |=5;            // enable INT4 for ePWM2

    EINT;
    ERTM;

    CpuTimer0Regs.TCR.bit.TSS = 0;  // start timer0

    while(1)
        {
            while(CpuTimer0.InterruptCount == 0)
            {
                EALLOW;
                SysCtrlRegs.WDKEY = 0x55;   // service WD #1
                SysCtrlRegs.WDKEY = 0xAA;   // service WD #2
                EDIS;
            }

            CpuTimer0.InterruptCount = 0;
        }
    }

void Gpio_select(void)
{
    EALLOW;
    GpioCtrlRegs.GPAMUX1.all = 0;       // GPIO15 ... GPIO0 = General Puropse I/O
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; // ePWM2A active
    GpioCtrlRegs.GPAMUX2.all = 0;       // GPIO31 ... GPIO16 = General Purpose I/O
    GpioCtrlRegs.GPBMUX1.all = 0;       // GPIO47 ... GPIO32 = General Purpose I/O
    GpioCtrlRegs.GPBMUX2.all = 0;       // GPIO63 ... GPIO48 = General Purpose I/O
    GpioCtrlRegs.GPCMUX1.all = 0;       // GPIO79 ... GPIO64 = General Purpose I/O
    GpioCtrlRegs.GPCMUX2.all = 0;       // GPIO87 ... GPIO80 = General Purpose I/O

    GpioCtrlRegs.GPADIR.all = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;  // peripheral explorer: LED LD1 at GPIO9
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1; // peripheral explorer: LED LD2 at GPIO11

    GpioCtrlRegs.GPBDIR.all = 0;        // GPIO63-32 as inputs
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1; // peripheral explorer: LED LD3 at GPIO34
    GpioCtrlRegs.GPBDIR.bit.GPIO49 = 1; // peripheral explorer: LED LD4 at GPIO49

    GpioCtrlRegs.GPCDIR.all = 0;        // GPIO87-64 as inputs
    EDIS;
}

interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    EALLOW;
    SysCtrlRegs.WDKEY = 0xAA;   // service WD #2
    EDIS;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void ePWM2A_compare_isr(void)
// ISR is triggered by ePWM2 PRD event
{
    //static unsigned int index=0;
    // Service watchdog every interrupt
    Voltage_VR1 = AdcMirror.ADCRESULT0; // store results global
    EALLOW;
    SysCtrlRegs.WDKEY = 0xAA;       // Service watchdog #2
    EDIS;

    Vol= (Voltage_VR1*3/4095)*10;
    V[1]=V[0];
    V[0]=Vol;
    D[1]=D[0];

    D[0]=D[1]+((kp+(ki*0.00025))*(VolRef-V[0]))+((-kp+(ki*0.00025))*(VolRef-V[1]));

    if (D[0]>0.9)
    { D[0]=0.9;}

    else if (D[0]<.1)
    {
        D[0]=.1;
    }
    var=EPwm2Regs.TBPRD*(1-D[0]);
    EPwm2Regs.CMPA.half.CMPA=EPwm2Regs.TBPRD*(1-D[0]);
    EPwm2Regs.ETCLR.bit.INT = 1;        // Clear ePWM1 Interrupt flag
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = 4;
}

//===========================================================================
// End of SourceCode.
//===========================================================================
