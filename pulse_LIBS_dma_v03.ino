// pulse generator for LIBS experiments
// STM32duino platform, intended for a STM32F103C8T6 (blue pill)
// four pulses of predefined delay and length
// real delays for the DP laser: t0=lamp;  
// DMA output to BSSR register, so it can turn to one (1) or left as is (0), upper 16bits means 1=turn zero, 0=left as it is (https://gist.github.com/iwalpola/6c36c9573fd322a268ce890a118571ca)

// v01 11ago2019  test DMA
// v02 11ago2019  PC-comm:    q0 to remove qswitch pulses (both), qxxx to activate qswitches in the xxx nth pulse (1=first, 2=second...), a1 to activate alternate delays, a0 to not use alternate delays for espectrometer... complete description in setup()
// measured cycle time is about 95ns, with the calibration waveform (1000 cycles) the delay is 97257ns , so cycle time is 97.257 ns ... no, it is 97.216ns
// v03 14ago2019  testing

#include <dma_private.h>

// pin naming in STM32-arduino is: .. i don't know yet 
#define BIT_LAMP 1 // first bit in the 16bits register = A0
#define BIT_Q1   2  // A1
#define BIT_Q2   4  // A2
#define BIT_SPEC 8 // fourth bit single output but dual delay   A3

// in bluepill PC13 is the onboard LED
#define PIN_LED PC13

#define NANOS  *1000
#define MICROS *1000000
#define CYCLE  97220 // ps for each value at the array
#define PULSE_LENGTH  8 MICROS // ps pulse length, for Lotis laser: 5..10us

uint32_t delay_q1 =160 MICROS; // first pulse QSWITCH1
uint32_t delay_q2 =155 MICROS; // second pulse
uint32_t delay_sp =159400 NANOS; // spectrometer
uint32_t delta_delay_sp =1000 NANOS; // increment for alternate delay of espectrometer

boolean alternate_delay=false; // if true, every other pulse for espectrometer will be delayed (for CFlibs)

uint32_t pulse_rate_hz=10; // every 100ms


#define MAX_CYCLES_VECTOR 2000 // for compile-time allocation, STM32F13C has 20k of RAM and 64k of Flash, with 2000 up to 190us delays, DO NOT INCREASE, it won't compile somehow

// THE vector
uint32_t vector[MAX_CYCLES_VECTOR];

uint32_t idx;
uint32_t ncycl_pulse;
char cmd; // for serial
uint32_t val; // for serial

uint32_t pulse_counter;
boolean starting=false; // after a "q" command
uint32_t npulses2activateQSW; // nth pulse that activate qswitch
boolean ledonoff;
boolean first_run=true;

void setup() 
{
  digitalWrite(PA0,0);
  digitalWrite(PA1,0);
  digitalWrite(PA2,0);
  digitalWrite(PA3,0);
  pinMode(PA0, OUTPUT);
  pinMode(PA1, OUTPUT);
  pinMode(PA2, OUTPUT);
  pinMode(PA3, OUTPUT);
  pinMode(PC13, OUTPUT);
  ledonoff=false;

  Serial.begin(38400); // creo que es la misma velocidad que el programa de Luis en el PIC
  
   // CREATE vector of outputs, first initialize to zeroes
  for(int i = 0; i < MAX_CYCLES_VECTOR; i++) 
   { vector[i]=0; }
  ncycl_pulse = PULSE_LENGTH / CYCLE; // cycles a pulse will last

  // lamp pulse, all others left as they are
  idx=0; // index within vector
  for (int i=0;i<ncycl_pulse;++i)
    vector[idx+i] = BIT_LAMP ; // | ((0xFFFF-BIT_LAMP)<<16) ;  // the other channels are already set to zero 
  idx=ncycl_pulse;
  vector[idx]= (BIT_LAMP<<16); // set to zero the LAMP bit
  

  //activate_qswitch1();  // OJO!!! xxxxxxxxxxxxxxx should start with no QSwitch pulses!!!!!
  //activate_qswitch2();
  set_spec_delay(0);
  starting = false;
  pulse_counter = 0;

  while (!Serial) ;  // https://forum.arduino.cc/index.php?topic=171889.0

  delay(3000); // without this delay, no serial output (!)   removed so autolibs startup is not delayed, 

  Serial.println("Pulse generator for LIBS vers 02 11aug2019 ");
  Serial.println("?:   status");
  Serial.println("q0:   remove qswitch pulses (both)");
  Serial.println("qnnn: activate qswitches in the n-th pulse (1=first pulse), usefull to get noise spectra.");
  Serial.println("a0:   do not use alternate delay for spectrometer   a1: use alternate delay");
  Serial.println("dnnn: delta delay for alternate delay [ns]  ej. d1000 = 1us alternate delay (first and even pulses have no extra delay)"); 
  Serial.println("hnnn: repetition rate of the lamp pulse (<= 10 Hz)"); 
  Serial.println("innn: delay for q-switch-1 [ns]"); 
  Serial.println("jnnn: delay for q-switch-2 [ns]"); 
  Serial.println("snnn: delay for spectrometer [ns]"); 
  Serial.println("c:    change to calibration pulse at BIT_LAMP (1000 cycles between rise edges)"); 
  Serial.println("r:    change to calibration pulse at BIT_LAMP (fastest square signal)");
 
  Serial.println("Current configuration:");
  print_status();


}

void loop() 
{

  delay(500);
  while(1)
  {
    noInterrupts();
    dmaTransfer(vector, (uint32_t*)&GPIOA->regs->BSRR, MAX_CYCLES_VECTOR);
    delay_us((MAX_CYCLES_VECTOR * CYCLE) / 1000000); // wait those microseconds until the entire vector has been sent
    delay_us(20); // just a bit more time just in case
    interrupts();

    // we can now change the pulse for the espectrometer on the fly provided it takes less than 100ms
    if (alternate_delay && (pulse_counter % 2))
        set_spec_delay(delta_delay_sp);
     else
        set_spec_delay(0);

    check_serial(); // check serial port
    
    ++pulse_counter;
    if (starting)
    {
      if (pulse_counter >= npulses2activateQSW)
      {
        activate_qswitch1();
        activate_qswitch2(); 
        starting = false; // already passed the no-qswitch period
      }
    }

    // delay of 100ms (for 10Hz)
    delay(1000 / pulse_rate_hz);
    digitalWrite(PIN_LED,ledonoff);
    ledonoff = !ledonoff;
    
  }
}

void dmaTransfer(uint32_t* from, uint32_t* to,  uint32_t dataLength)
{
  dma_init(DMA1);
  dma_setup_transfer(DMA1, DMA_CH1, from, DMA_SIZE_32BITS, to, DMA_SIZE_32BITS, DMA_PINC_MODE |  DMA_MEM_2_MEM);
  dma_set_num_transfers(DMA1, DMA_CH1, dataLength);
  dma_enable(DMA1, DMA_CH1);
}

void activate_qswitch1() // add q1 pulse
{
  uint32_t corrected_delay;
  if (delay_q1>100000000)
     corrected_delay = delay_q1 - (delay_q1/35 - 2893000); // correction for values above 100us, 
  else
     corrected_delay = delay_q1;   
 // qswitch1 pulse, all others left as they are
  idx=corrected_delay / CYCLE; // start of the pulse
  for (int i=0;i<idx;++i)// first erase the espectrometer bit before the new pulse
    vector[i] &= (((0xFFFF-BIT_Q1)<<16) | (0xFFFF-BIT_Q1));
 for (int i=0;i<ncycl_pulse;++i)
  { 
    vector[idx+i] |=  BIT_Q1; // set to one, others left 
    vector[idx+i] &= (((0xFFFF-BIT_Q1)<<16) | 0xFFFF); // set to zero the upper bit
  }
  idx=corrected_delay / CYCLE + ncycl_pulse;
  vector[idx] |= (BIT_Q1<<16); // set to one only this bit 
  vector[idx] &= ((0xFFFF<<16) | (0xFFFF-BIT_Q1)); // set to zero the lower bit
  
  for (int i=idx+1;i<MAX_CYCLES_VECTOR;++i)// erase the qswitch bit up to the end, because there could be a previous more delayed pulse after this one 
        vector[i] &= (((0xFFFF-BIT_Q1)<<16) | (0xFFFF-BIT_Q1));
}
void activate_qswitch2() // add q2 pulse
{
  uint32_t corrected_delay;
  if (delay_q2>100000000)
     corrected_delay = delay_q2 - (delay_q2/35 - 2893000); // correction for values above 100us, 
  else
     corrected_delay = delay_q2;   
 // qswitch2 pulse, all others left as they are
  idx=corrected_delay / CYCLE; // start of the pulse
  for (int i=0;i<idx;++i)// first erase the espectrometer bit before the new pulse
    vector[i] &= (((0xFFFF-BIT_Q2)<<16) | (0xFFFF-BIT_Q2));
 for (int i=0;i<ncycl_pulse;++i)
  { 
    vector[idx+i] |=  BIT_Q2; // set to one, others left 
    vector[idx+i] &= (((0xFFFF-BIT_Q2)<<16) | 0xFFFF); // set to zero the upper bit
  }
  idx=corrected_delay / CYCLE + ncycl_pulse;
  vector[idx] |= (BIT_Q2<<16); // set to one only this bit 
  vector[idx] &= ((0xFFFF<<16) | (0xFFFF-BIT_Q2)); // set to zero the lower bit
  
  for (int i=idx+1;i<MAX_CYCLES_VECTOR;++i)// erase the qswitch bit up to the end, because there could be a previous more delayed pulse after this one 
        vector[i] &= (((0xFFFF-BIT_Q2)<<16) | (0xFFFF-BIT_Q2));
}


void remove_qswitch1() // remove the q1 pulse
{
 for (int i=0;i<MAX_CYCLES_VECTOR;++i)// erase the qswitch bit 
        vector[i] &= ((0xFFFF-BIT_Q1)<<16) | (0xFFFF-BIT_Q1);
}


void remove_qswitch2() // remove the q2 pulse
{
 for (int i=0;i<MAX_CYCLES_VECTOR;++i)// erase the qswitch bit 
        vector[i] &= ((0xFFFF-BIT_Q2)<<16) | (0xFFFF-BIT_Q2);
}

void set_spec_delay(uint32_t delta)    // spectrometer pulse, all others left as they are
{
  uint32_t corrected_delay;
  if ((delay_sp+delta)>100000000)
     corrected_delay = (delay_sp + delta)- ((delay_sp+delta)/35 - 2893000); // correction for values above 100us, 
  else
     corrected_delay = (delay_sp+delta);   

  //Serial.print("delta (ps)=");Serial.println(delta); 
  //Serial.print("sp corrected=");Serial.println(corrected_delay); 
  idx=corrected_delay / CYCLE; // start of the pulse
  //Serial.print("sp idx=");Serial.println(idx);  
  for (int i=0;i<idx;++i)// first erase the espectrometer bit before the new pulse, just in case
    vector[i] &= (((0xFFFF-BIT_SPEC)<<16) | (0xFFFF-BIT_SPEC));
  for (int i=0;i<ncycl_pulse;++i)
  { 
    vector[idx+i] |=  BIT_SPEC; // set to one, others left 
    vector[idx+i] &= (((0xFFFF-BIT_SPEC)<<16) | 0xFFFF); // set to zero the upper bit
  }
  idx=(corrected_delay / CYCLE) + ncycl_pulse;
  //Serial.print("sp idx2=");Serial.println(idx);  
  vector[idx] |= (BIT_SPEC<<16); // set to one only this bit
  vector[idx] &= ((0xFFFF<<16) | (0xFFFF-BIT_SPEC)); // set to zero the lower bit
  
  for (int i=idx+1;i<MAX_CYCLES_VECTOR;++i)// erase the SPEC bit up to the end, because there could be a previous more delayed pulse after this one 
        vector[i] &= (((0xFFFF-BIT_SPEC)<<16) | (0xFFFF-BIT_SPEC));

}

void check_serial()
{
  if (Serial.available() > 0)
    {
      cmd = Serial.read();
      if (cmd=='q')
       {
        val = Serial.parseInt();;
        if (val==0) // remove qswitch pulses
           {
            remove_qswitch1();
            remove_qswitch2();
           }
         else
           { 
            npulses2activateQSW = val; // number of pulses to skip
            pulse_counter = 0;
            starting=1;
           }
       }
      if (cmd=='a')
      {
        val = Serial.parseInt();;
        if (val==0) // do not use alternate delays for spectrometer
            alternate_delay=false;
         else
            alternate_delay=true; // activates alternate delay
      }
      if (cmd=='d')
      {
        val = Serial.parseInt();
        if (val>100000)
          val=100000;  // max 100us
        delta_delay_sp = val * 1000; // to picoseconds
      }
      if (cmd=='i')
      {
        val = Serial.parseInt();
        if (val>200000)
          val=200000;  // max 200us
        delay_q1 = val * 1000; // to picoseconds
      }
      if (cmd=='j')
      {
        val = Serial.parseInt();
        if (val>200000)
          val=200000;  // max 200us
        delay_q2 = val * 1000; // to picoseconds
      }
      if (cmd=='s')
      {
        val = Serial.parseInt();
        if (val>200000)
          val=200000;  // max 200us
        delay_sp = val * 1000; // to picoseconds
      }
      if (cmd=='?')
      {
        print_status();
      }
      if (cmd=='h')  // repetition rate in Hz
      {
        val = Serial.parseInt();
        if (val>10)
         val=10;
        if (val<1)
         val=1;
        pulse_rate_hz = val; // repetition rate
      }

      if (cmd=='c') // change the pulse in BIT_LAMP to two separated 1000 cycles (for calibration)
      {
        create_debug_pulse_1000();
      }

      if (cmd=='r')
      {
        create_debug_pulse_square();
      }

    }
}

void print_status()
{
  Serial.print(" delay_q1 [ns]="); Serial.println(delay_q1 / 1000);
  Serial.print(" delay_q2 [ns]="); Serial.println(delay_q2 / 1000);
  Serial.print(" delay_sp [ns]="); Serial.println(delay_sp / 1000);
  Serial.print(" delta_delay_sp [ns]="); Serial.println(delta_delay_sp / 1000);
}

void create_debug_pulse_square()  // create a on/off pulse at bit 0 (A0?)
{
 for (int i=0;i<MAX_CYCLES_VECTOR;++i)
  if (i%2==0)
    vector[i] = BIT_LAMP ;  // set bit to one
  else
    vector[i] = BIT_LAMP << 16;  // set bit to zero
}

void create_debug_pulse_1000()  // create a pulse and 1000 cycles after that, another one
{
 for(int i = 0; i < MAX_CYCLES_VECTOR; i++) 
   { vector[i]=0; }
 vector[0] = BIT_LAMP ;  // set bit to one
 vector[1] = BIT_LAMP << 16;
 /* there are 1000 cycles in between the rise edges */
 vector[1000] = BIT_LAMP ;  // set bit to one
 vector[1001] = BIT_LAMP << 16;


}

