#ifndef _SYNTH
#define _SYNTH
//*************************************************************************************
//  Arduino synth V4.1
//  Optimized audio driver, modulation engine, envelope engine.
//
//  Dzl/Illutron 2014
//
//*************************************************************************************
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "tables.h"

#define DIFF 1
#define CHA 2
#define CHB 3

#define SINE     0
#define TRIANGLE 1
#define SQUARE   2
#define SAW      3
#define RAMP     4
#define NOISE    5

#define ENVELOPE0 0
#define ENVELOPE1 1
#define ENVELOPE2 2
#define ENVELOPE3 3

#define FS 20000.0                                              //-Sample rate (NOTE: must match tables.h)

#define SET(x,y) (x |=(1<<y))		        		//-Bit set/clear macros
#define CLR(x,y) (x &= (~(1<<y)))       			// |
#define CHK(x,y) (x & (1<<y))           			// |
#define TOG(x,y) (x^=(1<<y))            			//-+

volatile unsigned int WavePhaseAcc[4] = {
  0, 0, 0, 0};			//-Wave phase accumolators
volatile unsigned int WavePhaseInc[4] = {
  1000, 200, 300, 400};           //-Wave frequency tuning words
volatile unsigned char AMP[4] = {
  255, 255, 255, 255};           //-Wave amplitudes [0-255]
volatile unsigned int PITCH[4] = {
  500, 500, 500, 500};          //-Voice pitch
volatile int EnvelopeModulation[4] = {
  20, 0, 64, 127};                         //-Voice envelope modulation [0-1023 512=no mod. <512 pitch down >512 pitch up]
volatile unsigned int wavs[4];                                  //-Wave table selector [address of wave in memory]
volatile unsigned int envs[4];                                  //-Envelopte selector [address of envelope in memory]
volatile unsigned int EnvelopePhaseAcc[4] = {
  0x8000, 0x8000, 0x8000, 0x8000}; //-Envelope phase accumolator
volatile unsigned int EnvelopePhaseInc[4] = {
  10, 10, 10, 10};               //-Envelope speed tuning word
volatile unsigned char divider = 4;                             //-Sample rate decimator for envelope
volatile unsigned int tim = 0;
volatile unsigned char tik = 0;
volatile unsigned char output_mode;


// Output pins, Timers etc (Board specific, see Atmel datasheet of your chip, 16bit Timer, Register Description)
// Any changes must also be incorporate the begin function!
  #if defined(__AVR_ATmega8__) 
    // https://docs.arduino.cc/hacking/hardware/PinMapping
    // https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2486-8-bit-AVR-microcontroller-ATmega8_L_datasheet.pdf
    #define LED_PIN       13
    #define LED_PORT      PORTB //register address
    #define LED_BIT       5     //relevant bit in register address  //usage: TOG(LED_PORT, LED_BIT);

    /* TODO
    //PWM (2-Channel: A and B)
    #define PWM_A_PIN     11
    #define PWM_A_DDR     DDRB    // Data Direction byte of the port (DDR + port)
    #define PWM_A_Bit     3       // relevant bit in port
    #define PWM_A_VALUE   OCR2A   // Output Compare Register
    
    #define PWM_B_PIN     3
    #define PWM_B_DDR     DDRD    // Data Direction byte of the port (DDR + port)
    #define PWM_B_Bit     3       // relevant bit in port
    #define PWM_B_VALUE   OCR2B   // Output Compare Register

    #define PWM_TCCRA     TCCR2A  // Timer/Counter <2> Control Register A
    #define PWM_TCCRB     TCCR2B  

    //Audio Driver Interrupt - ADI
    #define ADI_TCCRA     TCCR1A  // Timer/Counter <1> Control Register A
    #define ADI_TCCRB     TCCR1B
    #define ADI_TCCRC     TCCR1C
    #define ADI_VALUE     OCR1A
    #define ADI_TIMSK     TIMSK1 // Timer/Counter <1> Interrupt Mask Register
    #define ADI_OCIE      OCIE1A // Output Compare Interrupt Enable Bit
    #define ADI_INTERRUPT TIMER1_COMPA_vect
    */
  #elif defined(__AVR_ATmega1280__)  || defined(__AVR_ATmega2560__)
    // https://docs.arduino.cc/hacking/hardware/PinMapping2560
    // https://ww1.microchip.com/downloads/en/devicedoc/atmel-2549-8-bit-avr-microcontroller-atmega640-1280-1281-2560-2561_datasheet.pdf
    #define LED_PIN       13
    #define LED_PORT      PORTB //register address
    #define LED_BIT       7     //relevant bit in register address
    
    //PWM (2-Channel: A and B)
    #define PWM_A_PIN     3       // PE5 (OC3C) -> Port E, Bit 3; Timer 3, Compare Register C
    #define PWM_A_DDR     DDRE    // Data Direction byte of the port (DDR + port)
    #define PWM_A_Bit     5       // relevant bit in port
    #define PWM_A_VALUE   OCR3C   // Output Compare Register
    
    #define PWM_B_PIN     2       // PE4 (OC3B)
    #define PWM_B_DDR     DDRE    // Data Direction byte of the port (DDR + port)
    #define PWM_B_Bit     4       // relevant bit in port
    #define PWM_B_VALUE   OCR3B   // Output Compare Register

    #define PWM_TCCRA     TCCR3A  // Timer/Counter <3> Control Register A
    #define PWM_TCCRB     TCCR3B  

    //Audio Driver Interrupt - ADI
    #define ADI_TCCRA     TCCR1A  // Timer/Counter <1> Control Register A
    #define ADI_TCCRB     TCCR1B
    #define ADI_TCCRC     TCCR1C
    #define ADI_VALUE     OCR1A
    #define ADI_TIMSK     TIMSK1 // Timer/Counter <1> Interrupt Mask Register
    #define ADI_OCIE      OCIE1A // Output Compare Interrupt Enable Bit
    #define ADI_INTERRUPT TIMER1_COMPA_vect
  #else 
    // ATmega168 (and ATmega328 ?) boards
    // https://docs.arduino.cc/hacking/hardware/PinMapping168
    // https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-9365-Automotive-Microcontrollers-ATmega88-ATmega168_Datasheet.pdf
    #define LED_PIN       13
    #define LED_PORT      PORTB //register address
    #define LED_BIT       5     //relevant bit in register address
    
    //PWM (2-Channel: A and B)
    #define PWM_A_PIN     11
    #define PWM_A_DDR     DDRB    // Data Direction byte of the port (DDR + port)
    #define PWM_A_Bit     3       // relevant bit in port
    #define PWM_A_VALUE   OCR2A   // Output Compare Register
    
    #define PWM_B_PIN     3       // PD3 (OC2B)
    #define PWM_B_DDR     DDRD    // Data Direction byte of the port (DDR + port)
    #define PWM_B_Bit     3       // relevant bit in port
    #define PWM_B_VALUE   OCR2B   // Output Compare Register

    #define PWM_TCCRA     TCCR2A  // Timer/Counter <2> Control Register A
    #define PWM_TCCRB     TCCR2B  

    //Audio Driver Interrupt - ADI
    #define ADI_TCCRA     TCCR1A  // Timer/Counter <1> Control Register A
    #define ADI_TCCRB     TCCR1B
    #define ADI_TCCRC     TCCR1C
    #define ADI_VALUE     OCR1A
    #define ADI_TIMSK     TIMSK1 // Timer/Counter <1> Interrupt Mask Register
    #define ADI_OCIE      OCIE1A // Output Compare Interrupt Enable Bit
    #define ADI_INTERRUPT TIMER1_COMPA_vect
  #endif
//


//*********************************************************************************************
//  Audio driver interrupt
//*********************************************************************************************

SIGNAL(ADI_INTERRUPT)
{
  //-------------------------------
  // Time division
  //-------------------------------
  divider++;
  if(!(divider&=0x03))
    tik=1;

  //-------------------------------
  // Volume envelope generator
  //-------------------------------

  if (!(((unsigned char*)&EnvelopePhaseAcc[divider])[1]&0x80))
    AMP[divider] = pgm_read_byte(envs[divider] + (((unsigned char*)&(EnvelopePhaseAcc[divider]+=EnvelopePhaseInc[divider]))[1]));
  else
    AMP[divider] = 0;

  //-------------------------------
  //  Synthesizer/audio mixer
  //-------------------------------

  PWM_A_VALUE = PWM_B_VALUE = 127 +
    ((
    (((signed char)pgm_read_byte(wavs[0] + ((unsigned char *)&(WavePhaseAcc[0] += WavePhaseInc[0]))[1]) * AMP[0]) >> 8) +
    (((signed char)pgm_read_byte(wavs[1] + ((unsigned char *)&(WavePhaseAcc[1] += WavePhaseInc[1]))[1]) * AMP[1]) >> 8) +
    (((signed char)pgm_read_byte(wavs[2] + ((unsigned char *)&(WavePhaseAcc[2] += WavePhaseInc[2]))[1]) * AMP[2]) >> 8) +
    (((signed char)pgm_read_byte(wavs[3] + ((unsigned char *)&(WavePhaseAcc[3] += WavePhaseInc[3]))[1]) * AMP[3]) >> 8)
    ) >> 2);

  //************************************************
  //  Modulation engine
  //************************************************
  //  WavePhaseInc[divider] = PITCH[divider] + (int)   (((PITCH[divider]/64)*(EnvelopePhaseAcc[divider]/64)) /128)*EnvelopeModulation[divider];
  WavePhaseInc[divider] = PITCH[divider] + (int)   (((PITCH[divider]>>6)*(EnvelopePhaseAcc[divider]>>6))/128)*EnvelopeModulation[divider];
	tim++;
}

class synth
{
private:

public:

  synth()
  {
  }

  //*********************************************************************
  //  Startup
  //*********************************************************************

  void begin()
  {
    output_mode=CHA;
    begin(output_mode);
  }

  void begin(unsigned char d)
  {
    //Audio Driver Interrupt - ADI
    ADI_TCCRA = 0x00;             //-Start audio interrupt
    ADI_TCCRB = 0x09;
    ADI_TCCRC = 0x00;
    ADI_VALUE = 16000000.0 / FS;	//- set Audio sample rate
    SET(ADI_TIMSK, ADI_OCIE);     //-Start audio interrupt
    sei();                        //-+

    //8bit PWM output
    output_mode=d;
    switch(d)
    {
      case DIFF: // Differential signal on CHA and CHB pins
        #if defined(__AVR_ATmega1280__)  || defined(__AVR_ATmega2560__)
          PWM_TCCRA = 0x2D;                                  
          PWM_TCCRB = 0x09;                                 
        #else
          PWM_TCCRA = 0xB3;                                  
          PWM_TCCRB = 0x01;                                  
        #endif
        PWM_A_VALUE = PWM_B_VALUE = 127;
        SET(PWM_A_DDR, PWM_A_Bit);				      // set PWM pin to output
        SET(PWM_B_DDR, PWM_B_Bit);				      // set PWM pin to output
        break;

      case CHB: // Single ended signal on CHB pin PWM_B_PIN
        #if defined(__AVR_ATmega1280__)  || defined(__AVR_ATmega2560__)
          PWM_TCCRA = 0x21;                                  
          PWM_TCCRB = 0x09;                                 
        #else
          PWM_TCCRA = 0x23;               
          PWM_TCCRB = 0x01; 
        #endif
        PWM_A_VALUE = PWM_B_VALUE = 127;
        SET(PWM_B_DDR, PWM_B_Bit);				      // set PWM pin to output
        break;

      case CHA:
      default:
        output_mode=CHA; // Single ended signal in CHA pin PWM_A_PIN
        #if defined(__AVR_ATmega1280__)  || defined(__AVR_ATmega2560__)
          PWM_TCCRA = 0x09;                                  
          PWM_TCCRB = 0x09;                                 
        #else
          PWM_TCCRA = 0x83;                                 
          PWM_TCCRB = 0x01;
        #endif
        PWM_A_VALUE = PWM_B_VALUE = 127;
        SET(PWM_A_DDR, PWM_A_Bit);				      // set PWM pin to output
        break;
    }
  }

  //*********************************************************************
  //  Timing/sequencing functions
  //*********************************************************************

  unsigned char synthTick(void)
  {
    if(tik)
    {
      tik=0;
      return 1;  //-True every 4 samples
    }
    return 0;
  }

  unsigned char voiceFree(unsigned char voice)
  {
    if (!(((unsigned char*)&EnvelopePhaseAcc[voice])[1]&0x80))
      return 0;
    return 1;
  }


  //*********************************************************************
  //  Setup all voice parameters in MIDI range
  //  voice[0-3],wave[0-6],pitch[0-127],envelope[0-4],length[0-127],mod[0-127:64=no mod]
  //*********************************************************************

  void setupVoice(unsigned char voice, unsigned char wave, unsigned char pitch, unsigned char env, unsigned char length, unsigned int mod)
  {
    setWave(voice,wave);
    setPitch(voice,pitch);
    setEnvelope(voice,env);
    setLength(voice,length);
    setModChar(voice,mod);
  }

  //*********************************************************************
  //  Setup wave [0-6]
  //*********************************************************************

  void setWave(unsigned char voice, unsigned char wave)
  {
    switch (wave)
    {
    case TRIANGLE:
      wavs[voice] = (unsigned int)TriangleTable;
      break;
    case SQUARE:
      wavs[voice] = (unsigned int)SquareTable;
      break;
    case SAW:
      wavs[voice] = (unsigned int)SawTable;
      break;
    case RAMP:
      wavs[voice] = (unsigned int)RampTable;
      break;
    case NOISE:
      wavs[voice] = (unsigned int)NoiseTable;
      break;
    default:
      wavs[voice] = (unsigned int)SinTable;
      break;
    }
  }
  //*********************************************************************
  //  Setup Pitch [0-127]
  //*********************************************************************

  void setPitch(unsigned char voice,unsigned char MIDInote)
  {
    PITCH[voice]=pgm_read_word(&PITCHS[MIDInote]);
  }

  //*********************************************************************
  //  Setup Envelope [0-4]
  //*********************************************************************

  void setEnvelope(unsigned char voice, unsigned char env)
  {
    switch (env)
    {
    case 1:
      envs[voice] = (unsigned int)Env0;
      break;
    case 2:
      envs[voice] = (unsigned int)Env1;
      break;
    case 3:
      envs[voice] = (unsigned int)Env2;
      break;
    case 4:
      envs[voice] = (unsigned int)Env3;
      break;
    default:
      envs[voice] = (unsigned int)Env0;
      break;
    }
  }

  //*********************************************************************
  //  Setup Length [0-128]
  //*********************************************************************

  void setLength(unsigned char voice,unsigned char length)
  {
    EnvelopePhaseInc[voice]=pgm_read_word(&EFTWS[length]);
  }

  //*********************************************************************
  //  Setup mod
  //*********************************************************************

  void setModChar(unsigned char voice, unsigned char mod) //mod 0..127
  {
    EnvelopeModulation[voice]=((int)mod-64)*8;//-512..511 0=no mod
  }
  void setMod(unsigned char voice, int mod)
  {
    EnvelopeModulation[voice]=mod;
  }

  //*********************************************************************
  //  Midi trigger
  //*********************************************************************

  void mTrigger(unsigned char voice,unsigned char MIDInote)
  {
    PITCH[voice]=pgm_read_word(&PITCHS[MIDInote]);
    EnvelopePhaseAcc[voice]=0;
    WavePhaseInc[divider] = PITCH[voice] + (int)   (((PITCH[voice]>>6)*(EnvelopePhaseAcc[voice]>>6))/128)*EnvelopeModulation[voice];
  }

  //*********************************************************************
  //  Set frequency direct
  //*********************************************************************

  void setFrequency(unsigned char voice,float f)
  {
    PITCH[voice]=f/(FS/65535.0);

  }

  //*********************************************************************
  //  Set time
  //*********************************************************************

  void setTime(unsigned char voice,float t)
  {
    EnvelopePhaseInc[voice]=(1.0/t)/(FS/(32767.5*10.0));//[s];
  }

  //*********************************************************************
  //  Simple trigger
  //*********************************************************************

  void trigger(unsigned char voice)
  {
    EnvelopePhaseAcc[voice]=0;
    WavePhaseInc[voice]=PITCH[voice];
    //    WavePhaseInc[voice]=PITCH[voice]+(PITCH[voice]*(EnvelopePhaseAcc[voice]/(32767.5*128.0  ))*((int)EnvelopeModulation[voice]-512));
  }

  //*********************************************************************
  //  Suspend/resume synth
  //*********************************************************************

  void suspend()
  {
    CLR(ADI_TIMSK, ADI_OCIE);                            //-Stop audio interrupt
  }
  void resume()
  {
    SET(ADI_TIMSK, ADI_OCIE);                            //-Start audio interrupt
  }

};

#endif













