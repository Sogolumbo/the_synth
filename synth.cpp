
//*************************************************************************************
//  Arduino synth V4.1
//  Optimized audio driver, modulation engine, envelope engine.
//
//  Dzl/Illutron 2014
//
//*************************************************************************************

#ifndef _SYNTH
#define _SYNTH
#pragma once

#include <synth.h>


#define FS 20000.0                            //-Sample rate (NOTE: must match tables.h)

#define SET(x,y) (x |=(1<<y))		        		  //-Bit set/clear macros
#define CLR(x,y) (x &= (~(1<<y)))       			// |
#define CHK(x,y) (x & (1<<y))           			// |
#define TOG(x,y) (x^=(1<<y))            			//-+

volatile unsigned int PITCH[4];          //-Voice pitch
volatile unsigned int WavePhaseAcc[4];			//-Wave phase accumolators
volatile unsigned int WavePhaseInc[4];           //-Wave frequency tuning words
volatile unsigned int wavs[4];                                  //-Wave table selector [address of wave in memory]

volatile unsigned int waveMix[4][32];          //-Waveform Mix Table
volatile unsigned int WaveMixPhaseAcc[4]; //-Waveform Mix phase (LFO phase) accumulator
volatile unsigned int WaveMixPhaseInc[4]; //-Waveform Mix phase (LFO speed) increment

volatile unsigned char AMP[4];           //-Wave amplitudes [0-255]

volatile EnvelopeState envelopes[4];             // selected Envelope
volatile AmpEnvParams ampEnvParams[4];   // Envelope parameters - attack, decay, release: [0-127], sustain: [0-255]
volatile unsigned int envs[4];                                  // Envelope selector [address of envelope in memory]
volatile unsigned int EnvelopePhaseAcc[] = {0x8000, 0x8000, 0x8000, 0x8000}; //-Envelope phase accumolator
volatile unsigned int EnvelopePhaseInc[4];               //-Envelope speed tuning word

volatile unsigned char divider = 4;                             //-Sample rate decimator for envelope
volatile unsigned int tim = 0;
volatile unsigned char tik = 0;
volatile unsigned char output_mode;

//*********************************************************************************************
//  Timer Register definitions
//*********************************************************************************************
// (Board specific, see Atmel datasheet of your chip, 16bit Timer, Register Description)
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
//  Runs once for every sample. Must be below 50µs to keep up, currently ~32µs
//*********************************************************************************************

SIGNAL(ADI_INTERRUPT)
{
  //-------------------------------
  // Time division (~2% CPU)
  //-------------------------------
  
  divider++;
  if(!(divider&=0x03))
    tik=1;
  

  //-------------------------------
  // Volume envelope generator (~8% CPU)
  //-------------------------------
  if (!(synth::envelopeSectionFinished(divider))){
    AMP[divider] = pgm_read_byte(envs[divider] + (((unsigned char*)&(EnvelopePhaseAcc[divider]+=EnvelopePhaseInc[divider]))[1]));
  }
  

  //-------------------------------
  //  Synthesizer/audio mixer (~20% CPU)
  //-------------------------------

  PWM_A_VALUE = PWM_B_VALUE = 127 +
    ((
    (((signed char)pgm_read_byte(wavs[0] + ((unsigned char *)&(WavePhaseAcc[0] += WavePhaseInc[0]))[1]) * AMP[0]) >> 8) +
    (((signed char)pgm_read_byte(wavs[1] + ((unsigned char *)&(WavePhaseAcc[1] += WavePhaseInc[1]))[1]) * AMP[1]) >> 8) +
    (((signed char)pgm_read_byte(wavs[2] + ((unsigned char *)&(WavePhaseAcc[2] += WavePhaseInc[2]))[1]) * AMP[2]) >> 8) +
    (((signed char)pgm_read_byte(wavs[3] + ((unsigned char *)&(WavePhaseAcc[3] += WavePhaseInc[3]))[1]) * AMP[3]) >> 8)
    ) >> 2);

  
  //-------------------------------
  //  Waveform Mix/LFO (?% CPU)
  //-------------------------------
  wavs[divider] = (unsigned char *)waveMix[divider][((unsigned char *)&(WaveMixPhaseAcc[divider] += WaveMixPhaseInc[divider]))[1] >> 2];

  tim++;
}

static void synth::begin(OUTPUTMODE d)
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

static unsigned char synth::synthTick(void)
{
  if(tik)
  {
    tik=0;
    return 1;  //-True every 4 samples
  }
  return 0;
}


//*********************************************************************
//  Setup all voice parameters in MIDI range
//  voice[0-3],wave[0-6],pitch[0-127],envelope[0-4],length[0-127],mod[0-127:64=no mod]
//*********************************************************************
static void synth::setupVoice(unsigned char voice, 
                              WAVEFORM wave,
                              unsigned char attack,
                              unsigned char decay,
                              unsigned char sustain,
                              unsigned char release)
{
  setWave(voice,wave);
  setAmpEnvParams(voice, attack, decay, sustain, release);
}


//  Setup wave
static void synth::setWave(unsigned char voice, WAVEFORM wave)
{
  WaveMixPhaseAcc[voice] = 0;
  WaveMixPhaseInc[voice] = 0;

  switch (wave)
  {
    case TRIANGLE:
      waveMix[voice][0] = (unsigned int)TriangleTable;
      break;
    case SQUARE:
      waveMix[voice][0] = (unsigned int)SquareTable;
      break;
    case SAW:
      waveMix[voice][0] = (unsigned int)SawTable;
      break;
    case RAMP:
      waveMix[voice][0] = (unsigned int)RampTable;
      break;
    case NOISE:
      waveMix[voice][0] = (unsigned int)NoiseTable;
      break;
    case EPIANO:
      waveMix[voice][0] = (unsigned int)EpianoTable;
      break;
    case SINE:
    default:
      waveMix[voice][0] = (unsigned int)SinTable;
      break;
  }
}

//  Setup mixed wave (with LFO)
static void synth::setWaveformMix(unsigned char voice, WAVEFORMMIX waveformMix)
{
  switch (waveformMix)
  {
    case PIANO:
      for (int i = 0; i < 32; i++){
        waveMix[voice][i] = (unsigned int) PianoMixTable + i/2;
      }
      break;
    case TRIANGLE_AND_SAW:
      for (int i = 0; i < 32; i++){
        waveMix[voice][i] = (unsigned int) TriangleAndSawMixTable + i;
      }
      break;
    case SINE_AND_TRIANGLE:
      for (int i = 0; i < 32; i++){
        waveMix[voice][i] = (unsigned int) SineAndTriangleMixTable + i/2;
      }
      break;
    case NO_MIX:
    default:
      break;
  }
  WaveMixPhaseAcc[voice] = 0;
  if(waveformMix != NO_MIX){
    WaveMixPhaseInc[voice] = pgm_read_word(&EFTWS[90]);
  }
  else{
    WaveMixPhaseInc[voice] = 0;
  }
}


//  Setup Pitch [0-127]
static void synth::setPitch(unsigned char voice,unsigned char MIDInote)
{
  PITCH[voice]=pgm_read_word(&PITCHS[MIDInote]);
}
//  Set frequency direct
static void synth::setFrequency(unsigned char voice,float f)
{
  PITCH[voice]=f/(FS/65535.0);
}

// takes parameters [0-127], saves sustain [0-255]
static void synth::setAmpEnvParams(unsigned char voice,
  	  unsigned char attack, unsigned char decay,
      unsigned char sustain, unsigned char release, unsigned char releaseShape = 0){
  ampEnvParams[voice].attack = attack;
  ampEnvParams[voice].decay = decay;
  ampEnvParams[voice].sustain = sustain<<1;
  ampEnvParams[voice].release = release;
  ampEnvParams[voice].releaseShape = releaseShape;
}

//  Setup Envelope
static void synth::setAmpEnvelopeState(unsigned char voice, EnvelopeState envelope)
{
  envelopes[voice] = envelope;
  switch (envelope)
  {
  case ATTACK:
    envs[voice] = (unsigned int)AttackDecayTable;
    setLength(voice, ampEnvParams[voice].attack);
    break;
  case DECAY:
    envs[voice] = ((unsigned char*)&AttackDecayTable+128-(ampEnvParams[voice].sustain>>1));
    setLength(voice, ampEnvParams[voice].decay);
    EnvelopePhaseAcc[voice] = (unsigned int)ampEnvParams[voice].sustain<<7;
    break;
  case SUSTAIN:
    envs[voice] = ((unsigned char*)&AttackDecayTable+255-(ampEnvParams[voice].sustain>>1));
    EnvelopePhaseAcc[voice] = 0;
    EnvelopePhaseInc[voice] = 0;
    break;
 
  case RELEASE:
    switch(ampEnvParams->releaseShape){
      case 0:
        envs[voice] = (unsigned int)Env0;
        break;
      case 1:
        envs[voice] = (unsigned int)Env1;
        break;
      case 2:
        envs[voice] = (unsigned int)Env2;
        break;
      case 3:
        envs[voice] = (unsigned int)Env3;
    }
    break;

  case IDLE:
  default:
    envs[voice] = (unsigned int)Env0;
    EnvelopePhaseInc[voice] = 0;
    EnvelopePhaseAcc[voice] = 0x7FFF; // highest number allowed
    break;
  }
}

// Find the point at which the release Envelope has the value of the sustain parameter
// - Assumption: using Env0 as RELEASE
// - Assumptions for the envelope: start: 255, end: 0, strictly decreasing
// - Root-finding algorithm: Bisection method [~16µs with n<6 --> accuracy of +-2 for 'EnvelopePhaseAcc']
static void synth::setAmpEnvelopeStatePhaseAccForRelease(unsigned char voice){  
  unsigned char left = 0, right=127;
  unsigned char probe = 64 - (AMP[voice] >> 2); //starting point (likely approximate, linear dropoff in the first half)
  for(unsigned char n = 0; n < 6; n++){
    if(pgm_read_byte(((unsigned char *) &Env0 + probe)) >= AMP[voice]){
      left = probe;
    }
    else{
      right = probe;
    }
    probe = left + ((right-left)>>1);
  }
  EnvelopePhaseAcc[voice] = (unsigned int)probe<<8;
}

static bool synth::envelopeSectionFinished(unsigned char voice)
{
  return (((unsigned char*)&EnvelopePhaseAcc[voice])[1]&0x80);
}

static void synth::updateEnvelope(unsigned char voice){
  if(envelopeSectionFinished(voice)){
    switch(envelopes[voice]){
      case ATTACK:
        synth::setAmpEnvelopeState(voice, DECAY);
        break;
      case DECAY:
        synth::setAmpEnvelopeState(voice, SUSTAIN);
        break;
      case RELEASE:
        synth::setAmpEnvelopeState(voice, IDLE);
      default:
        break;
    }
  }
}


//  Setup Length [0-128]
static void synth::setLength(unsigned char voice,unsigned char length)
{
  EnvelopePhaseInc[voice]=pgm_read_word(&EFTWS[length]);
}
//  Set time
static void synth::setTime(unsigned char voice,float t)
{
  EnvelopePhaseInc[voice]=(1.0/t)/(FS/(32767.5*10.0));//[s];
}


//  Midi trigger
static void synth::mStart(unsigned char voice,unsigned char MIDInote, EnvelopeState envelope = ATTACK)
{
  setAmpEnvelopeState(voice, envelope);
  PITCH[voice]=pgm_read_word(&PITCHS[MIDInote]);
  EnvelopePhaseAcc[voice]=0;
  WavePhaseAcc[voice] = 0;
  WavePhaseInc[voice] = PITCH[voice];
}
static void synth::mStop(unsigned char voice)
{
  EnvelopePhaseInc[voice] = 0;
  setAmpEnvelopeStatePhaseAccForRelease(voice);
  setAmpEnvelopeState(voice, RELEASE);
  setLength(voice, ampEnvParams[voice].release);
}


// Simple trigger
static void synth::trigger(unsigned char voice)
{
  setAmpEnvelopeState(voice, RELEASE);
  EnvelopePhaseAcc[voice]=0;
  WavePhaseInc[voice]=PITCH[voice];
}


//*********************************************************************
//  Suspend/resume synth
//*********************************************************************
static void synth::suspend()
{
  CLR(ADI_TIMSK, ADI_OCIE);                            //-Stop audio interrupt
}
static void synth::resume()
{
  SET(ADI_TIMSK, ADI_OCIE);                            //-Start audio interrupt
}

#endif