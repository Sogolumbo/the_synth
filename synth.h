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

#pragma once
#ifndef _SYNTH_H
#define _SYNTH_H

enum OUTPUTMODE{
  DIFF = 1,
  CHA,
  CHB
};

enum WAVEFORMMIX{
  NO_MIX,
  TRIANGLE_AND_SAW,
  SINE_AND_TRIANGLE,
  PIANO
};

enum WAVEFORM{
  SINE,
  TRIANGLE,
  SAW,
  RAMP,
  SQUARE,
  NOISE,
  EPIANO
};

enum EnvelopeState{ 
  IDLE,
  ATTACK,
  DECAY,
  SUSTAIN,
  RELEASE
};

struct AmpEnvParams{ // Amplitude envelope params
  unsigned char attack;
  unsigned char decay;
  unsigned char sustain;
  unsigned char release;
  unsigned char releaseShape;
};

extern volatile unsigned int PITCH[4];          //-Voice pitch
extern volatile unsigned int WavePhaseAcc[4];			//-Wave phase accumulators
extern volatile unsigned int WavePhaseInc[4];           //-Wave frequency tuning words
extern volatile unsigned int wavs[4];                                  //-Wave table selector [address of wave in memory]

extern volatile unsigned int waveMix[4][32];         //-Waveform Mix Table
extern volatile unsigned int WaveMixPhaseAcc[4]; //-Waveform Mix phase (LFO phase) accumulator
extern volatile unsigned int WaveMixPhaseInc[4]; //-Waveform Mix phase (LFO speed) increment

extern volatile unsigned char AMP[4];           //-Wave amplitudes [0-255]

extern volatile EnvelopeState envelopes[4];             // selected Envelope
extern volatile AmpEnvParams ampEnvParams[4];            // Envelope parameters - attack, decay, release: [0-127], sustain: [0-255]
extern volatile unsigned int envs[4];                                  // Envelope selector [address of envelope in memory]
extern volatile unsigned int EnvelopePhaseAcc[]; //-Envelope phase accumulator
extern volatile unsigned int EnvelopePhaseInc[4];               //-Envelope speed tuning word

extern volatile int EnvelopeModulation[4];                         //-Voice envelope modulation [0-1023 512=no mod. <512 pitch down >512 pitch up]

extern volatile unsigned char divider;                             //-Sample rate decimator for envelope
extern volatile unsigned int tim;
extern volatile unsigned char tik;
extern volatile unsigned char output_mode;


class synth
{
public:

  //*********************************************************************
  //  Startup
  //*********************************************************************
  static void begin(OUTPUTMODE d);

  //*********************************************************************
  //  Timing/sequencing functions
  //*********************************************************************
  static unsigned char synthTick(void);

  //*********************************************************************
  //  Setup all voice parameters in MIDI range
  //  voice[0-3],wave[0-6],pitch[0-127],envelope[0-4],length[0-127],mod[0-127:64=no mod]
  //*********************************************************************
  static void setupVoice(unsigned char voice, 
                         WAVEFORM wave = SAW,
                         unsigned char attack = 20,
                         unsigned char decay = 40,
                         unsigned char sustain = 70,
                         unsigned char release = 90);


  //  Setup wave
  static void setWave(unsigned char voice, WAVEFORM wave);
  static void setWaveformMix(unsigned char voice, WAVEFORMMIX waveformMix);

  
  //  Setup Pitch [0-127]
  static void setPitch(unsigned char voice,unsigned char MIDInote);
  //  Set frequency direct
  static void setFrequency(unsigned char voice,float f);

  //  Setup Envelope
  static void setAmpEnvParams(unsigned char voice, 
                           unsigned char attack = 20,
                           unsigned char decay = 40,
                           unsigned char sustain = 70,
                           unsigned char release = 90,
                           unsigned char releaseShape = 0);
  static void setAmpEnvelopeState(unsigned char voice, EnvelopeState envelope);
  static bool envelopeSectionFinished(unsigned char voice);
  static void updateEnvelope(unsigned char voice);
  static void setAmpEnvelopeStatePhaseAccForRelease(unsigned char voice);


  //  Setup Length [0-128]
  static void setLength(unsigned char voice,unsigned char length);
  //  Set time
  static void setTime(unsigned char voice,float t);


  //  Setup Modulation
  static void setModChar(unsigned char voice, unsigned char mod); //mod 0..127
  static void setMod(unsigned char voice, int mod);


  //  Midi trigger
  static void mStart(unsigned char voice,unsigned char MIDInote, EnvelopeState envelope = ATTACK);
  static void mStop(unsigned char voice);

  // Simple trigger
  static void trigger(unsigned char voice);

  
  //*********************************************************************
  //  Suspend/resume synth
  //*********************************************************************
  static void suspend();
  static void resume();

};

//*********************************************************************************************
//  Audio driver interrupt
//*********************************************************************************************

SIGNAL(ADI_INTERRUPT);

#endif













