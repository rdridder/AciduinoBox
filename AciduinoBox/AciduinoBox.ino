/*

  AciduinoBox

  The project combines Aciduino sequencer and AcidBox synth combo.
  Finally you have a sequencer 2 x TB-303 + 

*/
#pragma GCC optimize ("O2")

#include "config.h"
#include "fx_delay.h"
#ifndef NO_PSRAM
#include "fx_reverb.h"
#endif
#include "compressor.h"
#include "synthvoice.h"
#include "sampler.h"
#include <Wire.h>

//
// Aciduino
//
#include "src/aciduino.hpp"

#include "src/ports/esp32/s3.h"

// lookuptables
static float DRAM_ATTR WORD_ALIGNED_ATTR midi_pitches[128];
static float DRAM_ATTR WORD_ALIGNED_ATTR  midi_phase_steps[128];
static float DRAM_ATTR WORD_ALIGNED_ATTR  midi_tbl_steps[128];
static float DRAM_ATTR WORD_ALIGNED_ATTR  exp_square_tbl[TABLE_SIZE+1];
//static float square_tbl[TABLE_SIZE+1];
static float DRAM_ATTR WORD_ALIGNED_ATTR  saw_tbl[TABLE_SIZE+1];
static float DRAM_ATTR WORD_ALIGNED_ATTR  exp_tbl[TABLE_SIZE+1];
static float DRAM_ATTR WORD_ALIGNED_ATTR  knob_tbl[TABLE_SIZE+1]; // exp-like curve
static float DRAM_ATTR WORD_ALIGNED_ATTR  shaper_tbl[TABLE_SIZE+1]; // illinear tanh()-like curve
static float DRAM_ATTR WORD_ALIGNED_ATTR  lim_tbl[TABLE_SIZE+1]; // diode soft clipping at about 1.0
static float DRAM_ATTR WORD_ALIGNED_ATTR  sin_tbl[TABLE_SIZE+1];
static float DRAM_ATTR WORD_ALIGNED_ATTR  norm1_tbl[16][16]; // cutoff-reso pair gain compensation
static float DRAM_ATTR WORD_ALIGNED_ATTR  norm2_tbl[16][16]; // wavefolder-overdrive gain compensation
//static float (*tables[])[TABLE_SIZE+1] = {&exp_square_tbl, &square_tbl, &saw_tbl, &exp_tbl};

// service variables and arrays
volatile uint32_t s1t, s2t, drt, fxt, s1T, s2T, drT, fxT, art, arT, c0t, c0T, c1t, c1T; // debug timing: if we use less vars, compiler optimizes them
volatile uint32_t prescaler;
static  uint32_t  last_reset = 0;
static  float     param[POT_NUM];
static int    ctrl_hold_notes;

// Audio buffers of all kinds
volatile int current_gen_buf = 0; // set of buffers for generation
volatile int current_out_buf = 1 - 0; // set of buffers for output
static float DRAM_ATTR WORD_ALIGNED_ATTR  synth1_buf[2][DMA_BUF_LEN];    // synth1 mono
static float DRAM_ATTR WORD_ALIGNED_ATTR  synth2_buf[2][DMA_BUF_LEN];    // synth2 mono
static float DRAM_ATTR WORD_ALIGNED_ATTR  drums_buf_l[2][DMA_BUF_LEN];   // drums L
static float DRAM_ATTR WORD_ALIGNED_ATTR  drums_buf_r[2][DMA_BUF_LEN];   // drums R
static float DRAM_ATTR WORD_ALIGNED_ATTR  mix_buf_l[2][DMA_BUF_LEN];     // mix L channel
static float DRAM_ATTR WORD_ALIGNED_ATTR  mix_buf_r[2][DMA_BUF_LEN];     // mix R channel
static union {                              // instead of true converting
  int16_t WORD_ALIGNED_ATTR _signed[DMA_BUF_LEN * 2];
  uint16_t WORD_ALIGNED_ATTR _unsigned[DMA_BUF_LEN * 2];
} out_buf[2];                               // i2s L+R output buffer
size_t bytes_written;                       // i2s result

volatile boolean processing = false;
#ifndef NO_PSRAM
volatile float rvb_k1, rvb_k2, rvb_k3;
#endif
volatile float dly_k1, dly_k2, dly_k3;

// tasks for Core0 and Core1
TaskHandle_t SynthTask1;
TaskHandle_t SynthTask2;

// 303-like synths
SynthVoice Synth1(0); 
SynthVoice Synth2(1); 

// 808-like drums
Sampler Drums( DEFAULT_DRUMKIT ); // argument: starting drumset [0 .. total-1]

// Global effects
FxDelay Delay;
#ifndef NO_PSRAM
FxReverb Reverb;
#endif
Compressor Comp;


/* 
 * Core Tasks ************************************************************************************************************************
*/
// Core0 task 
// static void audio_task1(void *userData) {
static void IRAM_ATTR audio_task1(void *userData) {
  DEBUG("Task @Core0 started");
  delay(10);  
  while (true) {
    taskYIELD(); 
//    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) { // we need all the generators to fill the buffers here, so we wait
      c0t = micros();
      
//      taskYIELD(); 
      
      current_gen_buf = current_out_buf;      // swap buffers
      current_out_buf = 1 - current_gen_buf;
      
    //  xTaskNotifyGive(SynthTask2);            // if we are here, then we've already received a notification from task2
      
      s1t = micros();
      synth1_generate();
      s1T = micros() - s1t;
      
      s2t = micros();
      synth2_generate();
      s2T = micros() - s2t;
      
  //    taskYIELD(); 

      drt = micros();
      drums_generate();
      drT = micros() - drt;
      
      c1t = micros();
      fxt = micros();
      mixer(); 
      fxT = micros() - fxt;
      
      i2s_output();

 //   }
    
   // taskYIELD();

    taskYIELD();

    c0T = micros() - c0t;
  }
}

// task for Core1, which tipically runs user's code on ESP32
// static void IRAM_ATTR audio_task2(void *userData) {
static void IRAM_ATTR audio_task2(void *userData) {
  size_t prescaler = 0;
  DEBUG("Task @Core1 started");
  delay(10);
  while (true) {
    taskYIELD();
 
    c1t = micros();
    
      
    prescaler++;
    if (prescaler % 128 == 0) {
#ifdef TEST_POTS      
      readPots();
#endif

      // aciduino.run(); // we can place it here, if it's not to run that often
      // regular_checks(); // or inside of this function
      
#ifdef DEBUG_TIMING
        DEBF ("CORE micros: synt1, synt2, drums, mixer, DMA_LEN\t%d\t%d\t%d\t%d\t%d\r\n" , s1T, s2T, drT, fxT, DMA_BUF_TIME);
        //    DEBF ("TaskCore0=%dus TaskCore1=%dus DMA_BUF=%dus\r\n" , c0T , c1T , DMA_BUF_TIME);
        //    DEBF ("AllTheRestCore1=%dus\r\n" , arT);
#endif
    }
    
    c1T = micros() - c1t;
  }
}


/* 
 *  Quite an ordinary SETUP() *******************************************************************************************************************************
*/

// Acidbox
inline void handle_acid_midi_events(uint8_t msg_type, uint8_t byte1, uint8_t byte2, uint8_t channel, uint8_t port) {
  switch(msg_type) {
    case NOTE_ON:
      if(channel == 0) {
        Synth1.on_midi_noteON(byte1, byte2);
      } else if(channel == 1) {
        Synth2.on_midi_noteON(byte1, byte2);
      } else if(channel == 4) {
        Drums.NoteOn(byte1, byte2);
      }
      break;
    case NOTE_OFF:
      if(channel == 0) {
        Synth1.on_midi_noteOFF(byte1, byte2);
      } else if(channel == 1) {
        Synth2.on_midi_noteOFF(byte1, byte2);
      } else if(channel == 4) {
        Drums.NoteOff(byte1);
      }
      break;      
  }
}

void setup(void) {

#ifdef DEBUG_ON 
  DEBUG_PORT.begin(115200); 
#endif
  delay(200); // let the serial start
  DEBUG("=====> Starting setup");

  btStop(); // we don't want bluetooth to consume our precious cpu time 

  MidiInit(); // init midi input and handling of midi events

  /*
    for (int i = 0; i < GPIO_BUTTONS; i++) {
    pinMode(buttonGPIOs[i], INPUT_PULLDOWN);
    }
  */

  buildTables();

  for (int i = 0; i < POT_NUM; i++) pinMode( POT_PINS[i] , INPUT);

  Synth1.Init();
  Synth2.Init();
  Drums.Init();
#ifndef NO_PSRAM
  Reverb.Init();
#endif
  Delay.Init();
  Comp.Init(SAMPLE_RATE); 

  // silence while we haven't loaded anything reasonable
  for (int i = 0; i < DMA_BUF_LEN; i++) {
    drums_buf_l[current_gen_buf][i] = 0.0f ;
    drums_buf_r[current_gen_buf][i] = 0.0f ;
    synth1_buf[current_gen_buf][i] = 0.0f ;
    synth2_buf[current_gen_buf][i] = 0.0f ;
    out_buf[current_out_buf]._signed[i * 2] = 0 ;
    out_buf[current_out_buf]._signed[i * 2 + 1] = 0 ;
    mix_buf_l[current_out_buf][i] = 0.0f;
    mix_buf_r[current_out_buf][i] = 0.0f;
  }

  i2sInit();
  
  // inits all hardware setup for the selected port
  DEBUG("=====> Init ports aciduino");
  initPort();
  aciduino.setAcidBoxOutputCallback(handle_acid_midi_events);

  DEBUG("=====> Start synth tasks");
  xTaskCreatePinnedToCore( audio_task1, "SynthTask1", 5000, NULL, 1, &SynthTask1, 0 );
 // xTaskCreatePinnedToCore( audio_task2, "SynthTask2", 10000, NULL, 1, &SynthTask2, 1 );

  // somehow we should allow tasks to run
  // xTaskNotifyGive(SynthTask1);
  // xTaskNotifyGive(SynthTask2);
  processing = true;

DEBUG("=====> Setup done");
}

static uint32_t last_ms = micros();

/* 
 *  Finally, the LOOP () ***********************************************************************************************************
*/

void loop() { // default loopTask running on the Core1
//  static size_t counter = 0;
//  if (counter % 1024 == 0) {DEBUG("I am alive");}
  aciduino.run();   
  taskYIELD(); // this can wait
// processButtons();  
// vTaskDelete(NULL);
}

/* 
 *  Some debug and service routines *****************************************************************************************************************************
*/

void readPots() {
  static const float snap = 0.003f;
  static int i = 0;
  float tmp;
  static const float NORMALIZE_ADC = 1.0f / 4096.0f;
//read one pot per call
  tmp = (float)analogRead(POT_PINS[i]) * NORMALIZE_ADC;
  if (fabs(tmp - param[i]) > snap) {
    param[i] = tmp;
  //  paramChange(i, tmp);
  }

  i++;
  // if (i >= POT_NUM) i=0;
  i %= POT_NUM;
}

void paramChange(uint8_t paramNum, float paramVal) {
  // paramVal === param[paramNum];
  DEBF ("param %d val %0.4f\r\n" , paramNum, paramVal);
  paramVal *= 127.0;
  switch (paramNum) {
    case 0:
      //set_bpm( 40.0f + (paramVal * 160.0f));
      Synth2.ParseCC(CC_303_CUTOFF, paramVal);
      break;
    case 1:
      Synth2.ParseCC(CC_303_RESO, paramVal);
      break;
    case 2:
      Synth2.ParseCC(CC_303_OVERDRIVE, paramVal);
      Synth2.ParseCC(CC_303_DISTORTION, paramVal);
      break;
    case 3:
      Synth2.ParseCC(CC_303_ENVMOD_LVL, paramVal);
      break;
    case 4:
      Synth2.ParseCC(CC_303_ACCENT_LVL, paramVal);
      break;
    default:
      {}
  }
}


void regular_checks() {
    // place some code that won't require every single tick of core1 time
}


inline void  drums_generate() {
    for (int i=0; i < DMA_BUF_LEN; i++){
      Drums.Process( &drums_buf_l[current_gen_buf][i], &drums_buf_r[current_gen_buf][i] );      
    } 
}

inline void  synth1_generate() {
    for (int i=0; i < DMA_BUF_LEN; i++){
      synth1_buf[current_gen_buf][i] = Synth1.getSample() ;      
    } 
}

inline void  synth2_generate() {
    for (int i=0; i < DMA_BUF_LEN; i++){
      synth2_buf[current_gen_buf][i] = Synth2.getSample() ;      
    } 
}

void  mixer() { // sum buffers 
#ifdef DEBUG_MASTER_OUT
  static float meter = 0.0f;
#endif
  static float synth1_out_l, synth1_out_r, synth2_out_l, synth2_out_r, drums_out_l, drums_out_r;
  static float dly_l, dly_r, rvb_l, rvb_r;
  static float mono_mix;
    dly_k1 = Synth1._sendDelay;
    dly_k2 = Synth2._sendDelay;
    dly_k3 = Drums._sendDelay;
#ifndef NO_PSRAM 
    rvb_k1 = Synth1._sendReverb;
    rvb_k2 = Synth2._sendReverb;
    rvb_k3 = Drums._sendReverb;
#endif
    for (int i=0; i < DMA_BUF_LEN; i++) { 
      drums_out_l = drums_buf_l[current_out_buf][i];
      drums_out_r = drums_buf_r[current_out_buf][i];

      synth1_out_l = Synth1.GetPan() * synth1_buf[current_out_buf][i];
      synth1_out_r = (1.0f - Synth1.GetPan()) * synth1_buf[current_out_buf][i];
      synth2_out_l = Synth2.GetPan() * synth2_buf[current_out_buf][i];
      synth2_out_r = (1.0f - Synth2.GetPan()) * synth2_buf[current_out_buf][i];

      
      dly_l = dly_k1 * synth1_out_l + dly_k2 * synth2_out_l + dly_k3 * drums_out_l; // delay bus
      dly_r = dly_k1 * synth1_out_r + dly_k2 * synth2_out_r + dly_k3 * drums_out_r;
      Delay.Process( &dly_l, &dly_r );
#ifndef NO_PSRAM
      rvb_l = rvb_k1 * synth1_out_l + rvb_k2 * synth2_out_l + rvb_k3 * drums_out_l; // reverb bus
      rvb_r = rvb_k1 * synth1_out_r + rvb_k2 * synth2_out_r + rvb_k3 * drums_out_r;
      Reverb.Process( &rvb_l, &rvb_r );

      mix_buf_l[current_out_buf][i] = (synth1_out_l + synth2_out_l + drums_out_l + dly_l + rvb_l);
      mix_buf_r[current_out_buf][i] = (synth1_out_r + synth2_out_r + drums_out_r + dly_r + rvb_r);
#else
      mix_buf_l[current_out_buf][i] = (synth1_out_l + synth2_out_l + drums_out_l + dly_l);
      mix_buf_r[current_out_buf][i] = (synth1_out_r + synth2_out_r + drums_out_r + dly_r);
#endif
      mono_mix = 0.5f * (mix_buf_l[current_out_buf][i] + mix_buf_r[current_out_buf][i]);
  //    Comp.Process(mono_mix);     // calculate gain based on a mono mix

      Comp.Process(drums_out_l*0.25f);  // calc compressor gain, side-chain driven by drums


      mix_buf_l[current_out_buf][i] = (Comp.Apply( 0.25f * mix_buf_l[current_out_buf][i]));
      mix_buf_r[current_out_buf][i] = (Comp.Apply( 0.25f * mix_buf_r[current_out_buf][i]));

      
#ifdef DEBUG_MASTER_OUT
      if ( i % 16 == 0) meter = meter * 0.95f + fabs( mono_mix); 
#endif
  //    mix_buf_l[current_out_buf][i] = fclamp(mix_buf_l[current_out_buf][i] , -1.0f, 1.0f); // clipper
  //    mix_buf_r[current_out_buf][i] = fclamp(mix_buf_r[current_out_buf][i] , -1.0f, 1.0f);
     mix_buf_l[current_out_buf][i] = fast_shape( mix_buf_l[current_out_buf][i]); // soft limitter/saturator
     mix_buf_r[current_out_buf][i] = fast_shape( mix_buf_r[current_out_buf][i]);
   }
#ifdef DEBUG_MASTER_OUT
  meter *= 0.95f;
  meter += fabs(mono_mix); 
  DEBF("out= %0.5f\r\n", meter);
#endif
}
