/*
  Melody (Twinkle Twinkle Little Star)

  Plays a melody

  created 21 Jan 2010
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Tone
*/
//=====================================================================
#include "pitches.h"
//------------------------------
// buzzer output = 5pin
//------------------------------
#define BUZZER_OUT 5

//-----------------------------------------------
// IOピンの名前定義
// 接続するリーフに合わせて定義する
//-----------------------------------------------
#define LED1              6
#define LED2              7
#define LED3              8
#define LED4              9
#define LED5              10
#define LED6              11

//-----------------------------------------------
// notes in the melody:
//-----------------------------------------------
int melody[] = {
  NOTE_C5, NOTE_C5, NOTE_G5, NOTE_G5, NOTE_A5, NOTE_A5, NOTE_G5, NOTE_F5, NOTE_F5, NOTE_E5, NOTE_E5,NOTE_D5,NOTE_D5,NOTE_C5
};

//-----------------------------------------------
// note durations: 4 = quarter note, 8 = eighth note, etc.:
//-----------------------------------------------
int noteDurations[] = {
  1, 4, 4, 4, 4, 4, 2, 4, 4, 4, 4, 4, 4, 2
};

//=====================================================================
// setup
//=====================================================================
void setup(){
  setupPort();

  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 14; thisNote++) {

    switch (noteDurations[thisNote]){
      case 16:          // 16分音符
        digitalWrite(LED1,LOW);
        digitalWrite(LED2,LOW);
        digitalWrite(LED3,LOW);
        digitalWrite(LED4,LOW);
        digitalWrite(LED5,LOW);
        digitalWrite(LED6,HIGH);
      break;
      case 8:         // 8分音符
        digitalWrite(LED1,LOW);
        digitalWrite(LED2,LOW);
        digitalWrite(LED3,LOW);
        digitalWrite(LED4,LOW);
        digitalWrite(LED5,HIGH);
        digitalWrite(LED6,HIGH);
      break;
      case 4:         // 4分音符
        digitalWrite(LED1,LOW);
        digitalWrite(LED2,LOW);
        digitalWrite(LED3,LOW);
        digitalWrite(LED4,HIGH);
        digitalWrite(LED5,HIGH);
        digitalWrite(LED6,HIGH);
      break;
      case 2:         // 2分音符
        digitalWrite(LED1,LOW);
        digitalWrite(LED2,LOW);
        digitalWrite(LED3,HIGH);
        digitalWrite(LED4,HIGH);
        digitalWrite(LED5,HIGH);
        digitalWrite(LED6,HIGH);
      break;
      case 1:         // 全音符
        digitalWrite(LED1,LOW);
        digitalWrite(LED2,HIGH);
        digitalWrite(LED3,HIGH);
        digitalWrite(LED4,HIGH);
        digitalWrite(LED5,HIGH);
        digitalWrite(LED6,HIGH);
      break;
      default:          // その他
        digitalWrite(LED1,HIGH);
        digitalWrite(LED2,HIGH);
        digitalWrite(LED3,HIGH);
        digitalWrite(LED4,HIGH);
        digitalWrite(LED5,HIGH);
        digitalWrite(LED6,HIGH);
      break;
    }

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZER_OUT, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(BUZZER_OUT);
  }

// LED off
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3,LOW);
  digitalWrite(LED4,LOW);
  digitalWrite(LED5,LOW);
  digitalWrite(LED6,LOW);
}

//=====================================================================
// IOピンの入出力設定
// 接続するリーフに合わせて設定する
//=====================================================================
void setupPort(){
  pinMode(LED1, OUTPUT);       // PD6 : digital 6 = LED1
  digitalWrite(LED1, LOW);
  pinMode(LED2, OUTPUT);       // PD7 : digital 7 = LED2
  digitalWrite(LED2, LOW);
  pinMode(LED3, OUTPUT);        // PB0 : digital 8 = LED3
  digitalWrite(LED3, LOW);
  pinMode(LED4, OUTPUT);        // PB1 : digital 9 = LED4
  digitalWrite(LED4, LOW);
  pinMode(LED5, OUTPUT);        // PB2 : digital 10 = LED5
  digitalWrite(LED5, LOW);
  pinMode(LED6, OUTPUT);        // PB3 : digital 11 = LED6
  digitalWrite(LED6, LOW);
}

//=====================================================================
// Main loop
//=====================================================================
void loop() {
  // no need to repeat the melody.
}
