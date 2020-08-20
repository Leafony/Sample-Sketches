#define LEDC_CHANNEL_0     0      // use first channel of 16 channels (started from zero)
#define LEDC_TIMER_13_BIT  13     // use 13 bit precission for LEDC timer
#define LEDC_BASE_FREQ     5000   // use 5000 Hz as a LEDC base frequency

//------------------------------
// buzzer output = 13pin
//------------------------------
#define BUZZER_OUT 13

const int NOTE_NONE = NOTE_MAX;

// notes in the melody:
int melody[] = {
  NOTE_C, NOTE_C, NOTE_G, NOTE_G, NOTE_A, NOTE_A, NOTE_G, NOTE_F, NOTE_F, NOTE_E, NOTE_E,NOTE_D,NOTE_D,NOTE_C
};

int noteOctaves[] = {
  5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 4, 4, 4, 4, 4, 2, 4, 4, 4, 4, 4, 4, 2
};

//=====================================================================
// setup
//=====================================================================
void setup() {
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(BUZZER_OUT, LEDC_CHANNEL_0);

  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 14; thisNote++) {
    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    ledcWriteNote(LEDC_CHANNEL_0, (note_t)melody[thisNote], noteOctaves[thisNote]);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = 1000 / noteDurations[thisNote] * 1.30;
    delay(pauseBetweenNotes);

    // stop the tone playing:
    ledcWriteTone(LEDC_CHANNEL_0, 0);
    delay(30);
  }
}

//=====================================================================
// Main loop
//=====================================================================
void loop() {
  // no need to repeat the melody.
}
