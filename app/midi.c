/**
 *******************************************************************************
 * @file    midi.c
 * @author  DEEP Project
 * @date    June 13, 2025
 * @brief   MIDI communication module implementation
 *          Sends MIDI messages via UART2 for matrix keyboard controller
 *******************************************************************************
 */

#include "midi.h"
#include "config.h"
#include "stm32g4_uart.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define MIDI_TIMEOUT_MS         100
#define MIDI_UART_ID            UART2_ID

/* Private variables ---------------------------------------------------------*/
static bool midi_initialized = false;

/**
 * @brief Initialize MIDI module
 */
void MIDI_init(void)
{
    /* UART is already initialized in main.c via BSP_UART_init() */
    midi_initialized = true;

    /* Send initial MIDI reset to clear any stuck notes */
    MIDI_send_all_notes_off(1);

    printf("MIDI module initialized\r\n");
}

/**
 * @brief Send raw MIDI bytes via UART
 * @param data: Pointer to MIDI data buffer
 * @param length: Number of bytes to send
 */
void MIDI_send_raw(uint8_t *data, uint8_t length)
{
    if (!midi_initialized || data == NULL || length == 0) {
        return;
    }

    /* Send MIDI data via UART2 */
    for (uint8_t i = 0; i < length; i++) {
        BSP_UART_putc(MIDI_UART_ID, data[i]);
    }
}

/**
 * @brief Send MIDI Note On message
 * @param channel: MIDI channel (1-16)
 * @param note: MIDI note number (0-127)
 * @param velocity: Note velocity (1-127)
 */
void MIDI_send_note_on(uint8_t channel, uint8_t note, uint8_t velocity)
{
    if (channel < 1 || channel > 16 || note > 127 || velocity > 127) {
        return;
    }

    uint8_t midi_data[3];
    midi_data[0] = MIDI_NOTE_ON | (channel - 1);  // Status byte with channel
    midi_data[1] = note;                          // Note number
    midi_data[2] = velocity;                      // Velocity

    MIDI_send_raw(midi_data, 3);
}

/**
 * @brief Send MIDI Note Off message
 * @param channel: MIDI channel (1-16)
 * @param note: MIDI note number (0-127)
 * @param velocity: Release velocity (0-127)
 */
void MIDI_send_note_off(uint8_t channel, uint8_t note, uint8_t velocity)
{
    if (channel < 1 || channel > 16 || note > 127 || velocity > 127) {
        return;
    }

    uint8_t midi_data[3];
    midi_data[0] = MIDI_NOTE_OFF | (channel - 1); // Status byte with channel
    midi_data[1] = note;                          // Note number
    midi_data[2] = velocity;                      // Release velocity

    MIDI_send_raw(midi_data, 3);
}

/**
 * @brief Send MIDI Control Change message
 * @param channel: MIDI channel (1-16)
 * @param controller: Controller number (0-127)
 * @param value: Controller value (0-127)
 */
void MIDI_send_control_change(uint8_t channel, uint8_t controller, uint8_t value)
{
    if (channel < 1 || channel > 16 || controller > 127 || value > 127) {
        return;
    }

    uint8_t midi_data[3];
    midi_data[0] = MIDI_CONTROL_CHANGE | (channel - 1);
    midi_data[1] = controller;
    midi_data[2] = value;

    MIDI_send_raw(midi_data, 3);
}

/**
 * @brief Send MIDI Program Change message
 * @param channel: MIDI channel (1-16)
 * @param program: Program number (0-127)
 */
void MIDI_send_program_change(uint8_t channel, uint8_t program)
{
    if (channel < 1 || channel > 16 || program > 127) {
        return;
    }

    uint8_t midi_data[2];
    midi_data[0] = MIDI_PROGRAM_CHANGE | (channel - 1);
    midi_data[1] = program;

    MIDI_send_raw(midi_data, 2);
}

/**
 * @brief Send MIDI Pitch Bend message
 * @param channel: MIDI channel (1-16)
 * @param value: Pitch bend value (-8192 to +8191, 0 = center)
 */
void MIDI_send_pitch_bend(uint8_t channel, int16_t value)
{
    if (channel < 1 || channel > 16) {
        return;
    }

    /* Convert signed value to 14-bit unsigned (0-16383, 8192 = center) */
    uint16_t bend_value = (uint16_t)(value + 8192);
    if (bend_value > 16383) bend_value = 16383;

    uint8_t midi_data[3];
    midi_data[0] = MIDI_PITCH_BEND | (channel - 1);
    midi_data[1] = bend_value & 0x7F;        // LSB (7 bits)
    midi_data[2] = (bend_value >> 7) & 0x7F; // MSB (7 bits)

    MIDI_send_raw(midi_data, 3);
}

/**
 * @brief Send MIDI All Notes Off message
 * @param channel: MIDI channel (1-16)
 */
void MIDI_send_all_notes_off(uint8_t channel)
{
    MIDI_send_control_change(channel, MIDI_CC_ALL_NOTES_OFF, 0);
}

/**
 * @brief Convert note name to MIDI note number
 * @param note_name: Note name (e.g., "C4", "F#3", "Bb5")
 * @retval MIDI note number (0-127) or 255 if invalid
 */
uint8_t MIDI_note_name_to_number(const char* note_name)
{
    if (note_name == NULL || strlen(note_name) < 2) {
        return 255;
    }

    /* Note names to semitone offset from C */
    const char notes[] = "C C#D D#E F F#G G#A A#B ";
    const char *note_ptr = NULL;
    int semitone = -1;

    /* Handle flat notation (convert to sharp) */
    char note_char = note_name[0];
    char accidental = (strlen(note_name) > 2) ? note_name[1] : '\0';

    if (accidental == 'b') {
        /* Convert flat to equivalent sharp */
        switch (note_char) {
            case 'D': note_char = 'C'; accidental = '#'; break;
            case 'E': note_char = 'D'; accidental = '#'; break;
            case 'G': note_char = 'F'; accidental = '#'; break;
            case 'A': note_char = 'G'; accidental = '#'; break;
            case 'B': note_char = 'A'; accidental = '#'; break;
            default: return 255; // Invalid flat note
        }
    }

    /* Find semitone offset */
    for (int i = 0; i < 12; i++) {
        if (notes[i*2] == note_char) {
            if (accidental == '#' && notes[i*2 + 1] == '#') {
                semitone = i;
                break;
            } else if (accidental != '#' && notes[i*2 + 1] == ' ') {
                semitone = i;
                break;
            }
        }
    }

    if (semitone == -1) return 255;

    /* Extract octave number */
    int octave_pos = (accidental == '#' || accidental == 'b') ? 2 : 1;
    if (strlen(note_name) <= octave_pos) return 255;

    int octave = note_name[octave_pos] - '0';
    if (octave < 0 || octave > 9) return 255;

    /* Calculate MIDI note number (C4 = 60) */
    int midi_note = (octave + 1) * 12 + semitone;

    return (midi_note >= 0 && midi_note <= 127) ? midi_note : 255;
}

/**
 * @brief Convert MIDI note number to frequency (Hz)
 * @param note: MIDI note number (0-127)
 * @retval Frequency in Hz
 */
float MIDI_note_to_frequency(uint8_t note)
{
    if (note > 127) return 0.0f;

    /* A4 (MIDI note 69) = 440 Hz */
    /* Formula: f = 440 * 2^((n-69)/12) */
    return 440.0f * powf(2.0f, (float)(note - 69) / 12.0f);
}
