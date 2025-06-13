/**
 *******************************************************************************
 * @file    midi.h
 * @author  DEEP Project
 * @date    June 13, 2025
 * @brief   MIDI communication module for STM32G431KB
 *          Sends MIDI messages via UART2 for matrix keyboard controller
 *******************************************************************************
 */

#ifndef MIDI_H
#define MIDI_H

#include <stdint.h>
#include <stdbool.h>

/* MIDI Constants ------------------------------------------------------------*/
#define MIDI_CHANNELS           16
#define MIDI_MAX_VELOCITY       127
#define MIDI_MAX_NOTE           127

/* MIDI Status Bytes (for Channel 1, add channel-1 for other channels) */
#define MIDI_NOTE_OFF           0x80    // Note Off
#define MIDI_NOTE_ON            0x90    // Note On
#define MIDI_POLY_PRESSURE      0xA0    // Polyphonic Key Pressure
#define MIDI_CONTROL_CHANGE     0xB0    // Control Change
#define MIDI_PROGRAM_CHANGE     0xC0    // Program Change
#define MIDI_CHANNEL_PRESSURE   0xD0    // Channel Pressure
#define MIDI_PITCH_BEND         0xE0    // Pitch Bend Change

/* MIDI System Messages */
#define MIDI_SYSTEM_EXCLUSIVE   0xF0    // System Exclusive
#define MIDI_TIME_CODE          0xF1    // MIDI Time Code
#define MIDI_SONG_POSITION      0xF2    // Song Position Pointer
#define MIDI_SONG_SELECT        0xF3    // Song Select
#define MIDI_TUNE_REQUEST       0xF6    // Tune Request
#define MIDI_END_SYSEX          0xF7    // End of System Exclusive
#define MIDI_TIMING_CLOCK       0xF8    // Timing Clock
#define MIDI_START              0xFA    // Start
#define MIDI_CONTINUE           0xFB    // Continue
#define MIDI_STOP               0xFC    // Stop
#define MIDI_ACTIVE_SENSING     0xFE    // Active Sensing
#define MIDI_SYSTEM_RESET       0xFF    // System Reset

/* Common MIDI Notes (C4 = Middle C = 60) */
#define MIDI_C4                 60
#define MIDI_C3                 48
#define MIDI_C5                 72

/* Common MIDI Control Change Numbers */
#define MIDI_CC_MODULATION      1
#define MIDI_CC_VOLUME          7
#define MIDI_CC_PAN             10
#define MIDI_CC_EXPRESSION      11
#define MIDI_CC_SUSTAIN         64
#define MIDI_CC_ALL_NOTES_OFF   123

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief Initialize MIDI module
 * @retval None
 */
void MIDI_init(void);

/**
 * @brief Send MIDI Note On message
 * @param channel: MIDI channel (1-16)
 * @param note: MIDI note number (0-127)
 * @param velocity: Note velocity (1-127, 0 = Note Off)
 * @retval None
 */
void MIDI_send_note_on(uint8_t channel, uint8_t note, uint8_t velocity);

/**
 * @brief Send MIDI Note Off message
 * @param channel: MIDI channel (1-16)
 * @param note: MIDI note number (0-127)
 * @param velocity: Release velocity (0-127)
 * @retval None
 */
void MIDI_send_note_off(uint8_t channel, uint8_t note, uint8_t velocity);

/**
 * @brief Send MIDI Control Change message
 * @param channel: MIDI channel (1-16)
 * @param controller: Controller number (0-127)
 * @param value: Controller value (0-127)
 * @retval None
 */
void MIDI_send_control_change(uint8_t channel, uint8_t controller, uint8_t value);

/**
 * @brief Send MIDI Program Change message
 * @param channel: MIDI channel (1-16)
 * @param program: Program number (0-127)
 * @retval None
 */
void MIDI_send_program_change(uint8_t channel, uint8_t program);

/**
 * @brief Send MIDI Pitch Bend message
 * @param channel: MIDI channel (1-16)
 * @param value: Pitch bend value (-8192 to +8191, 0 = center)
 * @retval None
 */
void MIDI_send_pitch_bend(uint8_t channel, int16_t value);

/**
 * @brief Send raw MIDI bytes
 * @param data: Pointer to MIDI data buffer
 * @param length: Number of bytes to send
 * @retval None
 */
void MIDI_send_raw(uint8_t *data, uint8_t length);

/**
 * @brief Send MIDI All Notes Off message
 * @param channel: MIDI channel (1-16)
 * @retval None
 */
void MIDI_send_all_notes_off(uint8_t channel);

/**
 * @brief Convert note name to MIDI note number
 * @param note_name: Note name (e.g., "C4", "F#3", "Bb5")
 * @retval MIDI note number (0-127) or 255 if invalid
 */
uint8_t MIDI_note_name_to_number(const char* note_name);

/**
 * @brief Convert MIDI note number to frequency (Hz)
 * @param note: MIDI note number (0-127)
 * @retval Frequency in Hz
 */
float MIDI_note_to_frequency(uint8_t note);

#endif /* MIDI_H */
