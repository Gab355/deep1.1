/**
 *******************************************************************************
 * @file    stm32g4_matrix_keyboard.h
 * @author  vchav - Updated for MIDI support
 * @date    Jun 13, 2025
 * @brief   Header for 8x8 matrix keyboard with MCP23017
 *          Added support for full state reading for MIDI polyphony
 *******************************************************************************
 */

#ifndef STM32G4_MATRIX_KEYBOARD_H
#define STM32G4_MATRIX_KEYBOARD_H

#include "config.h"

#if USE_MATRIX_KEYBOARD

#include <stdint.h>
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
#define NO_KEY      0x00
#define MANY_KEYS   0xFF

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Initialize the matrix keyboard with custom key mapping
 * @param new_keyboard_keys: Array of 64 characters for key mapping
 */
void BSP_MATRIX_KEYBOARD_init(const char *new_keyboard_keys);

/**
 * @brief Get a single key press (legacy function)
 * @retval Character corresponding to pressed key, NO_KEY if none, MANY_KEYS if multiple
 */
char MATRIX_KEYBOARD_get_key(void);

/**
 * @brief Read complete state of all keys in the matrix
 * @retval 32-bit value where each bit represents a key state (1=pressed, 0=released)
 *         Bit 0 = position (1,1), Bit 1 = position (1,2), etc.
 *         Bit 8 = position (2,1), Bit 9 = position (2,2), etc.
 */
uint32_t MATRIX_KEYBOARD_read_all_touchs(void);

/**
 * @brief Demo function to show all pressed keys
 */
void BSP_MATRIX_KEYBOARD_demo_process_main(void);

/**
 * @brief Test I2C communication with MCP23017
 */
void BSP_MATRIX_KEYBOARD_test_i2c(void);

#endif /* USE_MATRIX_KEYBOARD */

#endif /* STM32G4_MATRIX_KEYBOARD_H */
