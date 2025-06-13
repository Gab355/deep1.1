/**
 *******************************************************************************
 * @file    stm32g4_matrix_keyboard.c
 * @author  vchav - Fixed version with public state function
 * @date    Jun 13, 2025
 * @brief   Corrected 8x8 matrix keyboard version (MCP23017)
 *          Added public function for full state reading (MIDI support)
 *******************************************************************************
 */

#include "config.h"
#if USE_MATRIX_KEYBOARD
#include "stm32g4_matrix_keyboard.h"
#include "../MCP23017/stm32g4_mcp23017.h"
#include "stm32g4_systick.h"
#include <stdbool.h>
#include <stdio.h>

#define MATRIX_ROWS 8
#define MATRIX_COLS 8
#define SCAN_DELAY_MS 2  // Délai entre les scans pour stabiliser

// Mapping dynamique
char keyboard_keys[MATRIX_ROWS * MATRIX_COLS];
static bool initialized = false;
static MCP23017_id_t keyboard_chip_id;

void BSP_MATRIX_KEYBOARD_init(const char * new_keyboard_keys)
{
    printf("Initializing matrix keyboard...\n");

    // Initialisation du MCP23017
    MCP23017_init();

    // Ajout du chip avec l'adresse correcte (A0/A1/A2 = GND = 0b000)
    keyboard_chip_id = MCP23017_add(I2C1, 0b000);
    printf("MCP23017 added with ID: %d\n", keyboard_chip_id);

    // Configuration PORTA comme SORTIE (colonnes) - initialement toutes à HIGH
    for (int col = 0; col < MATRIX_COLS; col++) {
        MCP23017_setIO(keyboard_chip_id, MCP23017_PORT_A, 1 << col, MCP23017_DIRECTION_OUTPUT);
        MCP23017_setGPIO(keyboard_chip_id, MCP23017_PORT_A, 1 << col, MCP23017_PIN_STATE_HIGH);
    }

    // Configuration PORTB comme ENTRÉE (lignes) avec pull-ups
    for (int row = 0; row < MATRIX_ROWS; row++) {
        MCP23017_setIO(keyboard_chip_id, MCP23017_PORT_B, 1 << row, MCP23017_DIRECTION_INPUT);
        MCP23017_setPullUp(keyboard_chip_id, MCP23017_PORT_B, 1 << row, MCP23017_PULL_UP_STATE_HIGH);
    }

    // Délai pour stabiliser la configuration
    HAL_Delay(10);

    // Copie du mapping des touches
    if (new_keyboard_keys) {
        for (int i = 0; i < MATRIX_ROWS * MATRIX_COLS; i++) {
            keyboard_keys[i] = new_keyboard_keys[i];
        }
    } else {
        for (int i = 0; i < MATRIX_ROWS * MATRIX_COLS; i++) {
            keyboard_keys[i] = ' ';
        }
    }

    initialized = true;
    printf("Matrix keyboard initialized successfully!\n");
}

uint32_t MATRIX_KEYBOARD_read_all_touchs(void)
{
    if (!initialized) return 0;

    uint32_t state = 0;

    // Remettre toutes les colonnes à HIGH avant de commencer
    for (int col = 0; col < MATRIX_COLS; col++) {
        MCP23017_setGPIO(keyboard_chip_id, MCP23017_PORT_A, 1 << col, MCP23017_PIN_STATE_HIGH);
    }
    HAL_Delay(SCAN_DELAY_MS);

    // Balayage colonne par colonne
    for (int col = 0; col < MATRIX_COLS; col++) {
        // Mettre la colonne courante à LOW, les autres à HIGH
        for (int c = 0; c < MATRIX_COLS; c++) {
            MCP23017_setGPIO(keyboard_chip_id, MCP23017_PORT_A, 1 << c,
                             (c == col) ? MCP23017_PIN_STATE_LOW : MCP23017_PIN_STATE_HIGH);
        }

        // Attendre la stabilisation
        HAL_Delay(SCAN_DELAY_MS);

        // Lire toutes les lignes d'un coup
        uint8_t port_b_value = 0;
        for (int pin = 0; pin < 8; pin++) {
            MCP23017_pinState_e pin_state;
            if (MCP23017_getGPIO(keyboard_chip_id, MCP23017_PORT_B, pin, &pin_state)) {
                if (pin_state == MCP23017_PIN_STATE_HIGH) {
                    port_b_value |= (1 << pin);
                }
            }
        }

        // Analyser chaque ligne
        for (int row = 0; row < MATRIX_ROWS; row++) {
            // Si la ligne est à LOW, alors la touche est pressée
            if (!(port_b_value & (1 << row))) {
                int key_index = row * MATRIX_COLS + col;
                if (key_index < 32) {  // Limité à 32 touches pour uint32_t
                    state |= (1UL << key_index);
                }

                // Debug optionnel (commenter pour éviter spam)
                // printf("Key detected: row=%d, col=%d, index=%d\n", row, col, key_index);
            }
        }
    }

    // Remettre toutes les colonnes à HIGH à la fin
    for (int col = 0; col < MATRIX_COLS; col++) {
        MCP23017_setGPIO(keyboard_chip_id, MCP23017_PORT_A, 1 << col, MCP23017_PIN_STATE_HIGH);
    }

    return state;
}

char MATRIX_KEYBOARD_get_key(void)
{
    if (!initialized) return NO_KEY;

    uint32_t touchs = MATRIX_KEYBOARD_read_all_touchs();

    if (touchs == 0) return NO_KEY;

    // Vérifier si plusieurs touches sont pressées (anti-ghosting basique)
    int count = 0;
    int first_key = -1;

    for (int i = 0; i < MATRIX_ROWS * MATRIX_COLS && i < 32; i++) {
        if (touchs & (1UL << i)) {
            count++;
            if (first_key == -1) first_key = i;
        }
    }

    if (count > 1) {
        printf("Multiple keys detected (%d keys)\n", count);
        return MANY_KEYS;
    }

    return (first_key >= 0) ? keyboard_keys[first_key] : NO_KEY;
}

void BSP_MATRIX_KEYBOARD_demo_process_main(void)
{
    if (!initialized) {
        printf("ERROR: Keyboard not initialized!\n");
        return;
    }

    static uint32_t last_state = 0;
    uint32_t current_state = MATRIX_KEYBOARD_read_all_touchs();

    // N'afficher que s'il y a un changement d'état
    if (current_state != last_state) {
        if (current_state == 0) {
            printf("All keys released.\n");
        } else {
            printf("Keys pressed - State: 0x%08lX\n", current_state);

            int count = 0;
            for (int i = 0; i < MATRIX_ROWS * MATRIX_COLS && i < 32; i++) {
                if (current_state & (1UL << i)) {
                    count++;
                    int row = i / MATRIX_COLS;
                    int col = i % MATRIX_COLS;
                    char key_char = keyboard_keys[i];

                    printf("  - Position (%d,%d)", row + 1, col + 1);  // +1 pour affichage lisible
                    if (key_char >= 32 && key_char <= 126) {
                        printf(" = '%c'", key_char);
                    }
                    printf("\n");
                }
            }

            if (count == 1) {
                // Une seule touche - récupérer le caractère
                char key = MATRIX_KEYBOARD_get_key();
                if (key != NO_KEY && key != MANY_KEYS) {
                    printf("Single key: '%c'\n", key);
                }
            } else {
                printf("Multiple keys detected (%d) - polyphonic!\n", count);
            }
        }

        last_state = current_state;
    }
}

// Fonction utilitaire pour test de connectivité I2C
void BSP_MATRIX_KEYBOARD_test_i2c(void)
{
    printf("Testing I2C communication with MCP23017...\n");

    // Test simple : lire l'état initial du port B
    uint8_t test_value = 0;
    for (int pin = 0; pin < 8; pin++) {
        MCP23017_pinState_e pin_state;
        if (MCP23017_getGPIO(keyboard_chip_id, MCP23017_PORT_B, pin, &pin_state)) {
            if (pin_state == MCP23017_PIN_STATE_HIGH) {
                test_value |= (1 << pin);
            }
        }
    }
    printf("Port B initial state: 0x%02X\n", test_value);

    // Test d'écriture sur port A
    MCP23017_setGPIO(keyboard_chip_id, MCP23017_PORT_A, 0x01, MCP23017_PIN_STATE_LOW);
    HAL_Delay(10);
    MCP23017_setGPIO(keyboard_chip_id, MCP23017_PORT_A, 0x01, MCP23017_PIN_STATE_HIGH);

    printf("I2C test completed.\n");
}

#endif
