/**
 *******************************************************************************
 * @file    main.c
 * @author  Generated - DEEP Project
 * @date    June 13, 2025
 * @brief   STM32G431KB Matrix Keyboard MIDI Controller - Version corrigée
 *          Détection multi-touches avec support MIDI polyphonique réel
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include <stdio.h>
#include <string.h>
#include "stm32g4_uart.h"
#include "MatrixKeyboard/stm32g4_matrix_keyboard.h"
#include "midi.h"

/* Private defines -----------------------------------------------------------*/
#define LED_BLINK_PERIOD_MS    1000   // LED heartbeat
#define KEYBOARD_SCAN_PERIOD_MS 10    // Scan rapide mais pas trop (10ms pour éviter spam)
#define MAX_MATRIX_KEYS        64     // 8x8 matrix
#define DEBOUNCE_COUNT         3      // 3 lectures consécutives pour valider un changement

/* Piano keyboard mapping pour matrice 8x8 */
static const char piano_layout[64] = {
    // Rangée 0 (1,1 à 1,8): Do à Sol
    'C', 'c#', 'D', 'd#', 'E', 'F', 'f#', 'G',
    // Rangée 1 (2,1 à 2,8): Sol# à Ré
    'g#', 'A', 'a#', 'B', 'C', 'c#', 'D', 'd#',
    // Rangée 2 (3,1 à 3,8): Mi à Do
    'E', 'F', 'f#', 'G', 'g#', 'A', 'a#', 'B',
    // Rangée 3 (4,1 à 4,8): Do# à Sol#
    'C', 'c#', 'D', 'd#', 'E', 'F', 'f#', 'G',
    // Rangée 4 (5,1 à 5,8): La à Fa
    'g#', 'A', 'a#', 'B', 'C', 'c#', 'D', 'd#',
    // Rangée 5 (6,1 à 6,8): Fa# à Ré
    'E', 'F', 'f#', 'G', 'g#', 'A', 'a#', 'B',
    // Rangée 6 (7,1 à 7,8): Ré# à Si
    'C', 'c#', 'D', 'd#', 'E', 'F', 'f#', 'G',
    // Rangée 7 (8,1 à 8,8): Notes hautes
    'g#', 'A', 'a#', 'B', 'C', 'c#', 'D', 'd#'
};

/* Mapping MIDI - Note de base Do3 = 48 */
static const uint8_t midi_notes[64] = {
    // Rangée 0: Do3 à Sol3
    48, 49, 50, 51, 52, 53, 54, 55,
    // Rangée 1: Sol#3 à Ré4
    56, 57, 58, 59, 60, 61, 62, 63,
    // Rangée 2: Mi4 à Do5
    64, 65, 66, 67, 68, 69, 70, 71,
    // Rangée 3: Do#5 à Sol#5
    72, 73, 74, 75, 76, 77, 78, 79,
    // Rangée 4: La5 à Fa6
    80, 81, 82, 83, 84, 85, 86, 87,
    // Rangée 5: Fa#6 à Ré7
    88, 89, 90, 91, 92, 93, 94, 95,
    // Rangée 6: Ré#7 à Si7
    96, 97, 98, 99, 100, 101, 102, 103,
    // Rangée 7: Notes très hautes
    104, 105, 106, 107, 108, 109, 110, 111
};

/* Structure de gestion d'état du clavier */
typedef struct {
    uint32_t current_state;          // État actuel des touches (32 bits suffisent pour 64 touches dans notre cas)
    uint32_t previous_state;         // État précédent pour détecter les changements
    uint8_t debounce_counter[64];    // Compteurs de débounce individuels par touche
    uint32_t stable_state;           // État stable après débounce
} KeyboardState_t;

/* Function prototypes -------------------------------------------------------*/
extern void Error_Handler(void);
extern void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void LED_Process(void);
static void Keyboard_MIDI_Process(void);
static void Process_Key_Changes(KeyboardState_t* kbd_state);
static void Send_MIDI_Note(int key_index, bool pressed);
static void Update_Keyboard_State(KeyboardState_t* kbd_state, uint32_t raw_state);

/* Déclaration de la fonction externe du BSP */
extern uint32_t MATRIX_KEYBOARD_read_all_touchs(void);

/* Private variables ---------------------------------------------------------*/
static uint32_t led_last_toggle = 0;
static uint32_t keyboard_last_scan = 0;
static KeyboardState_t keyboard_state = {0};

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* MCU Configuration ------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    /* Initialize UART pour debug et MIDI */
    BSP_UART_init(UART2_ID, 115200);
    BSP_SYS_set_std_usart(UART2_ID, UART2_ID, UART2_ID);

    /* Application startup */
    printf("\r\n");
    printf("===========================================\r\n");
    printf("  STM32G431KB - MIDI Matrix Keyboard v2\r\n");
    printf("  DEEP Project - Multi-Key Support\r\n");
    printf("===========================================\r\n");
    printf("System Clock: %lu MHz\r\n", HAL_RCC_GetHCLKFreq() / 1000000);

    /* Initialize MIDI */
    MIDI_init();
    printf("MIDI initialized - Channel 1, Polyphonic mode\r\n");

    /* Initialize matrix keyboard */
    printf("Initializing 8x8 matrix keyboard...\r\n");
    BSP_MATRIX_KEYBOARD_init(piano_layout);

    /* Test I2C connectivity */
    BSP_MATRIX_KEYBOARD_test_i2c();

    /* Clear keyboard state */
    memset(&keyboard_state, 0, sizeof(KeyboardState_t));

    printf("Matrix keyboard MIDI controller ready!\r\n");
    printf("Mapping: Position (row,col) -> Note\r\n");
    printf("  (1,1)=Do3, (1,2)=Do#3, (1,3)=Ré3, etc.\r\n");
    printf("Multi-key detection enabled (polyphonic MIDI)\r\n");
    printf("===========================================\r\n");

    /* Initialize timing */
    led_last_toggle = HAL_GetTick();
    keyboard_last_scan = HAL_GetTick();

    /* Main loop */
    while (1)
    {
        LED_Process();
        Keyboard_MIDI_Process();

        /* Small delay pour éviter de surcharger le CPU */
        HAL_Delay(2);
    }
}

/**
 * @brief GPIO Initialization Function
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_PIN_RESET);

    /* Configure GPIO pin : LED_GREEN_PIN */
    GPIO_InitStruct.Pin = LED_GREEN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GREEN_GPIO, &GPIO_InitStruct);
}

/**
 * @brief LED heartbeat process
 */
static void LED_Process(void)
{
    uint32_t current_tick = HAL_GetTick();

    if (current_tick - led_last_toggle >= LED_BLINK_PERIOD_MS)
    {
        HAL_GPIO_TogglePin(LED_GREEN_GPIO, LED_GREEN_PIN);
        led_last_toggle = current_tick;
    }
}

/**
 * @brief Update keyboard state avec débounce
 * @param kbd_state: Pointeur vers la structure d'état du clavier
 * @param raw_state: État brut lu de la matrice
 */
static void Update_Keyboard_State(KeyboardState_t* kbd_state, uint32_t raw_state)
{
    /* Sauvegarder l'état précédent */
    kbd_state->previous_state = kbd_state->stable_state;

    /* Traiter chaque touche avec débounce individuel */
    for (int i = 0; i < MAX_MATRIX_KEYS; i++) {
        bool raw_pressed = (raw_state & (1UL << i)) != 0;
        bool stable_pressed = (kbd_state->stable_state & (1UL << i)) != 0;

        if (raw_pressed == stable_pressed) {
            /* État inchangé, reset du compteur */
            kbd_state->debounce_counter[i] = 0;
        } else {
            /* État changé, incrémentation du compteur */
            kbd_state->debounce_counter[i]++;

            /* Si le seuil de débounce est atteint, valider le changement */
            if (kbd_state->debounce_counter[i] >= DEBOUNCE_COUNT) {
                if (raw_pressed) {
                    kbd_state->stable_state |= (1UL << i);
                } else {
                    kbd_state->stable_state &= ~(1UL << i);
                }
                kbd_state->debounce_counter[i] = 0;
            }
        }
    }

    /* Mettre à jour current_state pour la compatibilité */
    kbd_state->current_state = kbd_state->stable_state;
}

/**
 * @brief Processus principal de scan du clavier et MIDI
 */
static void Keyboard_MIDI_Process(void)
{
    uint32_t current_tick = HAL_GetTick();

    if (current_tick - keyboard_last_scan >= KEYBOARD_SCAN_PERIOD_MS)
    {
        /* Lire l'état brut de la matrice via le BSP */
        uint32_t raw_state = MATRIX_KEYBOARD_read_all_touchs();

        /* Mettre à jour l'état du clavier avec débounce */
        Update_Keyboard_State(&keyboard_state, raw_state);

        /* Traiter les changements d'état */
        Process_Key_Changes(&keyboard_state);

        keyboard_last_scan = current_tick;
    }
}

/**
 * @brief Traite les changements d'état du clavier et envoie les messages MIDI
 * @param kbd_state: Pointeur vers la structure d'état du clavier
 */
static void Process_Key_Changes(KeyboardState_t* kbd_state)
{
    /* Détecter les changements par XOR des états actuel et précédent */
    uint32_t changes = kbd_state->current_state ^ kbd_state->previous_state;

    if (changes == 0) {
        return; /* Aucun changement */
    }

    /* Traiter chaque touche qui a changé */
    for (int i = 0; i < MAX_MATRIX_KEYS; i++) {
        if (changes & (1UL << i)) {
            /* Cette touche a changé d'état */
            bool is_pressed = (kbd_state->current_state & (1UL << i)) != 0;
            Send_MIDI_Note(i, is_pressed);
        }
    }
}

/**
 * @brief Envoie un message MIDI note on/off pour une touche spécifique
 * @param key_index: Index de la touche (0-63)
 * @param pressed: true si pressée, false si relâchée
 */
static void Send_MIDI_Note(int key_index, bool pressed)
{
    if (key_index < 0 || key_index >= MAX_MATRIX_KEYS) return;

    uint8_t midi_note = midi_notes[key_index];
    char key_char = piano_layout[key_index];

    /* Calculer position (row, col) pour affichage */
    int row = key_index / 8 + 1;  // +1 pour affichage 1-indexé
    int col = key_index % 8 + 1;  // +1 pour affichage 1-indexé

    if (pressed) {
        /* Envoyer MIDI Note On */
        MIDI_send_note_on(1, midi_note, 100);  // Canal 1, vélocité 100
        printf("[MIDI] Note ON  - Position (%d,%d) '%c' -> MIDI:%d\r\n",
               row, col, key_char, midi_note);
    } else {
        /* Envoyer MIDI Note Off */
        MIDI_send_note_off(1, midi_note, 0);   // Canal 1, vélocité 0
        printf("[MIDI] Note OFF - Position (%d,%d) '%c' -> MIDI:%d\r\n",
               row, col, key_char, midi_note);
    }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert failed: file %s on line %lu\r\n", file, line);
}
#endif /* USE_FULL_ASSERT */
