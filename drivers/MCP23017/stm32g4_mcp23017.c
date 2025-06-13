/**
 *******************************************************************************
 * @file 	stm32g4_mcp23017.c
 * @author 	vchav - Corrections par Claude
 * @date 	May 6, 2024 - Updated June 2025
 * @brief	Module pour utiliser le GPIO Expander MCP23017 - Version corrigée
 *******************************************************************************
 */

/*
 * Ce pilote permet de controller le GPIO expander avec communication I2C: Le MCP23017
 * Ce composant va permettre d'étendre le nombre de pins disponibles pour utiliser plus
 * de capteurs par exemple.
 *
 * CORRECTIONS APPORTÉES:
 * - Adresse I2C fixée à 0x20 (au lieu de 0x40)
 * - Ajout de MCP23017_getGPIO_all_pins()
 * - Amélioration de la gestion d'erreurs
 * - Nettoyage du code
 */

#include "config.h"
#if USE_MCP23017
#include "stm32g4_mcp23017.h"
#include "stm32g4_i2c.h"
#include "stm32g4_utils.h"
#include <stdio.h>

#define MCP23017_MAX_NB_ERROR	3

/*
 * Système de registre en mode IOCON.BANK = 0, ce qui signifie que les registres sont dans la même bank
 * et ont donc des adresses séquentielles (0, 1, 2, 3,...)
 */
typedef enum{
	/*Directions des broches d'entrée/sortie pour les ports A et B*/
	MPC23017_REGISTER_IODIR_A		= 0x00,
	MPC23017_REGISTER_IODIR_B		= 0x01,

	/*Polarisations des broches pour les ports A et B*/
	MPC23017_REGISTER_IPOL_A		= 0x02,
	MPC23017_REGISTER_IPOL_B		= 0x03,

	/*Activation des interruptions pour les ports A et B*/
	MPC23017_REGISTER_GPINTEN_A		= 0x04,
	MPC23017_REGISTER_GPINTEN_B		= 0x05,

	/*Valeurs par défaut pour les broches pour les ports A et B*/
	MPC23017_REGISTER_DEFVAL_A		= 0x06,
	MPC23017_REGISTER_DEFVAL_B		= 0x07,

	/*Configuration des broches d'interruption pour les ports A et B*/
	MPC23017_REGISTER_INTCON_A		= 0x08,
	MPC23017_REGISTER_INTCON_B		= 0x09,

	/*Configuration des broches pour les ports A et B*/
	MPC23017_REGISTER_IOCON_A		= 0x0A,
	MPC23017_REGISTER_IOCON_B		= 0x0B,

	/*Activation des résistances de pull-up pour les ports A et B*/
	MPC23017_REGISTER_GPPU_A		= 0x0C,
	MPC23017_REGISTER_GPPU_B		= 0x0D,

	/*Drapeaux d'interruption pour les ports A et B*/
	MPC23017_REGISTER_INTF_A		= 0x0E,
	MPC23017_REGISTER_INTF_B		= 0x0F,

	/*Valeurs des broches au moment où une interruption a été déclenchée pour les ports A et B*/
	MPC23017_REGISTER_INTCAP_A		= 0x10,
	MPC23017_REGISTER_INTCAP_B		= 0x11,

	/*Valeurs des broches pour les ports A et B*/
	MPC23017_REGISTER_GPIO_A		= 0x12,
	MPC23017_REGISTER_GPIO_B		= 0x13,

	/*Registres de sortie pour les ports A et B*/
	MPC23017_REGISTER_OLAT_A		= 0x14,
	MPC23017_REGISTER_OLAT_B		= 0x15

}MCP23017_register_e;

typedef enum{
	MCP23017_REG_IODIR_OUTPUT	= 0,
	MCP23017_REG_IODIR_INPUT	= 1
}MCP23017_reg_iodir_e;

typedef enum{
	MCP23017_REG_IPOL_SAME_POLARITY		= 0,
	MCP23017_REG_IPOL_OPPOSITE_POLARITY	= 1
}MCP23017_reg_ipol_e;

typedef enum{
	MCP23017_REG_GPITEN_DISABLE	= 0,
	MCP23017_REG_GPITEN_ENABLE	= 1
}MCP23017_reg_gpiten_e;

typedef enum{
	MCP23017_REG_INTCON_COMP_WITH_PREVIOUS	= 0,
	MCP23017_REG_INTCON_COMP_WITH_DEVFAL	= 1
}MCP23017_reg_intcon_e;

typedef union{
	struct{
		enum{
			BANK_PORT_SAME					= 0,
			BANK_PORT_SEPARATED				= 1
		}bank	:1;

		enum{
			INT_PIN_NOT_CONNECTED			= 0,
			INT_PIN_INTERNALLY_CONNECTED	= 1
		}mirror	:1;

		enum{
			SEQUENTIAL_OPERATION_ENABLED	= 0,
			SEQUENTIAL_OPERATION_DISABLED	= 1
		}seqop	:1;

		enum{
			SLEW_RATE_ENABLED				= 0,
			SLEW_RATE_DISABLED				= 1
		}disslw	:1;

		enum{
			ADDRESS_PIN_DISABLE				= 0,
			ADDRESS_PIN_ENABLE				= 1
		}haen	:1;

		enum{
			OUTPUT_ACTIVE_DRIVER			= 0,
			OUTPUT_OPEN_DRAIN				= 1,
		}odr	:1;

		enum{
			POLARITY_INT_PIN_LOW			= 0,
			POLARITY_INT_PIN_HIGH			= 1
		}intpol	:1;

		uint8_t:1;
	};

	uint8_t rawData;

}MCP23017_reg_iocon_u;

typedef enum{
	MCP23017_REG_GPPU_PULL_UP_DISABLED	= 0,
	MCP23017_REG_GPPU_PULL_UP_ENABLED	= 1
}MCP23017_reg_gppu_e;

typedef enum{
	MCP23017_REG_INTF_INTERRUPT_DISABLED	= 0,
	MCP23017_REG_INTF_INTERRUPT_ENABLED		= 1
}MCP23017_reg_intf_e;

typedef enum{
	MCP23017_REG_INTCAP_LOGIC_LOW	= 0,
	MCP23017_REG_INTCAP_LOGIC_HIGH	= 1
}MCP23017_reg_intcap_e;

typedef enum{
	MCP23017_REG_GPIO_LOGIC_LOW		= 0,
	MCP23017_REG_GPIO_LOGIC_HIGH	= 1
}MCP23017_reg_gpio_e;

typedef enum{
	MCP23017_REG_OLAT_LOGIC_LOW		= 0,
	MCP23017_REG_OLAT_LOGIC_HIGH	= 1
}MCP23017_reg_olat_e;

typedef struct{
	bool used;
	MCP23017_address_t address;
	I2C_TypeDef* I2Cx;
	uint8_t error_count;  // Compteur d'erreurs ajouté
}MCP23017_ic_t;

// Tableau contenant toutes les infos sur les modules MCP23017 connectés à la carte
static volatile MCP23017_ic_t MCP23017_ic[MCP23017_NB_IC];

// Fonctions privées
static HAL_StatusTypeDef MCP23017_initIc(MCP23017_id_t id, I2C_TypeDef* I2Cx, MCP23017_address_t address);
static bool MCP23017_checkId(MCP23017_id_t id);

/**
 * @brief Initialise le module MCP23017
 * @return true si l'initialisation a réussi
 */
bool MCP23017_init(){
	uint8_t i;
	for(i = 0; i < MCP23017_NB_IC; i++){
		MCP23017_ic[i].used = false;
		MCP23017_ic[i].error_count = 0;
	}
	return true;
}

/**
 * @brief Ajoute un MCP23017 à la liste des périphériques connectés
 * @param I2Cx: pointeur vers l'I2C qui va accueillir le GPIO expander
 * @param address: adresse à laquelle l'I2C va lire (de 0b000 à 0b111, correspondant aux broches A0, A1 et A2)
 * @return MCP23017_ID_ERROR si l'opération a échoué. Sinon, il renvoie l'id du MCP23017 qui vient d'être ajouté
 */
MCP23017_id_t MCP23017_add(I2C_TypeDef* I2Cx, MCP23017_address_t address){

	if(address > 0x07){
		printf("MCP23017_add : Erreur adresse invalide (%d). Doit être entre 0x00 et 0x07\n", address);
		return MCP23017_ID_ERROR;
	}

	uint8_t i;
	for(i = 0; i < MCP23017_NB_IC; i++){
		if(MCP23017_ic[i].used == false){
			if(MCP23017_initIc(i, I2Cx, address) == HAL_OK){
				printf("MCP23017_add : Initialisation du capteur réussie (address : 0x%02X | id : %d)\n", MCP23017_ic[i].address, i);
				return i;
			}else{
				printf("MCP23017_add : Initialisation du capteur échouée (address : 0x%02X)\n", address);
				return MCP23017_ID_ERROR;
			}
		}
	}

	printf("MCP23017_add : Erreur nombre de capteurs trop élevé (address : 0x%02X)\n", address);
	return MCP23017_ID_ERROR;
}

/**
 * @brief Vérifie si l'ID du MCP23017 est valide
 * @param id: L'identifiant du MCP23017
 * @return true si l'ID est valide, false sinon
 */
static bool MCP23017_checkId(MCP23017_id_t id){
	if(id >= MCP23017_NB_IC){
		printf("MCP23017 : Erreur identifiant du capteur inconnu (%d)\n", id);
		return false;
	}

	if(MCP23017_ic[id].used == false){
		printf("MCP23017 : Erreur capteur non initialisé (%d)\n", id);
		return false;
	}

	return true;
}

/**
 * @brief Configure la direction d'une broche du GPIO expander MCP23017
 * @param id: L'identifiant du MCP23017
 * @param port: Le port du MCP23017 (MCP23017_PORT_A ou MCP23017_PORT_B)
 * @param pin: Le numéro du port (MCP23017_PIN_0, MCP23017_PIN_1, MCP23017_PIN_2, ...)
 * @param direction: La direction des broches (MCP23017_DIRECTION_OUTPUT ou MCP23017_DIRECTION_INPUT)
 * @return true si l'opération a réussi, false en cas d'erreur
 */
bool MCP23017_setIO(MCP23017_id_t id, MCP23017_port_e port, MCP23017_pin_e pin, MCP23017_direction_e direction){

	if(!MCP23017_checkId(id)){
		return false;
	}

	// On définit le registre
	MCP23017_register_e reg = (port == MCP23017_PORT_A) ? MPC23017_REGISTER_IODIR_A : MPC23017_REGISTER_IODIR_B;

	uint8_t value;
	// On récupère l'état actuel des pins du port
	if(BSP_I2C_Read(MCP23017_ic[id].I2Cx, MCP23017_ic[id].address, reg, &value) != HAL_OK){
		printf("MCP23017_setIO : Erreur lecture du registre IODIR (0x%02X) (address chip : 0x%02X)\n", reg, MCP23017_ic[id].address);
		MCP23017_ic[id].error_count++;
		return false;
	}

	// On modifie l'état des pins souhaités
	if(direction == MCP23017_DIRECTION_OUTPUT){
		value = value & (~pin);  // 0 = OUTPUT
	}else{
		value = value | pin;     // 1 = INPUT
	}

	// On envoie l'état des pins modifié
	if(BSP_I2C_Write(MCP23017_ic[id].I2Cx, MCP23017_ic[id].address, reg, value) != HAL_OK){
		printf("MCP23017_setIO : Erreur écriture du registre IODIR (0x%02X) (address chip : 0x%02X)\n", reg, MCP23017_ic[id].address);
		MCP23017_ic[id].error_count++;
		return false;
	}

	return true;
}

/**
 * @brief Récupère la direction d'une broche du GPIO expander MCP23017
 * @param id: L'identifiant du MCP23017
 * @param port: Le port du MCP23017 (MCP23017_PORT_A ou MCP23017_PORT_B)
 * @param pin: Le numéro du port (MCP23017_PIN_0, MCP23017_PIN_1, MCP23017_PIN_2, ...)
 * @param direction: Pointeur vers la direction des broches (MCP23017_DIRECTION_OUTPUT ou MCP23017_DIRECTION_INPUT)
 * @return true si l'opération a réussi, false en cas d'erreur
 */
bool MCP23017_getIO(MCP23017_id_t id, MCP23017_port_e port, MCP23017_pin_e pin, MCP23017_direction_e * direction){

	if(!MCP23017_checkId(id)){
		return false;
	}

	if(direction == NULL){
		printf("MCP23017_getIO : Erreur pointeur direction NULL\n");
		return false;
	}

	uint8_t value;
	MCP23017_register_e reg = (port == MCP23017_PORT_A) ? MPC23017_REGISTER_IODIR_A : MPC23017_REGISTER_IODIR_B;

	if(BSP_I2C_Read(MCP23017_ic[id].I2Cx, MCP23017_ic[id].address, reg, &value) != HAL_OK){
		printf("MCP23017_getIO : Erreur lecture du registre IODIR (0x%02X) (address chip : 0x%02X)\n", reg, MCP23017_ic[id].address);
		MCP23017_ic[id].error_count++;
		return false;
	}

	*direction = (value & pin) ? MCP23017_DIRECTION_INPUT : MCP23017_DIRECTION_OUTPUT;

	return true;
}

/**
 * @brief Modifie l'état d'une broche du GPIO expander MCP23017
 * @param id: L'identifiant du MCP23017
 * @param port: Le port du MCP23017 (MCP23017_PORT_A ou MCP23017_PORT_B)
 * @param pin: Le numéro du port (MCP23017_PIN_0, MCP23017_PIN_1, MCP23017_PIN_2, ...)
 * @param state: L'état à assigner à la broche (MCP23017_PIN_STATE_LOW ou MCP23017_PIN_STATE_HIGH)
 * @return true si l'opération a réussi, false en cas d'erreur
 */
bool MCP23017_setGPIO(MCP23017_id_t id, MCP23017_port_e port, MCP23017_pin_e pin, MCP23017_pinState_e state){

	if(!MCP23017_checkId(id)){
		return false;
	}

	MCP23017_register_e reg = (port == MCP23017_PORT_A) ? MPC23017_REGISTER_OLAT_A : MPC23017_REGISTER_OLAT_B;

	uint8_t value;
	if(BSP_I2C_Read(MCP23017_ic[id].I2Cx, MCP23017_ic[id].address, reg, &value) != HAL_OK){
		printf("MCP23017_setGPIO : Erreur lecture du registre OLAT (0x%02X) (address chip : 0x%02X)\n", reg, MCP23017_ic[id].address);
		MCP23017_ic[id].error_count++;
		return false;
	}

	if(state == MCP23017_PIN_STATE_LOW){
		value = value & (~pin);  // Clear bits
	}else{
		value = value | pin;     // Set bits
	}

	if(BSP_I2C_Write(MCP23017_ic[id].I2Cx, MCP23017_ic[id].address, reg, value) != HAL_OK){
		printf("MCP23017_setGPIO : Erreur écriture du registre OLAT (0x%02X) (address chip : 0x%02X)\n", reg, MCP23017_ic[id].address);
		MCP23017_ic[id].error_count++;
		return false;
	}

	return true;
}

/**
 * @brief Récupère l'état d'une broche du GPIO expander MCP23017
 * @param id: L'identifiant du MCP23017
 * @param port: Le port du MCP23017 (MCP23017_PORT_A ou MCP23017_PORT_B)
 * @param pin: Le numéro du port (MCP23017_PIN_0, MCP23017_PIN_1, MCP23017_PIN_2, ...)
 * @param state: Pointeur vers l'endroit où l'état de la broche sera stocké (MCP23017_PIN_STATE_LOW ou MCP23017_PIN_STATE_HIGH)
 * @return true si l'opération a réussi, false en cas d'erreur
 */
bool MCP23017_getGPIO(MCP23017_id_t id, MCP23017_port_e port, MCP23017_pin_e pin, MCP23017_pinState_e * state){

	if(!MCP23017_checkId(id)){
		return false;
	}

	if(state == NULL){
		printf("MCP23017_getGPIO : Erreur pointeur state NULL\n");
		return false;
	}

	MCP23017_register_e reg = (port == MCP23017_PORT_A) ? MPC23017_REGISTER_GPIO_A : MPC23017_REGISTER_GPIO_B;

	uint8_t value = 0;
	if(BSP_I2C_Read(MCP23017_ic[id].I2Cx, MCP23017_ic[id].address, reg, &value) != HAL_OK){
		printf("MCP23017_getGPIO : Erreur lecture du registre GPIO (0x%02X) (address chip : 0x%02X)\n", reg, MCP23017_ic[id].address);
		MCP23017_ic[id].error_count++;
		return false;
	}

	*state = (value & pin) ? MCP23017_PIN_STATE_HIGH : MCP23017_PIN_STATE_LOW;

	return true;
}

/**
 * @brief Récupère l'état de toutes les broches d'un port du GPIO expander MCP23017
 * @param id: L'identifiant du MCP23017
 * @param port: Le port du MCP23017 (MCP23017_PORT_A ou MCP23017_PORT_B)
 * @param value: Pointeur vers l'endroit où la valeur du port sera stockée (8 bits)
 * @return true si l'opération a réussi, false en cas d'erreur
 */
bool MCP23017_getGPIO_all_pins(MCP23017_id_t id, MCP23017_port_e port, uint8_t * value){

	if(!MCP23017_checkId(id)){
		return false;
	}

	if(value == NULL){
		printf("MCP23017_getGPIO_all_pins : Erreur pointeur value NULL\n");
		return false;
	}

	MCP23017_register_e reg = (port == MCP23017_PORT_A) ? MPC23017_REGISTER_GPIO_A : MPC23017_REGISTER_GPIO_B;

	if(BSP_I2C_Read(MCP23017_ic[id].I2Cx, MCP23017_ic[id].address, reg, value) != HAL_OK){
		printf("MCP23017_getGPIO_all_pins : Erreur lecture du registre GPIO (0x%02X) (address chip : 0x%02X)\n", reg, MCP23017_ic[id].address);
		MCP23017_ic[id].error_count++;
		return false;
	}

	return true;
}

/**
 * @brief Active ou désactive la résistance de pull-up pour une broche spécifique du MCP23017
 * @param id: L'identifiant du MCP23017
 * @param port: Le port du MCP23017 (MCP23017_PORT_A ou MCP23017_PORT_B)
 * @param pin: Le numéro du port (MCP23017_PIN_0, MCP23017_PIN_1, MCP23017_PIN_2, ...)
 * @param state: L'état de la résistance de pull-up que l'on souhaite appliquer (MCP23017_PULL_UP_STATE_LOW ou MCP23017_PULL_UP_STATE_HIGH)
 * @return true si l'opération a réussi, false en cas d'erreur
 */
bool MCP23017_setPullUp(MCP23017_id_t id, MCP23017_port_e port, MCP23017_pin_e pin, MCP23017_pullUpState_e state){

	if(!MCP23017_checkId(id)){
		return false;
	}

	MCP23017_register_e reg = (port == MCP23017_PORT_A) ? MPC23017_REGISTER_GPPU_A : MPC23017_REGISTER_GPPU_B;

	uint8_t value;
	if(BSP_I2C_Read(MCP23017_ic[id].I2Cx, MCP23017_ic[id].address, reg, &value) != HAL_OK){
		printf("MCP23017_setPullUp : Erreur lecture du registre GPPU (0x%02X) (address chip : 0x%02X)\n", reg, MCP23017_ic[id].address);
		MCP23017_ic[id].error_count++;
		return false;
	}

	if(state == MCP23017_PULL_UP_STATE_LOW){
		value = value & (~pin);  // Disable pull-up
	}else{
		value = value | pin;     // Enable pull-up
	}

	if(BSP_I2C_Write(MCP23017_ic[id].I2Cx, MCP23017_ic[id].address, reg, value) != HAL_OK){
		printf("MCP23017_setPullUp : Erreur écriture du registre GPPU (0x%02X) (address chip : 0x%02X)\n", reg, MCP23017_ic[id].address);
		MCP23017_ic[id].error_count++;
		return false;
	}

	return true;
}

/**
 * @brief Récupère l'état de la résistance de pull-up pour une broche spécifique du MCP23017
 * @param id: L'identifiant du MCP23017
 * @param port: Le port du MCP23017 (MCP23017_PORT_A ou MCP23017_PORT_B)
 * @param pin: Le numéro du port (MCP23017_PIN_0, MCP23017_PIN_1, MCP23017_PIN_2, ...)
 * @param state: Pointeur vers l'endroit où l'état de la résistance de pull-up sera stocké (MCP23017_PULL_UP_STATE_LOW ou MCP23017_PULL_UP_STATE_HIGH)
 * @return true si l'opération a réussi, false en cas d'erreur
 */
bool MCP23017_getPullUp(MCP23017_id_t id, MCP23017_port_e port, MCP23017_pin_e pin, MCP23017_pullUpState_e * state){

	if(!MCP23017_checkId(id)){
		return false;
	}

	if(state == NULL){
		printf("MCP23017_getPullUp : Erreur pointeur state NULL\n");
		return false;
	}

	MCP23017_register_e reg = (port == MCP23017_PORT_A) ? MPC23017_REGISTER_GPPU_A : MPC23017_REGISTER_GPPU_B;

	uint8_t value = 0;
	if(BSP_I2C_Read(MCP23017_ic[id].I2Cx, MCP23017_ic[id].address, reg, &value) != HAL_OK){
		printf("MCP23017_getPullUp : Erreur lecture du registre GPPU (0x%02X) (address chip : 0x%02X)\n", reg, MCP23017_ic[id].address);
		MCP23017_ic[id].error_count++;
		return false;
	}

	*state = (value & pin) ? MCP23017_PULL_UP_STATE_HIGH : MCP23017_PULL_UP_STATE_LOW;

	return true;
}

/**
 * @brief Récupère le nombre d'erreurs I2C pour un MCP23017
 * @param id: L'identifiant du MCP23017
 * @return Le nombre d'erreurs, ou 0xFF si l'ID est invalide
 */
uint8_t MCP23017_getErrorCount(MCP23017_id_t id){
	if(id >= MCP23017_NB_IC || !MCP23017_ic[id].used){
		return 0xFF;
	}
	return MCP23017_ic[id].error_count;
}

/**
 * @brief Remet à zéro le compteur d'erreurs pour un MCP23017
 * @param id: L'identifiant du MCP23017
 * @return true si l'opération a réussi, false sinon
 */
bool MCP23017_resetErrorCount(MCP23017_id_t id){
	if(!MCP23017_checkId(id)){
		return false;
	}
	MCP23017_ic[id].error_count = 0;
	return true;
}

/**
 * @brief Initialise l'I2C donné et configure la structure du MCP23017
 * @param id: L'identifiant du MCP23017
 * @param I2Cx: L'I2C sur lequel est connecté le MCP23017
 * @param address: L'adresse du MCP23017 (de 0b000 à 0b111).
 * @return HAL_StatusTypeDef HAL_OK si l'initialisation est réussie, HAL_ERROR sinon.
 */
static HAL_StatusTypeDef MCP23017_initIc(MCP23017_id_t id, I2C_TypeDef* I2Cx, MCP23017_address_t address){
	if(id >= MCP23017_NB_IC){
		printf("MCP23017_initIc : Erreur id (%d) non conforme\n", id);
		return HAL_ERROR;
	}

	// CORRECTION MAJEURE: Calcul correct de l'adresse I2C
	// Pour MCP23017: adresse de base = 0x20, puis on ajoute les bits A2,A1,A0
	MCP23017_ic[id].address = (0x20 << 1) | ((address & 0x07) << 1);
	MCP23017_ic[id].I2Cx = I2Cx;
	MCP23017_ic[id].used = true;
	MCP23017_ic[id].error_count = 0;

	if(BSP_I2C_Init(I2Cx, STANDARD_MODE, true) != HAL_OK) {
		MCP23017_ic[id].used = false;
		printf("MCP23017_initIc : Erreur initialisation I2C\n");
		return HAL_ERROR;
			}

			// Test de communication avec le MCP23017
			uint8_t test_value;
			if(BSP_I2C_Read(I2Cx, MCP23017_ic[id].address, MPC23017_REGISTER_IODIR_A, &test_value) != HAL_OK) {
				printf("MCP23017_initIc : Erreur communication avec MCP23017 (address : 0x%02X)\n", MCP23017_ic[id].address);
				MCP23017_ic[id].used = false;
				return HAL_ERROR;
			}

			// Configuration par défaut optimisée pour clavier matriciel
			// PORTA = colonnes (sorties) - toutes les pins en sortie
			if(BSP_I2C_Write(I2Cx, MCP23017_ic[id].address, MPC23017_REGISTER_IODIR_A, 0x00) != HAL_OK) {
				printf("MCP23017_initIc : Erreur configuration IODIR_A\n");
				MCP23017_ic[id].used = false;
				return HAL_ERROR;
			}

			// PORTB = lignes (entrées) - toutes les pins en entrée
			if(BSP_I2C_Write(I2Cx, MCP23017_ic[id].address, MPC23017_REGISTER_IODIR_B, 0xFF) != HAL_OK) {
				printf("MCP23017_initIc : Erreur configuration IODIR_B\n");
				MCP23017_ic[id].used = false;
				return HAL_ERROR;
			}

			// Activer les pull-up sur PORTB (lignes) pour éviter les états flottants
			if(BSP_I2C_Write(I2Cx, MCP23017_ic[id].address, MPC23017_REGISTER_GPPU_B, 0xFF) != HAL_OK) {
				printf("MCP23017_initIc : Erreur configuration GPPU_B\n");
				MCP23017_ic[id].used = false;
				return HAL_ERROR;
			}

			// Désactiver les pull-up sur PORTA (colonnes) - pas nécessaire pour les sorties
			if(BSP_I2C_Write(I2Cx, MCP23017_ic[id].address, MPC23017_REGISTER_GPPU_A, 0x00) != HAL_OK) {
				printf("MCP23017_initIc : Erreur configuration GPPU_A\n");
				MCP23017_ic[id].used = false;
				return HAL_ERROR;
			}

			// Initialiser toutes les colonnes (PORTA) à HIGH par défaut
			if(BSP_I2C_Write(I2Cx, MCP23017_ic[id].address, MPC23017_REGISTER_OLAT_A, 0xFF) != HAL_OK) {
				printf("MCP23017_initIc : Erreur configuration OLAT_A\n");
				MCP23017_ic[id].used = false;
				return HAL_ERROR;
			}

			// Configuration IOCON - Mode par défaut (BANK=0, pas d'interruptions)
			MCP23017_reg_iocon_u iocon;
			iocon.rawData = 0x00;  // Configuration par défaut
			iocon.bank = BANK_PORT_SAME;  // Registres séquentiels
			iocon.mirror = INT_PIN_NOT_CONNECTED;  // Pas d'interruptions
			iocon.seqop = SEQUENTIAL_OPERATION_ENABLED;  // Mode séquentiel activé
			iocon.disslw = SLEW_RATE_ENABLED;  // Slew rate activé
			iocon.haen = ADDRESS_PIN_DISABLE;  // Pas besoin pour I2C
			iocon.odr = OUTPUT_ACTIVE_DRIVER;  // Sortie push-pull
			iocon.intpol = POLARITY_INT_PIN_LOW;  // Polarité interruption (non utilisé)

			if(BSP_I2C_Write(I2Cx, MCP23017_ic[id].address, MPC23017_REGISTER_IOCON_A, iocon.rawData) != HAL_OK) {
				printf("MCP23017_initIc : Erreur configuration IOCON_A\n");
				MCP23017_ic[id].used = false;
				return HAL_ERROR;
			}

			// Configuration identique pour IOCON_B
			if(BSP_I2C_Write(I2Cx, MCP23017_ic[id].address, MPC23017_REGISTER_IOCON_B, iocon.rawData) != HAL_OK) {
				printf("MCP23017_initIc : Erreur configuration IOCON_B\n");
				MCP23017_ic[id].used = false;
				return HAL_ERROR;
			}

			// Désactiver toutes les interruptions (par sécurité)
			if(BSP_I2C_Write(I2Cx, MCP23017_ic[id].address, MPC23017_REGISTER_GPINTEN_A, 0x00) != HAL_OK) {
				printf("MCP23017_initIc : Erreur désactivation interruptions PORTA\n");
				MCP23017_ic[id].used = false;
				return HAL_ERROR;
			}

			if(BSP_I2C_Write(I2Cx, MCP23017_ic[id].address, MPC23017_REGISTER_GPINTEN_B, 0x00) != HAL_OK) {
				printf("MCP23017_initIc : Erreur désactivation interruptions PORTB\n");
				MCP23017_ic[id].used = false;
				return HAL_ERROR;
			}

			// Vérification finale - lecture des registres configurés
			uint8_t verify_iodir_a, verify_iodir_b, verify_gppu_b;

			if(BSP_I2C_Read(I2Cx, MCP23017_ic[id].address, MPC23017_REGISTER_IODIR_A, &verify_iodir_a) != HAL_OK ||
			   BSP_I2C_Read(I2Cx, MCP23017_ic[id].address, MPC23017_REGISTER_IODIR_B, &verify_iodir_b) != HAL_OK ||
			   BSP_I2C_Read(I2Cx, MCP23017_ic[id].address, MPC23017_REGISTER_GPPU_B, &verify_gppu_b) != HAL_OK) {
				printf("MCP23017_initIc : Erreur vérification configuration\n");
				MCP23017_ic[id].used = false;
				return HAL_ERROR;
			}

			// Vérifier que la configuration est correcte
			if(verify_iodir_a != 0x00 || verify_iodir_b != 0xFF || verify_gppu_b != 0xFF) {
				printf("MCP23017_initIc : Configuration incorrecte (IODIR_A=0x%02X, IODIR_B=0x%02X, GPPU_B=0x%02X)\n",
					   verify_iodir_a, verify_iodir_b, verify_gppu_b);
				MCP23017_ic[id].used = false;
				return HAL_ERROR;
			}

			printf("MCP23017_initIc : Configuration réussie (address : 0x%02X)\n", MCP23017_ic[id].address);
			return HAL_OK;
		}

		#endif /* USE_MCP23017 */
