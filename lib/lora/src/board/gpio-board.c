/*
 * This file is derived from the MicroPython project, http://micropython.org/
 *
 * Copyright (c) 2016, Pycom Limited and its licensors.
 *
 * This software is licensed under the GNU GPL version 3 or any later version,
 * with permitted additional terms. For more information see the Pycom Licence
 * v1.0 document supplied with this file, or available at:
 * https://www.pycom.io/opensource/licensing
 *
 * This file contains code under the following copyright and licensing notices.
 * The code has been changed but otherwise retained.
 */

/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board GPIO driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/

#include "gpio-board.h"
#include "board.h"

/*!
 * \brief Initializes the given GPIO object
 *
 * \param [IN] obj    Pointer to the GPIO object to be initialized
 * \param [IN] pin    Pin name ( please look in pinName-board.h file )
 * \param [IN] mode   Pin mode [PIN_INPUT, PIN_OUTPUT]
 * \param [IN] config Pin config [PIN_PUSH_PULL, PIN_OPEN_DRAIN]
 * \param [IN] type   Pin type [PIN_NO_PULL, PIN_PULL_UP, PIN_PULL_DOWN]
 * \param [IN] value  Default output value at initialisation
 */
void GpioMcuInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value ) {    
    gpio_config_t gpioConfig;
    obj->pin = pin;
    gpioConfig.pin_bit_mask = 1<<pin;
    
    switch(mode){
      case PIN_INPUT:
        gpioConfig.mode = (config == PIN_OPEN_DRAIN) ? GPIO_MODE_INPUT_OUTPUT_OD : GPIO_MODE_INPUT;
        break;
      case PIN_OUTPUT:
        gpioConfig.mode = (config == PIN_OPEN_DRAIN) ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT;
        break;
      default:
        gpioConfig.mode = GPIO_MODE_INPUT_OUTPUT;
        break;
    }
    
    if(type == PIN_PULL_UP){
      gpioConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    } else {
      gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    }
    
    if(type == PIN_PULL_DOWN){
      gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
    } else {
      gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    }
    
    gpioConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpioConfig);
    
    // write default value
    GpioMcuWrite(obj, value);
}

/*!
 * \brief GPIO IRQ Initialization
 *
 * \param [IN] obj         Pointer to the GPIO object to be initialized
 * \param [IN] irqMode     IRQ mode [NO_IRQ, IRQ_RISING_EDGE,
 *                                  IRQ_FALLING_EDGE, IRQ_RISING_FALLING_EDGE]
 * \param [IN] irqPriority IRQ priority [IRQ_VERY_LOW_PRIORITY, IRQ_LOW_PRIORITY
 *                                       IRQ_MEDIUM_PRIORITY, IRQ_HIGH_PRIORITY
 *                                       IRQ_VERY_HIGH_PRIORITY]
 * \param [IN] irqHandler  Callback function pointer
 */
void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler ) {
    // unused
    (void)irqPriority;
  
    gpio_intr_disable(obj->pin);    
    gpio_set_intr_type(obj->pin, irqMode);
    gpio_isr_handler_add(obj->pin, (gpio_isr_t)irqHandler, NULL);
    gpio_intr_enable(obj->pin);
}

/*!
 * \brief GPIO IRQ DeInitialization
 *
 * \param [IN] obj         Pointer to the GPIO object to be Deinitialized
 */
void GpioMcuRemoveInterrupt( Gpio_t *obj ) {
  gpio_intr_disable(obj->pin);
  gpio_isr_handler_remove(obj->pin);
}

/*!
 * \brief Writes the given value to the GPIO output
 *
 * \param [IN] obj    Pointer to the GPIO object
 * \param [IN] value  New GPIO output value
 */
IRAM_ATTR void GpioMcuWrite( Gpio_t *obj, uint32_t value ) {
    gpio_set_level(obj->pin, value);
}

/*!
 * \brief Reads the current GPIO input value
 *
 * \param [IN] obj    Pointer to the GPIO object
 * \retval value  Current GPIO input value
 */
IRAM_ATTR uint32_t GpioMcuRead( Gpio_t *obj ) {
    return gpio_get_level(obj->pin);
}
