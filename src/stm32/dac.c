// DAC functions on STM32
//
// Copyright (C) 2019  Sasha Zbrozek <s.zbrozek@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "command.h" // shutdown
#include "gpio.h" // gpio_dac_setup
#include "internal.h" // gpio_peripheral
#include "sched.h" // sched_shutdown

DECL_CONSTANT("DAC_MAX", 4095);

static const uint8_t dac_pins[] = {
    GPIO('A', 4), GPIO('A', 5),
};

struct gpio_dac gpio_dac_setup(uint32_t pin)
{
    // Find pin in dac_pins table.
    int chan;
    for (chan=0; ; chan++) {
        if (chan >= ARRAY_SIZE(dac_pins))
            shutdown("Not a valid DAC pin.");
        if (dac_pins[chan] == pin)
            break;
    }

    // There's only one DAC peripheral.
    DAC_TypeDef *dac = DAC1;
    uint32_t dac_base = DAC_BASE;

    // Enable the DAC.
    enable_pclock(dac_base);
    dac->CR &= ~(0xffff << (16 * chan));

    // Single DAC mode with SW trigger
    //dac->CR |= (0xF << 2) << (16 * chan);
    dac->CR |= 1 << (16 * chan);


    // Disconnect the pin from the pad driver.
    gpio_peripheral(pin, GPIO_ANALOG, 0);

    return (struct gpio_dac){ .dac = dac, .chan = chan };
}

void gpio_dac_write(struct gpio_dac g, uint32_t data)
{
    DAC_TypeDef *dac = g.dac;
    switch (g.chan) {
    case 0:
        dac->DHR12R1 = data;
        break;
    case 1:
        dac->DHR12R2 = data;
        break;
    default:
        break;
    }
    dac->SWTRIGR = 1 << g.chan;
}

void
gpio_dual_dac_write(struct gpio_dac g, uint32_t data1, uint32_t data2)
{
    DAC_TypeDef *dac = g.dac;
    uint32_t data = g.chan ? ((data1 & 0xFFF) << 16) | (data2 & 0xFFF)
        : ((data2 & 0xFFF) << 16) | (data1 & 0xFFF);
    dac->DHR12RD = data;
}

void
gpio_dac_wait(struct gpio_dac g, uint32_t data)
{
    DAC_TypeDef *dac = g.dac;
    data &= 0xFFF;
    switch (g.chan) {
    case 0:
        while(dac->DOR1 != data);
        break;
    case 1:
        while(dac->DOR2 != data);
        break;
    default:
        break;
    }
}
