/*
 * Copyright (C) 2016 Ingenic Semiconductor
 *
 * SunWenZhong(Fighter) <wenzhong.sun@ingenic.com, wanmyqawdr@126.com>
 *
 * For project-5
 *
 * Release under GPLv2
 *
 */


#include "./floader_m200.h"

enum gpio_port {
    GPIO_PORT_A,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_F,
    GPIO_PORT_G,

    GPIO_NR_PORTS,
};

#define PXPIN       0x00
#define PXINT       0x10
#define PXINTS      0x14
#define PXINTC      0x18
#define PXMSK       0x20
#define PXMSKS      0x24
#define PXMSKC      0x28
#define PXPAT1      0x30
#define PXPAT1S     0x34
#define PXPAT1C     0x38
#define PXPAT0      0x40
#define PXPAT0S     0x44
#define PXPAT0C     0x48
#define PXFLG       0x50
#define PXFLGC      0x58
#define PXPE        0x70
#define PXPES       0x74
#define PXPEC       0x78

#define GPIO_PXPIN(n)   (GPIO_BASE + (PXPIN + (n)*0x100))
#define GPIO_PXINT(n)   (GPIO_BASE + (PXINT + (n)*0x100))
#define GPIO_PXINTS(n)  (GPIO_BASE + (PXINTS + (n)*0x100))
#define GPIO_PXINTC(n)  (GPIO_BASE + (PXINTC + (n)*0x100))
#define GPIO_PXMSK(n)   (GPIO_BASE + (PXMSK + (n)*0x100))
#define GPIO_PXMSKS(n)  (GPIO_BASE + (PXMSKS + (n)*0x100))
#define GPIO_PXMSKC(n)  (GPIO_BASE + (PXMSKC + (n)*0x100))
#define GPIO_PXPAT1(n)  (GPIO_BASE + (PXPAT1 + (n)*0x100))
#define GPIO_PXPAT1S(n) (GPIO_BASE + (PXPAT1S + (n)*0x100))
#define GPIO_PXPAT1C(n) (GPIO_BASE + (PXPAT1C + (n)*0x100))
#define GPIO_PXPAT0(n)  (GPIO_BASE + (PXPAT0 + (n)*0x100))
#define GPIO_PXPAT0S(n) (GPIO_BASE + (PXPAT0S + (n)*0x100))
#define GPIO_PXPAT0C(n) (GPIO_BASE + (PXPAT0C + (n)*0x100))
#define GPIO_PXFLG(n)   (GPIO_BASE + (PXFLG + (n)*0x100))
#define GPIO_PXFLGC(n)  (GPIO_BASE + (PXFLGC + (n)*0x100))
#define GPIO_PXPE(n)    (GPIO_BASE + (PXPE + (n)*0x100))
#define GPIO_PXPES(n)   (GPIO_BASE + (PXPES + (n)*0x100))
#define GPIO_PXPEC(n)   (GPIO_BASE + (PXPEC + (n)*0x100))

static void check_gpio(int gpio)
{
    if (gpio < 0 || gpio > 192) {
        debug("Error, wrong gpio number: %d\n", gpio);
        stop(STOP_ERROR_WRONG_GPIO_NUM);
    }
}

int gpio_request(int gpio, const char *label)
{
    debug("gpio request: %d, label: %s\n", gpio, label);

    check_gpio(gpio);

    return 0;
}

int gpio_free(int gpio)
{
    debug("gpio free: %d\n", gpio);

    check_gpio(gpio);

    return 0;
}

int gpio_set_value(int gpio, int value)
{
    check_gpio(gpio);

    int port = gpio / 32;
    int pin = gpio & 0x1f;

    if (value)
        writel(1 << pin, GPIO_PXPAT0S(port));
    else
        writel(1 << pin, GPIO_PXPAT0C(port));

    return 0;
}

int gpio_get_value(int gpio)
{
    check_gpio(gpio);

    int port = gpio / 32;
    int pin = gpio & 0x1f;

    return !!(readl(GPIO_PXPIN(port)) & (1 << pin));
}

int gpio_enable_pullup(int gpio)
{
    check_gpio(gpio);

    int port = gpio / 32;
    int pin = gpio & 0x1f;

    writel(1 << pin, GPIO_PXPEC(port));

    return 0;
}

int gpio_disable_pullup(int gpio)
{
    check_gpio(gpio);

    int port = gpio / 32;
    int pin = gpio & 0x1f;

    writel(1 << pin, GPIO_PXPES(port));

    return 0;
}

int gpio_direction_input(int gpio)
{
    check_gpio(gpio);

    int port = gpio / 32;
    int pin = gpio & 0x1f;

    writel(1 << pin, GPIO_PXINTC(port));
    writel(1 << pin, GPIO_PXMSKS(port));
    writel(1 << pin, GPIO_PXPAT1S(port));

    return 0;
}

int gpio_direction_output(int gpio, int value)
{
    check_gpio(gpio);

    int port = gpio / 32;
    int pin = gpio & 0x1f;

    writel(1 << pin, GPIO_PXINTC(port));
    writel(1 << pin, GPIO_PXMSKS(port));
    writel(1 << pin, GPIO_PXPAT1C(port));

    gpio_set_value(gpio, value);

    return 0;
}


int gpio_set_func(int gpio, enum gpio_function func)
{
    check_gpio(gpio);

    int port = gpio / 32;
    int pin = gpio & 0x1f;

    switch (func) {
    case GPIO_FUNC_0:
        writel(1 << pin, GPIO_PXINTC(port));
        writel(1 << pin, GPIO_PXMSKC(port));
        writel(1 << pin, GPIO_PXPAT1C(port));
        writel(1 << pin, GPIO_PXPAT0C(port));

        break;

    case GPIO_FUNC_1:
        writel(1 << pin, GPIO_PXINTC(port));
        writel(1 << pin, GPIO_PXMSKC(port));
        writel(1 << pin, GPIO_PXPAT1C(port));
        writel(1 << pin, GPIO_PXPAT0S(port));

        break;

    case GPIO_FUNC_2:
        writel(1 << pin, GPIO_PXINTC(port));
        writel(1 << pin, GPIO_PXMSKC(port));
        writel(1 << pin, GPIO_PXPAT1S(port));
        writel(1 << pin, GPIO_PXPAT0C(port));

        break;

    case GPIO_FUNC_3:
        writel(1 << pin, GPIO_PXINTC(port));
        writel(1 << pin, GPIO_PXMSKC(port));
        writel(1 << pin, GPIO_PXPAT1S(port));
        writel(1 << pin, GPIO_PXPAT0S(port));

        break;
    }

    return 0;
}

void gpio_enable_pa30_as_fall_edge_irq()
{
    writel(1 << 30, GPIO_PXFLGC(0));
    writel(1 << 30, GPIO_PXINTS(0));
    writel(1 << 30, GPIO_PXPAT1S(0));
    writel(1 << 30, GPIO_PXPAT0C(0));
    writel(1 << 30, GPIO_PXMSKC(0));

    writel(1 << 17, INTC_BASE + IMCR_OFF);
}

void gpio_disable_ack_pa30_irq()
{
    writel(1 << 17, INTC_BASE + IMSR_OFF);

    writel(1 << 30, GPIO_PXMSKC(0));
    writel(1 << 30, GPIO_PXFLGC(0));
}

