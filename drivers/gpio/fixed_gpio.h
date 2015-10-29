#ifndef __FIXED_GPIO_H__
#define __FIXED_GPIO_H__

struct fixed_gpio {
	int pin;
	int active_level;
};

static inline int fixed_gpio_is_valid(struct fixed_gpio gpio)
{
	return gpio.pin;
}

static inline int fixed_gpio_is_active(struct fixed_gpio gpio)
{
	return gpio.pin >= 0 ?
		gpio_get_value(gpio.pin) == gpio.active_level : 0;
}

static inline void fixed_gpio_direction_input(struct fixed_gpio gpio)
{
	if (gpio.pin >= 0)
		gpio_direction_input(gpio.pin);
}

static inline void fixed_gpio_direction_output(struct fixed_gpio gpio, int value)
{
	if (gpio.pin >= 0)
		gpio_direction_output(gpio.pin, value);
}

#ifdef CONFIG_JZ_GPIO
static inline void fixed_gpio_set_active_irq(struct fixed_gpio gpio)
{
	if (gpio.pin >= 0) {
		if(gpio.active_level == 0)
			gpio_as_irq_rise_edge(gpio.pin);
		else
			gpio_as_irq_fall_edge(gpio.pin);
		/* unmask IRQ_GPIOn depends on GPIO_WAKEUP */
		intc_unmask_gpio_irq(gpio.pin);
		gpio_ack_irq(gpio.pin);
	}
}

static inline void fixed_gpio_set_deactive_irq(struct fixed_gpio gpio)
{
	if (gpio.pin >= 0) {
		if(gpio.active_level == 1)
			gpio_as_irq_rise_edge_edge(gpio.pin);
		else
			gpio_as_irq_fall_edge(gpio.pin);
		/* unmask IRQ_GPIOn depends on GPIO_WAKEUP */
		intc_unmask_gpio_irq(gpio.pin);
		gpio_ack_irq(gpio.pin);
	}
}

static inline void fixed_gpio_disable_pull(struct fixed_gpio gpio)
{
	if (gpio.pin >= 0)
		gpio_disable_pull(gpio.pin);
}
#endif

#endif /* __FIXED_GPIO_H__ */
