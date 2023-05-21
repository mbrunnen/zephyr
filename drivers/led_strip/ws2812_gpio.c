/*
 * Copyright (c) 2018 Intel Corporation
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2021 Seagate Technology LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/dt-bindings/led/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT worldsemi_ws2812_gpio

LOG_MODULE_REGISTER(ws2812_gpio);
#define LOG_LEVEL CONFIG_LED_STRIP_LOG_LEVEL

#define DELAY_CORR 350
#define DELAY_T0H  NS_TO_SYS_CLOCK_HW_CYCLES(350 - DELAY_CORR)
#define DELAY_T0L  NS_TO_SYS_CLOCK_HW_CYCLES(900 - DELAY_CORR)
#define DELAY_T1H  NS_TO_SYS_CLOCK_HW_CYCLES(900 - DELAY_CORR)
#define DELAY_T1L  NS_TO_SYS_CLOCK_HW_CYCLES(350 - DELAY_CORR)

#define NS_TO_SYS_CLOCK_HW_CYCLES(ns)                                                              \
	((uint64_t)sys_clock_hw_cycles_per_sec() * (ns) / NSEC_PER_SEC + 1)

struct ws2812_gpio_cfg {
	struct gpio_dt_spec in_gpio;
	uint8_t num_colors;
	const uint8_t *color_mapping;
};

static void ws2812_gpio_delay(uint32_t cycles_to_wait)
{
	uint32_t start = k_cycle_get_32();

	/* Wait until the given number of cycles have passed */
	while ((k_cycle_get_32() - start) < cycles_to_wait) {
	}
}

static inline void ws2812_write_bit(const struct gpio_dt_spec *gpio, uint8_t bit)
{
	if (bit > 0) {
		gpio_pin_set_dt(gpio, 1);
		ws2812_gpio_delay(DELAY_T1H);
		gpio_pin_set_dt(gpio, 0);
		ws2812_gpio_delay(DELAY_T1L);
	} else {
		gpio_pin_set_dt(gpio, 1);
		ws2812_gpio_delay(DELAY_T0H);
		gpio_pin_set_dt(gpio, 0);
		ws2812_gpio_delay(DELAY_T0L);
	}
}

static int ws2812_gpio_start(const struct device *dev, uint8_t *buf, size_t len)
{
	const struct ws2812_gpio_cfg *config = dev->config;
	const struct gpio_dt_spec *in_gpio = &config->in_gpio;
	int rc = 0;

	while (len--) {
		uint8_t byte = *buf++;
		uint8_t mask = BIT(7);

		do {
			ws2812_write_bit(in_gpio, byte & mask);
		} while (mask >>= 1);
	}
	return rc;
}

static int ws2812_gpio_update_rgb(const struct device *dev, struct led_rgb *pixels,
				  size_t num_pixels)
{
	const struct ws2812_gpio_cfg *config = dev->config;
	uint8_t *ptr = (uint8_t *)pixels;
	size_t i;

	/* Convert from RGB to on-wire format (e.g. GRB, GRBW, RGB, etc) */
	for (i = 0; i < num_pixels; i++) {
		uint8_t j;

		for (j = 0; j < config->num_colors; j++) {
			switch (config->color_mapping[j]) {
			/* White channel is not supported by LED strip API. */
			case LED_COLOR_ID_WHITE:
				*ptr++ = 0;
				break;
			case LED_COLOR_ID_RED:
				*ptr++ = pixels[i].r;
				break;
			case LED_COLOR_ID_GREEN:
				*ptr++ = pixels[i].g;
				break;
			case LED_COLOR_ID_BLUE:
				*ptr++ = pixels[i].b;
				break;
			default:
				return -EINVAL;
			}
		}
	}

	return ws2812_gpio_start(dev, (uint8_t *)pixels, num_pixels * config->num_colors);
}

static int ws2812_gpio_update_channels(const struct device *dev, uint8_t *channels,
				       size_t num_channels)
{
	LOG_ERR("update_channels not implemented");
	return -ENOTSUP;
}

static const struct led_strip_driver_api ws2812_gpio_api = {
	.update_rgb = ws2812_gpio_update_rgb,
	.update_channels = ws2812_gpio_update_channels,
};

/*
 * Retrieve the channel to color mapping (e.g. RGB, BGR, GRB, ...) from the
 * "color-mapping" DT property.
 */
#define WS2812_COLOR_MAPPING(idx)                                                                  \
	static const uint8_t ws2812_gpio_##idx##_color_mapping[] = DT_INST_PROP(idx, color_mapping)

#define WS2812_NUM_COLORS(idx) (DT_INST_PROP_LEN(idx, color_mapping))

/*
 * The inline assembly above is designed to work on nRF51 devices with
 * the 16 MHz clock enabled.
 *
 * TODO: try to make this portable, or at least port to more devices.
 */

#define WS2812_GPIO_DEVICE(idx)                                                                    \
                                                                                                   \
	static int ws2812_gpio_##idx##_init(const struct device *dev)                              \
	{                                                                                          \
		const struct ws2812_gpio_cfg *cfg = dev->config;                                   \
		uint8_t i;                                                                         \
                                                                                                   \
		if (!device_is_ready(cfg->in_gpio.port)) {                                         \
			LOG_ERR("GPIO device not ready");                                          \
			return -ENODEV;                                                            \
		}                                                                                  \
                                                                                                   \
		for (i = 0; i < cfg->num_colors; i++) {                                            \
			switch (cfg->color_mapping[i]) {                                           \
			case LED_COLOR_ID_WHITE:                                                   \
			case LED_COLOR_ID_RED:                                                     \
			case LED_COLOR_ID_GREEN:                                                   \
			case LED_COLOR_ID_BLUE:                                                    \
				break;                                                             \
			default:                                                                   \
				LOG_ERR("%s: invalid channel to color mapping."                    \
					" Check the color-mapping DT property",                    \
					dev->name);                                                \
				return -EINVAL;                                                    \
			}                                                                          \
		}                                                                                  \
                                                                                                   \
		return gpio_pin_configure_dt(&cfg->in_gpio, GPIO_OUTPUT);                          \
	}                                                                                          \
                                                                                                   \
	WS2812_COLOR_MAPPING(idx);                                                                 \
                                                                                                   \
	static const struct ws2812_gpio_cfg ws2812_gpio_##idx##_cfg = {                            \
		.in_gpio = GPIO_DT_SPEC_INST_GET(idx, in_gpios),                                   \
		.num_colors = WS2812_NUM_COLORS(idx),                                              \
		.color_mapping = ws2812_gpio_##idx##_color_mapping,                                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, ws2812_gpio_##idx##_init, NULL, NULL, &ws2812_gpio_##idx##_cfg, \
			      POST_KERNEL, CONFIG_LED_STRIP_INIT_PRIORITY, &ws2812_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_GPIO_DEVICE)
