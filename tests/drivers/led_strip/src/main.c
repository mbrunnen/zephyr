/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/ztest.h>
#include <zephyr/fff.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>

DEFINE_FFF_GLOBALS;

struct ws2812_gpio_fixture {
	const struct device *dev;
};

static void *ws2812_gpio_setup(void)
{
	static struct ws2812_gpio_fixture fixture = {
		.dev = DEVICE_DT_GET(DT_NODELABEL(led_strip)),
	};

	zassert_not_null(fixture.dev);
	return &fixture;
}

ZTEST_SUITE(ws2812_gpio, NULL, ws2812_gpio_setup, NULL, NULL, NULL);

#define RGB(_r, _g, _b)                                                                            \
	{                                                                                          \
		.r = (_r), .g = (_g), .b = (_b)                                                    \
	}

FAKE_VOID_FUNC(ws2812_gpio_delay, uint32_t);

ZTEST_F(ws2812_gpio, test_delay)
{
	int rc = 0;

	struct led_rgb pixels[] = {
		RGB(0x0f, 0x00, 0x00),
		RGB(0x00, 0x0f, 0x00),
		RGB(0x00, 0x00, 0x0f),
	};

	rc = led_strip_update_rgb(fixture->dev, pixels, ARRAY_SIZE(pixels));
	TC_PRINT("%d", ws2812_gpio_delay_fake.call_count);
	zassert_equal(ws2812_gpio_delay_fake.call_count, 1);
}
