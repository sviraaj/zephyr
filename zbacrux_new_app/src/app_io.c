#include "app_io.h"

static void button_pressed(struct device *dev, struct gpio_callback *cb,
			   u32_t pins)
{

	if (pins & BIT(DT_ALIAS_SW0_GPIOS_PIN)) {
	} else {
	}
}

int app_io_init(struct device* gpio, struct device* pwm)
{
    int ret = 0;
	static struct gpio_callback button_cb;

    if (!gpio || !pwm)
    {
        error_log("GPIO IO null\n");
        return -1;
    }

	ret = gpio_pin_configure(gpio, DT_ALIAS_SW0_GPIOS_PIN,
			   (GPIO_DIR_IN | GPIO_PUD_PULL_UP | GPIO_INT | GPIO_INT_EDGE |
			    GPIO_INT_ACTIVE_LOW));
    if (ret != 0)
    {
        error_log("%d, %d", __LINE__, ret);
    }

	ret = gpio_pin_configure(gpio, DT_ALIAS_SW1_GPIOS_PIN,
			   (GPIO_DIR_IN | GPIO_PUD_PULL_UP | GPIO_INT | GPIO_INT_EDGE |
			    GPIO_INT_ACTIVE_LOW));
    if (ret != 0)
    {
        error_log("%d, %d", __LINE__, ret);
    }

	ret = gpio_pin_configure(gpio, DT_ALIAS_SW2_GPIOS_PIN,
			   (GPIO_DIR_IN | GPIO_PUD_PULL_UP | GPIO_INT | GPIO_INT_EDGE |
			    GPIO_INT_ACTIVE_LOW));
    if (ret != 0)
    {
        error_log("%d, %d", __LINE__, ret);
    }

	ret = gpio_pin_configure(gpio, DT_ALIAS_SW3_GPIOS_PIN,
			   (GPIO_DIR_IN | GPIO_PUD_PULL_UP | GPIO_INT | GPIO_INT_EDGE |
			    GPIO_INT_ACTIVE_LOW));
    if (ret != 0)
    {
        error_log("%d, %d", __LINE__, ret);
    }

	gpio_init_callback(&button_cb, button_pressed,
			   BIT(DT_ALIAS_SW0_GPIOS_PIN) | BIT(DT_ALIAS_SW1_GPIOS_PIN) |
			   BIT(DT_ALIAS_SW2_GPIOS_PIN) | BIT(DT_ALIAS_SW3_GPIOS_PIN));

	ret = gpio_add_callback(gpio, &button_cb);
    if (ret != 0)
    {
        error_log("%d, %d", __LINE__, ret);
    }

	ret = gpio_pin_enable_callback(gpio, DT_ALIAS_SW0_GPIOS_PIN);
    if (ret != 0)
    {
        error_log("%d, %d", __LINE__, ret);
    }

	ret = gpio_pin_enable_callback(gpio, DT_ALIAS_SW1_GPIOS_PIN);
    if (ret != 0)
    {
        error_log("%d, %d", __LINE__, ret);
    }

	ret = gpio_pin_enable_callback(gpio, DT_ALIAS_SW2_GPIOS_PIN);
    if (ret != 0)
    {
        error_log("%d, %d", __LINE__, ret);
    }

	ret = gpio_pin_enable_callback(gpio, DT_ALIAS_SW3_GPIOS_PIN);
    if (ret != 0)
    {
        error_log("%d, %d", __LINE__, ret);
    }

    return ret;
}
