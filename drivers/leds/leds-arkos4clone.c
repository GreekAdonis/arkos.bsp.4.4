/*
 * Arkos4Clone LED 驱动程序
 *
 * LED 分类（独立兼容模式，所有类型可共存）：
 *
 * 第一类：电源灯
 *   - led-gpio: 双色 LED，用于充电指示
 *   - led-red, led-blue: 独立 LED，用于充电指示
 *
 * 第二类：摇杆灯（独立初始化，可共存）
 *   - joy-green, joy-red, joy-blue: RGB 三色 LED
 *   - joy-left, joy-right: 左右 LED
 *   - pulse-gpios, irq-gpios: 单线脉冲协议 RGB LED
 *
 * 兼容性：
 *   - v1/v2 硬件共用同一 DTB
 *   - GPIO 申请失败时自动跳过该 LED
 *   - 所有 LED 类型独立初始化，互不影响
 *
 * 充电指示功能（仅电源灯）：
 *   - 充电中：高电平颜色亮（brightness=1）
 *   - 充满：低电平颜色亮（brightness=0）
 *   - 未充电：允许 sysfs 控制
 *   - 用户可设置：0/1/2（低电平颜色/高电平颜色/高阻态）
 *
 * 设备树示例：
 *
 *   leds: arkos4clone-leds {
 *       compatible = "arkos4clone-led";
 *       // 电源灯
 *       led-gpio = <&gpio2 RK_PB5 GPIO_ACTIVE_HIGH>;
 *       led-red = <&gpio0 RK_PC1 GPIO_ACTIVE_HIGH>;
 *       led-blue = <&gpio0 RK_PA0 GPIO_ACTIVE_HIGH>;
 *       // 摇杆灯 (v1 和 v2 共用)
 *       joy-green = <&gpio2 RK_PA1 GPIO_ACTIVE_HIGH>;
 *       joy-red = <&gpio2 RK_PA2 GPIO_ACTIVE_HIGH>;
 *       joy-blue = <&gpio2 RK_PA0 GPIO_ACTIVE_HIGH>;
 *       pulse-gpios = <&gpio0 RK_PB3 GPIO_ACTIVE_HIGH>;
 *       irq-gpios = <&gpio0 RK_PB4 GPIO_ACTIVE_HIGH>;
 *   };
 *
 * Sysfs 接口：
 *   /sys/devices/platform/arkos4clone-led/
 *     - status: 查看所有 LED 状态
 *     - gpio: 读取/设置 LED 状态
 *     - colors: 查看可用颜色
 *     - pulse: 直接发送脉冲数 (仅 pulse LED)
 *     - test: GPIO 测试 (仅 pulse LED)
 *
 *   /sys/class/leds/
 *     - arkos4clone-led/brightness: 双色 LED (0/1)
 *     - joyled/brightness: 脉冲 LED (0-255)
 *     - led-red/brightness, led-blue/brightness: 独立电源灯
 *     - joy-xxx/brightness: joystick LEDs
 *
 * Copyright (C) 2024 lcdyk0517 <lcdyk0517@qq.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

/* 最大独立 LED 数量 */
#define MAX_LEDS		7

/* 充电状态轮询间隔（毫秒） */
#define CHARGE_POLL_INTERVAL	2000

/* ===== 脉冲 LED (pulse-gpio) 定义 ===== */
/* 脉冲时序参数 */
#define PULSE_DELAY_US		1000
#define PULSE_GAP_US		5000

/* 脉冲 LED 模式 (脉冲数量编码) */
enum pulse_led_mode {
	PULSE_MODE_OFF		= 10,
	PULSE_MODE_RED		= 1,
	PULSE_MODE_RED_GREEN	= 2,
	PULSE_MODE_GREEN	= 3,
	PULSE_MODE_GREEN_BLUE	= 4,
	PULSE_MODE_BLUE		= 5,
	PULSE_MODE_BLUE_RED	= 6,
	PULSE_MODE_RED_GREEN_BLUE = 7,
	PULSE_MODE_BREATHING	= 8,
	PULSE_MODE_SCROLLING	= 9,
};

/* 独立 LED 索引 */
enum {
	LED_RED = 0,
	LED_BLUE,
	LED_JOY_GREEN,
	LED_JOY_RED,
	LED_JOY_BLUE,
	LED_JOY_LEFT,
	LED_JOY_RIGHT,
};

/* 独立 LED 结构体 */
struct arkos4clone_led {
	struct led_classdev cdev;
	struct gpio_desc *gpiod;
	bool active_low;
	bool valid;
	struct work_struct work;
	int new_level;
	int index;
	struct arkos4clone_led_priv *priv;
};

/* 前向声明 */
static int arkos4clone_pulse_led_init(struct arkos4clone_led_priv *priv);
static void arkos4clone_led_cleanup(struct arkos4clone_led *led);
static int arkos4clone_charge_monitor_init(struct arkos4clone_led_priv *priv);

/* LED classdev mode 属性前向声明 */
static ssize_t mode_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR_RW(mode);

/* LED classdev 专用属性组 (显示在 /sys/class/leds/joyled/ 下) */
static struct attribute *joyled_attrs[] = {
	&dev_attr_mode.attr,
	NULL,
};
ATTRIBUTE_GROUPS(joyled);

/* LED 名称数组，用于设备树属性匹配 */
static const char *led_names[MAX_LEDS] = {
	"led-red",
	"led-blue",
	"joy-green",
	"joy-red",
	"joy-blue",
	"joy-left",
	"joy-right",
};

/**
 * struct arkos4clone_led_priv - 驱动私有数据
 */
struct arkos4clone_led_priv {
	struct device *dev;
	int num_leds;
	struct arkos4clone_led leds[MAX_LEDS];

	/* 双色 LED */
	bool has_bicolor;
	int bicolor_gpio;
	bool bicolor_active_low;
	char bicolor_high_color[16];
	char bicolor_low_color[16];
	struct led_classdev bicolor_cdev;

	/* 脉冲 LED */
	bool has_pulse_led;
	int pulse_gpio;
	int irq_gpio;
	int irq_num;
	struct led_classdev pulse_cdev;
	int pulse_mode;

	/* 充电监控 */
	struct power_supply *psy;
	struct delayed_work charge_work;
	bool charging;
	bool full;
	bool charge_monitoring;
};

/* ===== 脉冲 LED 函数实现 ===== */

/**
 * send_pulse_count - 通过单线协议发送脉冲序列
 * @pulse_gpio: 脉冲输出 GPIO 编号
 * @pulse_count: 脉冲数量 (决定 LED 模式)
 *
 * 协议时序:
 *   1. 拉高 GPIO，延时 7 个周期 (起始信号)
 *   2. 拉低 GPIO，延时 1 个周期
 *   3. 发送 pulse_count 个数据脉冲 (每个脉冲: 高→延时→低→延时)
 *   4. 结束延时，确保低电平
 */
static void send_pulse_count(int pulse_gpio, int pulse_count)
{
	int i;

	/* 第一步: 起始信号 - 拉高并保持 7 个延时周期 */
	gpio_set_value(pulse_gpio, 1);
	for (i = 0; i < 7; i++)
		udelay(PULSE_DELAY_US);

	/* 第二步: 拉低 */
	gpio_set_value(pulse_gpio, 0);
	udelay(PULSE_DELAY_US);

	/* 第三步: 发送数据脉冲 */
	for (i = 0; i < pulse_count; i++) {
		gpio_set_value(pulse_gpio, 1);
		udelay(PULSE_DELAY_US);
		gpio_set_value(pulse_gpio, 0);
		udelay(PULSE_DELAY_US);
	}

	/* 第四步: 结束延时并确保低电平 */
	udelay(PULSE_GAP_US);
	gpio_set_value(pulse_gpio, 0);
}

/**
 * pulse_led_irq_handler - 脉冲 LED 控制器中断处理函数
 *
 * 当 LED 控制器通过 irq-gpios 发送信号时触发。
 */
static irqreturn_t pulse_led_irq_handler(int irq, void *dev_id)
{
	struct arkos4clone_led_priv *priv = dev_id;

	if (!priv)
		return IRQ_NONE;

	dev_dbg(priv->dev, "Pulse LED 控制器中断触发\n");
	return IRQ_HANDLED;
}

/**
 * arkos4clone_pulse_led_set - 设置脉冲 LED 模式
 * @led_cdev: LED 类设备指针
 * @brightness: 亮度值 (0-255)
 *
 * 亮度值映射:
 *   0:       关闭
 *   1-28:    红色
 *   29-56:   黄色
 *   57-84:   绿色
 *   85-112:  青色
 *   113-140: 蓝色
 *   141-168: 紫色
 *   169-196: 白色
 *   197-224: 心跳
 *   225-255: 彩虹
 */
static void arkos4clone_pulse_led_set(struct led_classdev *led_cdev,
				      enum led_brightness brightness)
{
	struct arkos4clone_led_priv *priv =
		container_of(led_cdev, struct arkos4clone_led_priv, pulse_cdev);
	int target_mode;

	if (!gpio_is_valid(priv->pulse_gpio))
		return;

	if (brightness == LED_OFF) {
		target_mode = PULSE_MODE_OFF;
	} else if (brightness <= 28) {
		target_mode = PULSE_MODE_RED;
	} else if (brightness <= 56) {
		target_mode = PULSE_MODE_RED_GREEN;
	} else if (brightness <= 84) {
		target_mode = PULSE_MODE_GREEN;
	} else if (brightness <= 112) {
		target_mode = PULSE_MODE_GREEN_BLUE;
	} else if (brightness <= 140) {
		target_mode = PULSE_MODE_BLUE;
	} else if (brightness <= 168) {
		target_mode = PULSE_MODE_BLUE_RED;
	} else if (brightness <= 196) {
		target_mode = PULSE_MODE_RED_GREEN_BLUE;
	} else if (brightness <= 224) {
		target_mode = PULSE_MODE_BREATHING;
	} else {
		target_mode = PULSE_MODE_SCROLLING;
	}

	if (target_mode != priv->pulse_mode) {
		send_pulse_count(priv->pulse_gpio, target_mode);
		priv->pulse_mode = target_mode;
	}
}

/**
 * arkos4clone_pulse_led_get - 获取脉冲 LED 当前状态
 * @led_cdev: LED 类设备指针
 */
static enum led_brightness arkos4clone_pulse_led_get(struct led_classdev *led_cdev)
{
	struct arkos4clone_led_priv *priv =
		container_of(led_cdev, struct arkos4clone_led_priv, pulse_cdev);
	return priv->pulse_mode ? LED_FULL : LED_OFF;
}

/* ===== 原有 LED 函数实现 ===== */

/**
 * arkos4clone_led_work - LED 工作队列处理函数
 * @work: 工作队列结构体指针
 *
 * 在进程上下文中设置 GPIO 电平，用于可能睡眠的 GPIO 操作。
 */
static void arkos4clone_led_work(struct work_struct *work)
{
	struct arkos4clone_led *led =
		container_of(work, struct arkos4clone_led, work);

	if (led->gpiod)
		gpiod_set_value_cansleep(led->gpiod, led->new_level);
}

/**
 * arkos4clone_led_set_raw - 直接设置 LED GPIO 电平
 * @led: LED 结构体指针
 * @level: 电平值（0 或 1）
 *
 * 根据 GPIO 是否可能睡眠，选择直接设置或通过工作队列设置。
 */
static void arkos4clone_led_set_raw(struct arkos4clone_led *led, int level)
{
	if (!led->gpiod)
		return;

	led->new_level = level;

	if (gpiod_cansleep(led->gpiod))
		schedule_work(&led->work);
	else
		gpiod_set_value(led->gpiod, level);
}

/**
 * arkos4clone_bicolor_set - 设置双色 LED 颜色（led-gpio）
 * @priv: 私有数据结构指针
 * @color: 颜色值
 *   0 = 低电平颜色
 *   1 = 高电平颜色
 *   2 = 高阻态
 */
static void arkos4clone_bicolor_set(struct arkos4clone_led_priv *priv, int color)
{
	int level;

	if (!gpio_is_valid(priv->bicolor_gpio))
		return;

	if (color == 2) {
		/* 高阻态：切换为输入模式 */
		gpio_direction_input(priv->bicolor_gpio);
		return;
	}

	/* 根据 active_low 标志计算实际 GPIO 电平 */
	if (priv->bicolor_active_low)
		level = color ? 0 : 1;
	else
		level = color ? 1 : 0;

	gpio_direction_output(priv->bicolor_gpio, level);
}

/**
 * arkos4clone_bicolor_get - 获取双色 LED 当前颜色（led-gpio）
 * @priv: 私有数据结构指针
 *
 * 返回：0=低电平颜色，1=高电平颜色，2=高阻态
 */
static int arkos4clone_bicolor_get(struct arkos4clone_led_priv *priv)
{
	int value;
	struct gpio_desc *gpiod;

	if (!gpio_is_valid(priv->bicolor_gpio))
		return 0;

	/* 检查是否为输入模式（高阻态） */
	gpiod = gpio_to_desc(priv->bicolor_gpio);
	if (gpiod && gpiod_get_direction(gpiod) == GPIOF_DIR_IN)
		return 2;

	value = gpio_get_value(priv->bicolor_gpio);

	/* 根据 active_low 标志转换 */
	if (priv->bicolor_active_low)
		return value ? 0 : 1;
	else
		return value ? 1 : 0;
}

/**
 * arkos4clone_led_set - LED 类设备的亮度设置回调
 * @led_cdev: LED 类设备指针
 * @brightness: 亮度值
 *
 * 对于 led-red 和 led-blue，在充电或充满时阻止用户控制。
 */
static void arkos4clone_led_set(struct led_classdev *led_cdev,
				enum led_brightness brightness)
{
	struct arkos4clone_led *led =
		container_of(led_cdev, struct arkos4clone_led, cdev);
	struct arkos4clone_led_priv *priv = led->priv;
	int level;

	if (!led->gpiod || !priv)
		return;

	/* 充电或充满时，阻止 led-red 和 led-blue 的用户控制 */
	if ((led->index == LED_RED || led->index == LED_BLUE)) {
		if (priv->charge_monitoring && (priv->charging || priv->full)) {
			dev_dbg(priv->dev, "%s control blocked during charging\n", led_cdev->name);
			return;
		}
	}

	level = (brightness == LED_OFF) ? 0 : 1;
	arkos4clone_led_set_raw(led, level);
}

/**
 * arkos4clone_led_get - LED 类设备的亮度获取回调
 * @led_cdev: LED 类设备指针
 *
 * 返回：当前亮度值
 */
static enum led_brightness arkos4clone_led_get(struct led_classdev *led_cdev)
{
	struct arkos4clone_led *led =
		container_of(led_cdev, struct arkos4clone_led, cdev);

	if (!led->gpiod)
		return LED_OFF;

	return gpiod_get_value_cansleep(led->gpiod) ? LED_FULL : LED_OFF;
}

/**
 * arkos4clone_bicolor_cdev_set - 双色 LED 类设备的亮度设置回调（led-gpio）
 * @led_cdev: LED 类设备指针
 * @brightness: 亮度值（0=低电平颜色，1=高电平颜色）
 *
 * 充电或充满时阻止用户控制。
 */
static void arkos4clone_bicolor_cdev_set(struct led_classdev *led_cdev,
					 enum led_brightness brightness)
{
	struct arkos4clone_led_priv *priv =
		container_of(led_cdev, struct arkos4clone_led_priv, bicolor_cdev);

	/* 充电或充满时阻止用户控制 */
	if (priv->charge_monitoring && (priv->charging || priv->full)) {
		dev_dbg(priv->dev, "led control blocked during charging\n");
		return;
	}

	arkos4clone_bicolor_set(priv, brightness);
}

/**
 * arkos4clone_bicolor_cdev_get - 双色 LED 类设备的亮度获取回调（led-gpio）
 * @led_cdev: LED 类设备指针
 *
 * 返回：当前颜色值
 */
static enum led_brightness arkos4clone_bicolor_cdev_get(struct led_classdev *led_cdev)
{
	struct arkos4clone_led_priv *priv =
		container_of(led_cdev, struct arkos4clone_led_priv, bicolor_cdev);

	return arkos4clone_bicolor_get(priv);
}

/**
 * arkos4clone_led_init - 初始化独立 LED
 * @dev: 设备指针
 * @led: LED 结构体指针
 * @name: LED 名称
 * @gpio: GPIO 编号
 * @active_low: 是否低电平有效
 * @index: LED 索引
 * @priv: 私有数据结构指针
 *
 * 返回：成功返回 0，失败返回负错误码
 */
static int arkos4clone_led_init(struct device *dev,
				struct arkos4clone_led *led,
				const char *name,
				int gpio,
				bool active_low,
				int index,
				struct arkos4clone_led_priv *priv)
{
	unsigned long flags = GPIOF_OUT_INIT_LOW;
	int ret;

	led->valid = false;
	led->active_low = active_low;
	led->index = index;
	led->priv = priv;

	if (!gpio_is_valid(gpio)) {
		dev_dbg(dev, "LED %s: GPIO not configured\n", name);
		return 0;
	}

	if (active_low)
		flags |= GPIOF_ACTIVE_LOW;

	ret = devm_gpio_request_one(dev, gpio, flags, name);
	if (ret) {
		dev_warn(dev, "LED %s: failed to request GPIO %d: %d\n",
			 name, gpio, ret);
		return ret;
	}

	led->gpiod = gpio_to_desc(gpio);
	if (!led->gpiod) {
		dev_err(dev, "LED %s: failed to get GPIO descriptor\n", name);
		return -EINVAL;
	}

	INIT_WORK(&led->work, arkos4clone_led_work);

	led->cdev.name = name;
	led->cdev.brightness_set = arkos4clone_led_set;
	led->cdev.brightness_get = arkos4clone_led_get;
	led->cdev.max_brightness = 1;
	led->cdev.brightness = LED_OFF;
	led->cdev.flags = LED_CORE_SUSPENDRESUME;

	ret = led_classdev_register(dev, &led->cdev);
	if (ret) {
		dev_err(dev, "LED %s: failed to register: %d\n", name, ret);
		return ret;
	}

	led->valid = true;
	dev_info(dev, "LED %s: GPIO %d (active-%s)\n",
		 name, gpio, active_low ? "low" : "high");

	return 0;
}

/**
 * arkos4clone_bicolor_init - 初始化双色 LED（led-gpio）
 * @priv: 私有数据结构指针
 *
 * 从设备树读取 led-gpio、led-high-color、led-low-color 属性，
 * 并注册 LED 类设备。双色 LED 只有两种状态。
 *
 * 返回：成功返回 0，失败返回负错误码
 */
static int arkos4clone_bicolor_init(struct arkos4clone_led_priv *priv)
{
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	const char *color;
	int gpio, ret;

	gpio = of_get_named_gpio_flags(np, "led-gpio", 0, &flags);
	if (!gpio_is_valid(gpio)) {
		dev_dbg(dev, "No bicolor LED (led-gpio) configured\n");
		return 0;
	}

	priv->bicolor_gpio = gpio;
	priv->bicolor_active_low = (flags & OF_GPIO_ACTIVE_LOW) != 0;

	/* 获取颜色名称 */
	color = of_get_property(np, "led-high-color", NULL);
	if (color)
		strncpy(priv->bicolor_high_color, color, sizeof(priv->bicolor_high_color) - 1);
	else
		strcpy(priv->bicolor_high_color, "red");

	color = of_get_property(np, "led-low-color", NULL);
	if (color)
		strncpy(priv->bicolor_low_color, color, sizeof(priv->bicolor_low_color) - 1);
	else
		strcpy(priv->bicolor_low_color, "blue");

	/* 申请 GPIO */
	ret = devm_gpio_request_one(dev, gpio,
				    GPIOF_OUT_INIT_LOW | (priv->bicolor_active_low ? GPIOF_ACTIVE_LOW : 0),
				    "arkos4clone-led");
	if (ret) {
		dev_err(dev, "Failed to request LED GPIO %d: %d\n", gpio, ret);
		return ret;
	}

	/* 注册 LED 类设备 */
	priv->bicolor_cdev.name = "arkos4clone-led";
	priv->bicolor_cdev.max_brightness = 2;	/* 0=低电平颜色，1=高电平颜色，2=高阻态 */
	priv->bicolor_cdev.brightness_set = arkos4clone_bicolor_cdev_set;
	priv->bicolor_cdev.brightness_get = arkos4clone_bicolor_cdev_get;
	priv->bicolor_cdev.brightness = 0;	/* 默认：低电平颜色 */
	priv->bicolor_cdev.flags = LED_CORE_SUSPENDRESUME;

	ret = led_classdev_register(dev, &priv->bicolor_cdev);
	if (ret) {
		dev_err(dev, "Failed to register LED: %d\n", ret);
		return ret;
	}

	priv->has_bicolor = true;
	dev_info(dev, "LED (bicolor): GPIO %d, high=%s, low=%s, active-%s\n",
		 gpio, priv->bicolor_high_color, priv->bicolor_low_color,
		 priv->bicolor_active_low ? "low" : "high");

	return 0;
}

/**
 * arkos4clone_pulse_led_init - 初始化脉冲 LED（pulse-gpio）
 * @priv: 私有数据结构指针
 *
 * 从设备树读取 pulse-gpios 和 irq-gpios 属性，
 * 并注册 joyled LED 类设备。
 *
 * 返回：成功返回 0，失败返回负错误码
 */
static int arkos4clone_pulse_led_init(struct arkos4clone_led_priv *priv)
{
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	int gpio, ret;

	/* 获取脉冲 GPIO (pulse-gpios) */
	gpio = of_get_named_gpio(np, "pulse-gpios", 0);
	if (!gpio_is_valid(gpio)) {
		dev_dbg(dev, "No pulse LED (pulse-gpios) configured\n");
		return 0;
	}

	priv->pulse_gpio = gpio;
	priv->irq_gpio = -EINVAL;
	priv->irq_num = -EINVAL;

	/* 申请 pulse GPIO */
	ret = devm_gpio_request_one(dev, gpio, GPIOF_OUT_INIT_LOW,
				    "arkos4clone-pulse");
	if (ret) {
		dev_err(dev, "Failed to request pulse GPIO %d: %d\n", gpio, ret);
		return ret;
	}

	/* 获取中断 GPIO (irq-gpios, 可选) */
	gpio = of_get_named_gpio(np, "irq-gpios", 0);
	if (gpio_is_valid(gpio)) {
		ret = devm_gpio_request_one(dev, gpio, GPIOF_OUT_INIT_LOW,
					    "arkos4clone-pulse-irq");
		if (ret) {
			dev_warn(dev, "Failed to request irq GPIO %d: %d\n", gpio, ret);
		} else {
			/* 复位延时后改为输入模式 */
			udelay(100);
			gpio_direction_input(gpio);
			priv->irq_gpio = gpio;
			priv->irq_num = gpio_to_irq(gpio);
			if (priv->irq_num >= 0) {
				ret = devm_request_irq(dev, priv->irq_num,
						       pulse_led_irq_handler,
						       IRQF_TRIGGER_RISING,
						       "arkos4clone-pulse-irq",
						       priv);
				if (ret) {
					dev_warn(dev, "Failed to request irq %d: %d\n",
						 priv->irq_num, ret);
					priv->irq_num = -EINVAL;
				}
			}
		}
	}

	/* 注册 joyled LED 类设备 */
	priv->pulse_cdev.name = "joyled";
	priv->pulse_cdev.brightness_set = arkos4clone_pulse_led_set;
	priv->pulse_cdev.brightness_get = arkos4clone_pulse_led_get;
	priv->pulse_cdev.max_brightness = 255;
	priv->pulse_cdev.flags = LED_CORE_SUSPENDRESUME;
	priv->pulse_cdev.groups = joyled_groups;

	ret = led_classdev_register(dev, &priv->pulse_cdev);
	if (ret) {
		dev_err(dev, "Failed to register joyled: %d\n", ret);
		return ret;
	}

	/* 发送初始化脉冲 (11脉冲 = 关闭) */
	send_pulse_count(priv->pulse_gpio, 11);
	priv->pulse_mode = 11;
	priv->has_pulse_led = true;

	dev_info(dev, "Pulse LED: GPIO %d (irq=%d)\n",
		 priv->pulse_gpio, priv->irq_gpio);

	return 0;
}

/**
 * arkos4clone_led_cleanup - 清理独立 LED 资源
 * @led: LED 结构体指针
 */
static void arkos4clone_led_cleanup(struct arkos4clone_led *led)
{
	if (led->valid) {
		led_classdev_unregister(&led->cdev);
		cancel_work_sync(&led->work);
		led->valid = false;
	}
}

/**
 * arkos4clone_update_charge_leds - 根据充电状态更新 LED
 * @priv: 私有数据结构指针
 *
 * 充电中：高电平颜色亮（brightness=1）
 * 充满：低电平颜色亮（brightness=0）
 * 未充电：允许 sysfs 控制
 */
static void arkos4clone_update_charge_leds(struct arkos4clone_led_priv *priv)
{
	/* 双色 LED 优先用于充电指示 */
	if (priv->has_bicolor) {
		if (priv->full) {
			/* 充满：低电平颜色亮 */
			arkos4clone_bicolor_set(priv, 0);
		} else if (priv->charging) {
			/* 充电中：高电平颜色亮 */
			arkos4clone_bicolor_set(priv, 1);
		}
	}

	/* 同时处理独立 led-red/led-blue（如果已配置） */
	if (priv->leds[LED_RED].valid || priv->leds[LED_BLUE].valid) {
		bool has_red = priv->leds[LED_RED].valid;
		bool has_blue = priv->leds[LED_BLUE].valid;

		if (priv->full) {
			/* 充满：蓝色亮，红色灭 */
			if (has_blue)
				arkos4clone_led_set_raw(&priv->leds[LED_BLUE], 1);
			if (has_red)
				arkos4clone_led_set_raw(&priv->leds[LED_RED], 0);
		} else if (priv->charging) {
			/* 充电中：红色亮，蓝色灭 */
			if (has_red)
				arkos4clone_led_set_raw(&priv->leds[LED_RED], 1);
			if (has_blue)
				arkos4clone_led_set_raw(&priv->leds[LED_BLUE], 0);
		}
	}
}

/**
 * arkos4clone_charge_work - 充电状态检测工作队列
 * @work: 工作队列结构体指针
 *
 * 定期轮询电源状态，检测充电和充满状态的变化。
 */
static void arkos4clone_charge_work(struct work_struct *work)
{
	struct arkos4clone_led_priv *priv =
		container_of(to_delayed_work(work), struct arkos4clone_led_priv, charge_work);
	union power_supply_propval val_status;
	bool charging = false;
	bool full = false;
	int ret;

	if (!priv->psy)
		goto reschedule;

	/* 从电池电源供应获取充电状态 */
	ret = power_supply_get_property(priv->psy, POWER_SUPPLY_PROP_STATUS, &val_status);
	if (ret) {
		dev_err(priv->dev, "Failed to get STATUS property: %d\n", ret);
		goto reschedule;
	}

	/* 根据状态判断充电和充满 */
	switch (val_status.intval) {
	case POWER_SUPPLY_STATUS_CHARGING:
		charging = true;
		full = false;
		break;
	case POWER_SUPPLY_STATUS_FULL:
		charging = false;
		full = true;
		break;
	case POWER_SUPPLY_STATUS_DISCHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
	case POWER_SUPPLY_STATUS_UNKNOWN:
	default:
		charging = false;
		full = false;
		break;
	}

	/* 仅在状态变化时更新 LED */
	if (priv->charging != charging || priv->full != full) {
		dev_dbg(priv->dev, "Power: status=%d (%s), charging=%d, full=%d\n",
			val_status.intval,
			val_status.intval == POWER_SUPPLY_STATUS_CHARGING ? "CHARGING" :
			val_status.intval == POWER_SUPPLY_STATUS_FULL ? "FULL" :
			val_status.intval == POWER_SUPPLY_STATUS_DISCHARGING ? "DISCHARGING" : "OTHER",
			charging, full);

		priv->charging = charging;
		priv->full = full;

		if (charging || full) {
			arkos4clone_update_charge_leds(priv);
		}
	}

reschedule:
	schedule_delayed_work(&priv->charge_work,
			      msecs_to_jiffies(CHARGE_POLL_INTERVAL));
}

/**
 * arkos4clone_charge_monitor_init - 初始化充电监控
 * @priv: 私有数据结构指针
 *
 * 检查是否有可用于充电指示的 LED，并尝试获取电源供应对象。
 *
 * 返回：成功返回 0
 */
static int arkos4clone_charge_monitor_init(struct arkos4clone_led_priv *priv)
{
	bool has_charge_led = false;

	/* 检查是否有可用于充电指示的 LED */
	if (priv->has_bicolor)
		has_charge_led = true;

	/* 如果配置了 led-red 或 led-blue，启用充电监控 */
	if (priv->leds[LED_RED].valid || priv->leds[LED_BLUE].valid)
		has_charge_led = true;

	if (!has_charge_led) {
		dev_info(priv->dev, "Charge monitoring disabled (no charge LED configured)\n");
		return 0;
	}

	/* 尝试获取电源供应对象 */
	priv->psy = power_supply_get_by_name("battery");
	if (priv->psy) {
		dev_dbg(priv->dev, "Found power supply: battery\n");
	} else {
		priv->psy = power_supply_get_by_name("charger");
		if (priv->psy)
			dev_dbg(priv->dev, "Found power supply: charger\n");
	}
	if (!priv->psy) {
		priv->psy = power_supply_get_by_name("usb");
		if (priv->psy)
			dev_dbg(priv->dev, "Found power supply: usb\n");
	}
	if (!priv->psy) {
		priv->psy = power_supply_get_by_name("dc");
		if (priv->psy)
			dev_dbg(priv->dev, "Found power supply: dc\n");
	}
	if (!priv->psy) {
		priv->psy = power_supply_get_by_name("mains");
		if (priv->psy)
			dev_dbg(priv->dev, "Found power supply: mains\n");
	}

	if (!priv->psy) {
		dev_info(priv->dev, "No power supply found, charge monitoring disabled\n");
		return 0;
	}

	priv->charging = false;
	priv->full = false;
	priv->charge_monitoring = true;

	INIT_DELAYED_WORK(&priv->charge_work, arkos4clone_charge_work);
	schedule_delayed_work(&priv->charge_work, msecs_to_jiffies(500));

	dev_info(priv->dev, "Charge monitoring enabled\n");
	return 0;
}

/**
 * arkos4clone_charge_monitor_exit - 退出充电监控
 * @priv: 私有数据结构指针
 */
static void arkos4clone_charge_monitor_exit(struct arkos4clone_led_priv *priv)
{
	if (priv->charge_monitoring) {
		cancel_delayed_work_sync(&priv->charge_work);
	}
	if (priv->psy) {
		power_supply_put(priv->psy);
		priv->psy = NULL;
	}
}

/**
 * status_show - 显示所有 LED 状态
 * @dev: 设备指针
 * @attr: 设备属性指针
 * @buf: 输出缓冲区
 *
 * Sysfs 接口：/sys/devices/platform/arkos4clone-led/status
 *
 * 返回：写入缓冲区的字节数
 */
static ssize_t status_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct arkos4clone_led_priv *priv = dev_get_drvdata(dev);
	int i, count = 0;

	if (priv->charge_monitoring) {
		count += sprintf(buf + count, "charging: %s\n",
				 priv->full ? "full" : priv->charging ? "yes" : "no");
	}

	if (priv->has_bicolor) {
		int color = arkos4clone_bicolor_get(priv);
		const char *color_name;
		if (color == 2)
			color_name = "off";
		else if (color == 1)
			color_name = priv->bicolor_high_color;
		else
			color_name = priv->bicolor_low_color;
		count += sprintf(buf + count, "arkos4clone-led: %d (%s)\n",
				 color, color_name);
	}

	for (i = 0; i < MAX_LEDS; i++) {
		struct arkos4clone_led *led = &priv->leds[i];

		if (led->valid) {
			int state = gpiod_get_value(led->gpiod);
			count += sprintf(buf + count, "%s: %d\n",
					 led_names[i], state);
		}
	}

	return count;
}

static DEVICE_ATTR_RO(status);

/**
 * gpio_store - 设置 LED 状态
 * @dev: 设备指针
 * @attr: 设备属性指针
 * @buf: 输入缓冲区
 * @count: 输入数据长度
 *
 * Sysfs 接口：/sys/devices/platform/arkos4clone-led/gpio
 * 格式：<led名称> <值>
 * 例如：echo "arkos4clone-led 1" > gpio
 *       echo "joy-green 1" > gpio
 *
 * 返回：处理的字节数或负错误码
 */
static ssize_t gpio_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct arkos4clone_led_priv *priv = dev_get_drvdata(dev);
	char name[32];
	int value, i;

	if (sscanf(buf, "%31s %d", name, &value) != 2)
		return -EINVAL;

	/* 双色 LED 控制 (led-gpio) */
	if (priv->has_bicolor && (!strcmp(name, "arkos4clone-led") || !strcmp(name, "led"))) {
		if (priv->charge_monitoring && (priv->charging || priv->full)) {
			dev_dbg(dev, "led control blocked during charging\n");
			return -EBUSY;
		}
		if (value == 0 || value == 1)
			arkos4clone_bicolor_set(priv, value);
		return count;
	}

	/* 独立 LED 控制 */
	for (i = 0; i < MAX_LEDS; i++) {
		if (!strcmp(name, led_names[i]) && priv->leds[i].valid) {
			/* 充电或充满时，阻止 led-red 和 led-blue 的控制 */
			if ((i == LED_RED || i == LED_BLUE) &&
			    priv->charge_monitoring && (priv->charging || priv->full)) {
				dev_dbg(dev, "%s control blocked during charging\n", name);
				return -EBUSY;
			}
			arkos4clone_led_set(&priv->leds[i].cdev,
					    value ? LED_FULL : LED_OFF);
			return count;
		}
	}

	return -ENODEV;
}

/**
 * gpio_show - 显示 LED 控制帮助和当前状态
 * @dev: 设备指针
 * @attr: 设备属性指针
 * @buf: 输出缓冲区
 *
 * Sysfs 接口：/sys/devices/platform/arkos4clone-led/gpio
 *
 * 返回：写入缓冲区的字节数
 */
static ssize_t gpio_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct arkos4clone_led_priv *priv = dev_get_drvdata(dev);
	int i, count = 0;

	if (priv->has_bicolor) {
		int color = arkos4clone_bicolor_get(priv);
		const char *color_name;
		if (color == 2)
			color_name = "off";
		else if (color == 1)
			color_name = priv->bicolor_high_color;
		else
			color_name = priv->bicolor_low_color;
		count += sprintf(buf + count, "arkos4clone-led: %d (%s)\n"
				 "  0 = %s\n"
				 "  1 = %s\n"
				 "  2 = off (high-Z)\n",
				 color, color_name,
				 priv->bicolor_low_color,
				 priv->bicolor_high_color);
	}

	for (i = 0; i < MAX_LEDS; i++) {
		if (priv->leds[i].valid) {
			count += sprintf(buf + count, "%s: %d\n",
					 led_names[i],
					 gpiod_get_value(priv->leds[i].gpiod));
		}
	}

	if (count == 0)
		count = sprintf(buf, "No LEDs configured\n");

	return count;
}

static DEVICE_ATTR_RW(gpio);

/**
 * colors_show - 显示可用颜色
 * @dev: 设备指针
 * @attr: 设备属性指针
 * @buf: 输出缓冲区
 *
 * Sysfs 接口：/sys/devices/platform/arkos4clone-led/colors
 *
 * 返回：写入缓冲区的字节数
 */
static ssize_t colors_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct arkos4clone_led_priv *priv = dev_get_drvdata(dev);
	int count = 0;

	if (priv->has_bicolor) {
		count += sprintf(buf + count, "arkos4clone-led:\n");
		count += sprintf(buf + count, "  0: %s\n", priv->bicolor_low_color);
		count += sprintf(buf + count, "  1: %s\n", priv->bicolor_high_color);
		count += sprintf(buf + count, "  2: off (high-Z)\n");
	}

	/* 显示独立 LED */
	if (priv->leds[LED_RED].valid)
		count += sprintf(buf + count, "led-red: 0/1\n");
	if (priv->leds[LED_BLUE].valid)
		count += sprintf(buf + count, "led-blue: 0/1\n");
	if (priv->leds[LED_JOY_GREEN].valid)
		count += sprintf(buf + count, "joy-green: 0/1\n");
	if (priv->leds[LED_JOY_RED].valid)
		count += sprintf(buf + count, "joy-red: 0/1\n");
	if (priv->leds[LED_JOY_BLUE].valid)
		count += sprintf(buf + count, "joy-blue: 0/1\n");
	if (priv->leds[LED_JOY_LEFT].valid)
		count += sprintf(buf + count, "joy-left: 0/1\n");
	if (priv->leds[LED_JOY_RIGHT].valid)
		count += sprintf(buf + count, "joy-right: 0/1\n");

	if (count == 0)
		count = sprintf(buf, "No LEDs configured\n");

	return count;
}

static DEVICE_ATTR_RO(colors);

/* ===== 脉冲 LED sysfs 属性 ===== */

/**
 * pulse_store - 直接发送脉冲数
 * @dev: 设备指针
 * @attr: 设备属性指针
 * @buf: 输入缓冲区
 * @count: 输入数据长度
 *
 * Sysfs 接口：/sys/devices/platform/arkos4clone-led/pulse
 * 用法: echo <脉冲数> > pulse
 *
 * 注意: 0 会转换为 10 (关闭 LED)
 */
static ssize_t pulse_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct arkos4clone_led_priv *priv = dev_get_drvdata(dev);
	unsigned long pulse_count;
	int ret;

	if (!priv->has_pulse_led)
		return -ENODEV;

	ret = kstrtoul(buf, 10, &pulse_count);
	if (ret)
		return ret;

	if (pulse_count > 255)
		return -EINVAL;

	/* 0 转换为 10 (关闭 LED) */
	if (pulse_count == 0)
		pulse_count = PULSE_MODE_OFF;

	dev_info(dev, "发送脉冲: %lu\n", pulse_count);
	send_pulse_count(priv->pulse_gpio, pulse_count);
	priv->pulse_mode = pulse_count;

	return count;
}

static DEVICE_ATTR_WO(pulse);

/* 模式字符串映射 */
static const struct {
	const char *name;
	int mode;
} pulse_mode_map[] = {
	{ "off",		PULSE_MODE_OFF },
	{ "red",		PULSE_MODE_RED },
	{ "red_green",		PULSE_MODE_RED_GREEN },
	{ "green",		PULSE_MODE_GREEN },
	{ "green_blue",		PULSE_MODE_GREEN_BLUE },
	{ "blue",		PULSE_MODE_BLUE },
	{ "blue_red",		PULSE_MODE_BLUE_RED },
	{ "red_green_blue",	PULSE_MODE_RED_GREEN_BLUE },
	{ "breathing",		PULSE_MODE_BREATHING },
	{ "scrolling",		PULSE_MODE_SCROLLING },
};

/**
 * mode_store - 设置脉冲 LED 模式（字符串）
 * @dev: 设备指针
 * @attr: 设备属性指针
 * @buf: 输入缓冲区
 * @count: 输入数据长度
 *
 * Sysfs 接口：/sys/devices/platform/arkos4clone-led/mode
 * 用法: echo red > mode
 *       echo breathing > mode
 *       echo red_green_blue > mode
 *
 * 支持的模式:
 *   off, red, red_green, green, green_blue, blue, blue_red,
 *   red_green_blue, breathing, scrolling
 */
static ssize_t mode_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct arkos4clone_led_priv *priv;
	char mode_str[32];
	int i;

	if (!led_cdev)
		return -ENODEV;

	priv = container_of(led_cdev, struct arkos4clone_led_priv, pulse_cdev);

	if (!priv->has_pulse_led)
		return -ENODEV;

	/* 复制并去除换行符 */
	if (count >= sizeof(mode_str))
		return -EINVAL;
	strncpy(mode_str, buf, count);
	mode_str[count] = '\0';
	if (mode_str[count - 1] == '\n')
		mode_str[count - 1] = '\0';

	/* 查找模式 */
	for (i = 0; i < ARRAY_SIZE(pulse_mode_map); i++) {
		if (strcasecmp(mode_str, pulse_mode_map[i].name) == 0) {
			dev_info(dev, "设置模式: %s (脉冲数: %d)\n",
				 pulse_mode_map[i].name, pulse_mode_map[i].mode);
			send_pulse_count(priv->pulse_gpio, pulse_mode_map[i].mode);
			priv->pulse_mode = pulse_mode_map[i].mode;
			return count;
		}
	}

	dev_err(dev, "未知模式: %s\n", mode_str);
	return -EINVAL;
}

/**
 * mode_show - 显示当前模式和可用模式
 * @dev: 设备指针
 * @attr: 设备属性指针
 * @buf: 输出缓冲区
 *
 * Sysfs 接口：/sys/devices/platform/arkos4clone-led/mode
 */
static ssize_t mode_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct arkos4clone_led_priv *priv;
	int i, count = 0;
	const char *current_mode = "unknown";

	if (!led_cdev)
		return sprintf(buf, "pulse LED not available\n");

	priv = container_of(led_cdev, struct arkos4clone_led_priv, pulse_cdev);

	if (!priv->has_pulse_led)
		return sprintf(buf, "pulse LED not available\n");

	/* 查找当前模式名称 */
	for (i = 0; i < ARRAY_SIZE(pulse_mode_map); i++) {
		if (priv->pulse_mode == pulse_mode_map[i].mode) {
			current_mode = pulse_mode_map[i].name;
			break;
		}
	}

	count += sprintf(buf + count, "current: %s\n\n", current_mode);
	count += sprintf(buf + count, "available modes:\n");
	for (i = 0; i < ARRAY_SIZE(pulse_mode_map); i++) {
		count += sprintf(buf + count, "  %s\n", pulse_mode_map[i].name);
	}

	return count;
}

/**
 * test_store - GPIO 测试
 * @dev: 设备指针
 * @attr: 设备属性指针
 * @buf: 输入缓冲区
 * @count: 输入数据长度
 *
 * Sysfs 接口：/sys/devices/platform/arkos4clone-led/test
 * 用法: echo 1 > test  (拉高)
 *       echo 0 > test  (拉低)
 * 用于验证 GPIO 硬件连接是否正常
 */
static ssize_t test_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct arkos4clone_led_priv *priv = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	if (!priv->has_pulse_led)
		return -ENODEV;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;

	gpio_set_value(priv->pulse_gpio, value ? 1 : 0);
	dev_info(dev, "GPIO 测试: %lu\n", value);

	return count;
}

static DEVICE_ATTR_WO(test);

/* Sysfs 属性数组 */
static struct attribute *arkos4clone_led_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_gpio.attr,
	&dev_attr_colors.attr,
	&dev_attr_pulse.attr,
	&dev_attr_mode.attr,
	&dev_attr_test.attr,
	NULL,
};

/* Sysfs 属性组 */
static const struct attribute_group arkos4clone_led_attr_group = {
	.attrs = arkos4clone_led_attrs,
};

/**
 * arkos4clone_led_remove - 平台设备移除函数
 * @pdev: 平台设备指针
 *
 * 清理所有资源。
 *
 * 返回：0
 */
static int arkos4clone_led_remove(struct platform_device *pdev)
{
	struct arkos4clone_led_priv *priv = platform_get_drvdata(pdev);
	int i;

	arkos4clone_charge_monitor_exit(priv);
	sysfs_remove_group(&pdev->dev.kobj, &arkos4clone_led_attr_group);

	/* 关闭并注销脉冲 LED */
	if (priv->has_pulse_led) {
		if (gpio_is_valid(priv->pulse_gpio))
			send_pulse_count(priv->pulse_gpio, PULSE_MODE_OFF);
		led_classdev_unregister(&priv->pulse_cdev);
	}

	if (priv->has_bicolor)
		led_classdev_unregister(&priv->bicolor_cdev);

	for (i = 0; i < MAX_LEDS; i++)
		arkos4clone_led_cleanup(&priv->leds[i]);

	return 0;
}

/**
 * arkos4clone_led_shutdown - 平台设备关机函数
 * @pdev: 平台设备指针
 *
 * 系统关机时关闭所有 LED。
 */
static void arkos4clone_led_shutdown(struct platform_device *pdev)
{
	struct arkos4clone_led_priv *priv = platform_get_drvdata(pdev);

	if (priv && priv->has_pulse_led && gpio_is_valid(priv->pulse_gpio))
		send_pulse_count(priv->pulse_gpio, PULSE_MODE_OFF);
}

/**
 * arkos4clone_led_probe - 平台设备探测函数
 * @pdev: 平台设备指针
 *
 * 解析设备树，初始化 LED，创建 Sysfs 接口，启动充电监控。
 * 所有 LED 类型独立兼容，可以共存。
 *
 * 返回：成功返回 0，失败返回负错误码
 */
static int arkos4clone_led_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct arkos4clone_led_priv *priv;
	int i, ret, count = 0;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->charge_monitoring = false;
	priv->charging = false;
	priv->full = false;
	priv->has_bicolor = false;
	priv->has_pulse_led = false;
	priv->bicolor_gpio = -EINVAL;
	priv->pulse_gpio = -EINVAL;
	priv->irq_gpio = -EINVAL;
	priv->irq_num = -EINVAL;
	platform_set_drvdata(pdev, priv);

	/* 初始化双色 LED（led-gpio，如果已配置） */
	ret = arkos4clone_bicolor_init(priv);
	if (ret)
		return ret;

	if (priv->has_bicolor)
		count++;

	/* 初始化脉冲 LED（pulse-gpio，如果已配置且硬件存在） */
	ret = arkos4clone_pulse_led_init(priv);
	if (ret) {
		dev_info(dev, "pulse LED init skipped or failed: %d\n", ret);
		/* 不返回错误，继续初始化其他 LED */
	}

	if (priv->has_pulse_led)
		count++;

	/* 初始化独立 LED */
	for (i = 0; i < MAX_LEDS; i++) {
		enum of_gpio_flags flags;
		int gpio;

		gpio = of_get_named_gpio_flags(np, led_names[i], 0, &flags);
		if (gpio_is_valid(gpio)) {
			bool active_low = (flags & OF_GPIO_ACTIVE_LOW) != 0;
			ret = arkos4clone_led_init(dev, &priv->leds[i],
						   led_names[i], gpio, active_low,
						   i, priv);
			if (ret == 0 && priv->leds[i].valid)
				count++;
		}
	}

	if (count == 0) {
		dev_warn(dev, "No LEDs configured\n");
		return -ENODEV;
	}

	priv->num_leds = count;

	/* 创建 Sysfs 接口 */
	ret = sysfs_create_group(&dev->kobj, &arkos4clone_led_attr_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs group: %d\n", ret);
		if (priv->has_pulse_led)
			led_classdev_unregister(&priv->pulse_cdev);
		if (priv->has_bicolor)
			led_classdev_unregister(&priv->bicolor_cdev);
		for (i = 0; i < MAX_LEDS; i++)
			arkos4clone_led_cleanup(&priv->leds[i]);
		return ret;
	}

	/* 初始化充电监控 */
	arkos4clone_charge_monitor_init(priv);

	dev_info(dev, "Arkos4Clone LED driver loaded (%d LED%s%s%s)\n",
		 count, count > 1 ? "s" : "",
		 priv->has_bicolor ? ", bicolor" : "",
		 priv->has_pulse_led ? ", pulse" : "");
	return 0;
}

/* 设备树匹配表 */
static const struct of_device_id arkos4clone_led_of_match[] = {
	{ .compatible = "arkos4clone-led", },
	{ },
};
MODULE_DEVICE_TABLE(of, arkos4clone_led_of_match);

/* 平台驱动结构体 */
static struct platform_driver arkos4clone_led_driver = {
	.probe = arkos4clone_led_probe,
	.remove = arkos4clone_led_remove,
	.shutdown = arkos4clone_led_shutdown,
	.driver = {
		.name = "arkos4clone-led",
		.of_match_table = arkos4clone_led_of_match,
	},
};

module_platform_driver(arkos4clone_led_driver);

MODULE_AUTHOR("lcdyk0517 <lcdyk0517@qq.com>");
MODULE_DESCRIPTION("Arkos4Clone LED 驱动：电源灯(充电指示) + 摇杆灯(三选一互斥)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:arkos4clone-led");