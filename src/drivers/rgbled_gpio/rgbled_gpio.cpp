/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rgbled.cpp
 *
 * Driver for the onboard RGB LED controller (TCA62724FMG) connected via I2C.
 *
 * @author Julian Oes <julian@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_getopt.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>

#include <px4_workqueue.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_rgbled.h>

#define RGBLED_PWM_MAX_TIMER      3
#define RGBLED_PWM_MAX_CHANNEL    3
#define RGBLED_ONTIME 120
#define RGBLED_OFFTIME 120


#define ADDR			PX4_I2C_OBDEV_LED	/**< I2C adress of TCA62724FMG */
#define RGBLED_PWM_SCALE          78


#define REG(_tmr, _reg)	(*(volatile uint32_t *)(pwm_timers[_tmr].base + _reg))

#define rCR1(_tmr)    	REG(_tmr, STM32_GTIM_CR1_OFFSET)
#define rCR2(_tmr)    	REG(_tmr, STM32_GTIM_CR2_OFFSET)
#define rSMCR(_tmr)   	REG(_tmr, STM32_GTIM_SMCR_OFFSET)
#define rDIER(_tmr)   	REG(_tmr, STM32_GTIM_DIER_OFFSET)
#define rSR(_tmr)     	REG(_tmr, STM32_GTIM_SR_OFFSET)
#define rEGR(_tmr)    	REG(_tmr, STM32_GTIM_EGR_OFFSET)
#define rCCMR1(_tmr)  	REG(_tmr, STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2(_tmr)  	REG(_tmr, STM32_GTIM_CCMR2_OFFSET)
#define rCCER(_tmr)   	REG(_tmr, STM32_GTIM_CCER_OFFSET)
#define rCNT(_tmr)    	REG(_tmr, STM32_GTIM_CNT_OFFSET)
#define rPSC(_tmr)    	REG(_tmr, STM32_GTIM_PSC_OFFSET)
#define rARR(_tmr)    	REG(_tmr, STM32_GTIM_ARR_OFFSET)
#define rCCR1(_tmr)   	REG(_tmr, STM32_GTIM_CCR1_OFFSET)
#define rCCR2(_tmr)   	REG(_tmr, STM32_GTIM_CCR2_OFFSET)
#define rCCR3(_tmr)   	REG(_tmr, STM32_GTIM_CCR3_OFFSET)
#define rCCR4(_tmr)   	REG(_tmr, STM32_GTIM_CCR4_OFFSET)
#define rDCR(_tmr)    	REG(_tmr, STM32_GTIM_DCR_OFFSET)
#define rDMAR(_tmr)   	REG(_tmr, STM32_GTIM_DMAR_OFFSET)
#define rBDTR(_tmr)	REG(_tmr, STM32_ATIM_BDTR_OFFSET)

/* array of timers dedicated to PWM servo use */
struct rgb_led_pwm_timer {
	uint32_t	base;
	uint32_t	clock_register;
	uint32_t	clock_bit;
	uint32_t	clock_freq;
};

/* array of channels in logical order */
struct rgb_led_pwm_channel {
	uint32_t	gpio;
	uint8_t		timer_index;
	uint8_t		timer_channel;
	uint8_t     default_value;
};

class RGBLED_GPIO : public device::CDev
{
public:
	RGBLED_GPIO(int bus, int rgbled);
	virtual ~RGBLED_GPIO();


	virtual int		init();
	virtual int		probe();
	virtual int		info();
	virtual int		ioctl(device::file_t *filp, int cmd, unsigned long arg);

private:
	work_s			_work;

	rgbled_mode_t		_mode;
	rgbled_pattern_t	_pattern;

	uint8_t			_r;
	uint8_t			_g;
	uint8_t			_b;
    bool            _enable;
	float			_brightness;
	float			_max_brightness;

	bool			_running;
	int			_led_interval;
	bool			_should_run;
	int			_counter;
	int			_param_sub;

    const struct rgb_led_pwm_timer pwm_timers[RGBLED_PWM_MAX_TIMER] = {
    	{
    		.base = STM32_TIM3_BASE,
    		.clock_register = STM32_RCC_APB1ENR,
    		.clock_bit = RCC_APB1ENR_TIM3EN,
    		.clock_freq = STM32_APB1_TIM3_CLKIN
    	},
    	{
    		.base = STM32_TIM4_BASE,
    		.clock_register = STM32_RCC_APB1ENR,
    		.clock_bit = RCC_APB1ENR_TIM4EN,
    		.clock_freq = STM32_APB1_TIM4_CLKIN
    	},
    	{
    		.base = STM32_TIM2_BASE,
    		.clock_register = STM32_RCC_APB1ENR,
    		.clock_bit = RCC_APB1ENR_TIM2EN,
    		.clock_freq = STM32_APB1_TIM2_CLKIN
    	}
    };

    const struct rgb_led_pwm_channel pwm_channels[RGBLED_PWM_MAX_CHANNEL] = {
    	{
    		.gpio = RGB_BLUE_LED_PWM,
    		.timer_index = 2,
    		.timer_channel = 1,
    		.default_value = 0,
    	},
    	{
    		.gpio = RGB_GREEN_LED_PWM,
    		.timer_index = 1,
    		.timer_channel = 1,
    		.default_value = 0,
    	},
    	{
    		.gpio = RGB_RED_LED_PWM,
    		.timer_index = 0,
    		.timer_channel = 2,
    		.default_value = 0,
    	},
    };

	void 			set_color(rgbled_color_t ledcolor);
	void			set_mode(rgbled_mode_t mode);
	void			set_pattern(rgbled_pattern_t *pattern);

	static void		led_trampoline(void *arg);
	void			led();

	int			send_led_enable(bool enable);
	int			send_led_rgb();
	int			get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b);
	void		update_params();

    void        pwm_timer_init(unsigned timer);
    void        pwm_timer_channel_init(unsigned channel);
};

/* for now, we only support one RGBLED */
namespace
{
RGBLED_GPIO *g_rgbled = nullptr;
}

void rgbled_usage();

extern "C" __EXPORT int rgbled_main(int argc, char *argv[]);

RGBLED_GPIO::RGBLED_GPIO(int bus, int rgbled) :
	CDev("rgbled", RGBLED0_DEVICE_PATH, 0),
	_mode(RGBLED_MODE_OFF),
	_r(0),
	_g(0),
	_b(0),
	_brightness(1.0f),
	_max_brightness(1.0f),
	_running(false),
	_led_interval(0),
	_should_run(false),
	_counter(0),
	_param_sub(-1)
{
	memset(&_work, 0, sizeof(_work));
	memset(&_pattern, 0, sizeof(_pattern));
}

RGBLED_GPIO::~RGBLED_GPIO()
{
}

void
RGBLED_GPIO::pwm_timer_init(unsigned timer)
{
	/* enable the timer clock before we try to talk to it */
	modifyreg32(pwm_timers[timer].clock_register, 0, pwm_timers[timer].clock_bit);

	/* disable and configure the timer */
	rCR1(timer) = 0;
	rCR2(timer) = 0;
	rSMCR(timer) = 0;
	rDIER(timer) = 0;
	rCCER(timer) = 0;
	rCCMR1(timer) = 0;
	rCCMR2(timer) = 0;
	rCCER(timer) = 0;
	rDCR(timer) = 0;

	if ((pwm_timers[timer].base == STM32_TIM1_BASE) || (pwm_timers[timer].base == STM32_TIM8_BASE)) {
		/* master output enable = on */
		rBDTR(timer) = ATIM_BDTR_MOE;
	}

	/* configure the timer to free-run at 1MHz */
	rPSC(timer) = (pwm_timers[timer].clock_freq / 1000000) - 1;

	/* configure the timer to update at the desired rate */
	rARR(timer) = 256 * RGBLED_PWM_SCALE - 1;

	/* generate an update event; reloads the counter and all registers */
	rEGR(timer) = GTIM_EGR_UG;
}

void
RGBLED_GPIO::pwm_timer_channel_init(unsigned channel)
{
	unsigned timer = pwm_channels[channel].timer_index;

	/* configure the GPIO first */
	stm32_configgpio(pwm_channels[channel].gpio);

	/* configure the channel */
	switch (pwm_channels[channel].timer_channel) {
	case 1:
		rCCMR1(timer) |= (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC1M_SHIFT) | GTIM_CCMR1_OC1PE;
		rCCR1(timer) = pwm_channels[channel].default_value * RGBLED_PWM_SCALE;
		rCCER(timer) |= GTIM_CCER_CC1E;
		break;

	case 2:
		rCCMR1(timer) |= (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC2M_SHIFT) | GTIM_CCMR1_OC2PE;
		rCCR2(timer) = pwm_channels[channel].default_value * RGBLED_PWM_SCALE;
		rCCER(timer) |= GTIM_CCER_CC2E;
		break;

	case 3:
		rCCMR2(timer) |= (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR2_OC3M_SHIFT) | GTIM_CCMR2_OC3PE;
		rCCR3(timer) = pwm_channels[channel].default_value * RGBLED_PWM_SCALE;
		rCCER(timer) |= GTIM_CCER_CC3E;
		break;

	case 4:
		rCCMR2(timer) |= (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR2_OC4M_SHIFT) | GTIM_CCMR2_OC4PE;
		rCCR4(timer) = pwm_channels[channel].default_value * RGBLED_PWM_SCALE;
		rCCER(timer) |= GTIM_CCER_CC4E;
		break;
	}
}

int
RGBLED_GPIO::init()
{
	int ret;
    int i;

    ret = CDev::init();
    if (ret != OK) {
        return ret;
    }

    for (i = 0; i < RGBLED_PWM_MAX_TIMER; i++) {
		if (pwm_timers[i].base != 0) {
			pwm_timer_init(i);
		}
    }

    for (i = 0; i < RGBLED_PWM_MAX_CHANNEL; i++) {
        if (pwm_channels[i].timer_channel != 0) {
            pwm_timer_channel_init(i);
        }
    }

    for (i = 0; i < RGBLED_PWM_MAX_TIMER; i++) {
        /* arm requires the timer be enabled */
        rCR1(i) |= GTIM_CR1_CEN | GTIM_CR1_ARPE;
    }
    /* switch off LED on start */
	send_led_enable(false);
	send_led_rgb();

	return OK;
}

int
RGBLED_GPIO::probe()
{
	return OK;
}

int
RGBLED_GPIO::info()
{
	int ret;
	bool on, powersave;
	uint8_t r, g, b;

	ret = get(on, powersave, r, g, b);

	if (ret == OK) {
		/* we don't care about power-save mode */
		log("state: %s", on ? "ON" : "OFF");
		log("red: %u, green: %u, blue: %u", (unsigned)r, (unsigned)g, (unsigned)b);

	} else {
		warnx("failed to read led");
	}

	return ret;
}

int
RGBLED_GPIO::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = ENOTTY;

	switch (cmd) {
	case RGBLED_SET_RGB:
		/* set the specified color */
		_r = ((rgbled_rgbset_t *) arg)->red;
		_g = ((rgbled_rgbset_t *) arg)->green;
		_b = ((rgbled_rgbset_t *) arg)->blue;
		send_led_rgb();
		return OK;

	case RGBLED_SET_COLOR:
		/* set the specified color name */
		set_color((rgbled_color_t)arg);
		send_led_rgb();
		return OK;

	case RGBLED_SET_MODE:
		/* set the specified mode */
		set_mode((rgbled_mode_t)arg);
		return OK;

	case RGBLED_SET_PATTERN:
		/* set a special pattern */
		set_pattern((rgbled_pattern_t *)arg);
		return OK;

	default:
		/* see if the parent class can make any use of it */
#ifdef __PX4_NUTTX
		ret = CDev::ioctl(filp, cmd, arg);
#else
		ret = VDev::ioctl(filp, cmd, arg);
#endif
		break;
	}

	return ret;
}


void
RGBLED_GPIO::led_trampoline(void *arg)
{
	RGBLED_GPIO *rgbl = reinterpret_cast<RGBLED_GPIO *>(arg);

	rgbl->led();
}

/**
 * Main loop function
 */
void
RGBLED_GPIO::led()
{
	if (!_should_run) {
		_running = false;
		return;
	}

	if (_param_sub < 0) {
		_param_sub = orb_subscribe(ORB_ID(parameter_update));
	}

	if (_param_sub >= 0) {
		bool updated = false;
		orb_check(_param_sub, &updated);

		if (updated) {
			parameter_update_s pupdate;
			orb_copy(ORB_ID(parameter_update), _param_sub, &pupdate);
			update_params();
			// Immediately update to change brightness
			send_led_rgb();
		}
	}

	switch (_mode) {
	case RGBLED_MODE_BLINK_SLOW:
	case RGBLED_MODE_BLINK_NORMAL:
	case RGBLED_MODE_BLINK_FAST:
		if (_counter >= 2) {
			_counter = 0;
		}

		send_led_enable(_counter == 0);

		break;

	case RGBLED_MODE_BREATHE:

		if (_counter >= 62) {
			_counter = 0;
		}

		int n;

		if (_counter < 32) {
			n = _counter;

		} else {
			n = 62 - _counter;
		}

		_brightness = n * n / (31.0f * 31.0f);
		send_led_rgb();
		break;

	case RGBLED_MODE_PATTERN:

		/* don't run out of the pattern array and stop if the next frame is 0 */
		if (_counter >= RGBLED_PATTERN_LENGTH || _pattern.duration[_counter] <= 0) {
			_counter = 0;
		}

		set_color(_pattern.color[_counter]);
		send_led_rgb();
		_led_interval = _pattern.duration[_counter];
		break;

	default:
		break;
	}

	_counter++;

	/* re-queue ourselves to run again later */
	work_queue(LPWORK, &_work, (worker_t)&RGBLED_GPIO::led_trampoline, this, _led_interval);
}

/**
 * Parse color constant and set _r _g _b values
 */
void
RGBLED_GPIO::set_color(rgbled_color_t color)
{
	switch (color) {
	case RGBLED_COLOR_OFF:
		_r = 0;
		_g = 0;
		_b = 0;
		break;

	case RGBLED_COLOR_RED:
		_r = 255;
		_g = 0;
		_b = 0;
		break;

	case RGBLED_COLOR_YELLOW:
		_r = 255;
		_g = 200;
		_b = 0;
		break;

	case RGBLED_COLOR_PURPLE:
		_r = 255;
		_g = 0;
		_b = 255;
		break;

	case RGBLED_COLOR_GREEN:
		_r = 0;
		_g = 255;
		_b = 0;
		break;

	case RGBLED_COLOR_BLUE:
		_r = 0;
		_g = 0;
		_b = 255;
		break;

	case RGBLED_COLOR_WHITE:
		_r = 255;
		_g = 255;
		_b = 255;
		break;

	case RGBLED_COLOR_AMBER:
		_r = 255;
		_g = 80;
		_b = 0;
		break;

	case RGBLED_COLOR_DIM_RED:
		_r = 90;
		_g = 0;
		_b = 0;
		break;

	case RGBLED_COLOR_DIM_YELLOW:
		_r = 80;
		_g = 30;
		_b = 0;
		break;

	case RGBLED_COLOR_DIM_PURPLE:
		_r = 45;
		_g = 0;
		_b = 45;
		break;

	case RGBLED_COLOR_DIM_GREEN:
		_r = 0;
		_g = 90;
		_b = 0;
		break;

	case RGBLED_COLOR_DIM_BLUE:
		_r = 0;
		_g = 0;
		_b = 90;
		break;

	case RGBLED_COLOR_DIM_WHITE:
		_r = 30;
		_g = 30;
		_b = 30;
		break;

	case RGBLED_COLOR_DIM_AMBER:
		_r = 80;
		_g = 20;
		_b = 0;
		break;

	default:
		warnx("color unknown");
		break;
	}
}

/**
 * Set mode, if mode not changed has no any effect (doesn't reset blinks phase)
 */
void
RGBLED_GPIO::set_mode(rgbled_mode_t mode)
{
	if (mode != _mode) {
		_mode = mode;

		switch (mode) {
		case RGBLED_MODE_OFF:
			_should_run = false;
			send_led_enable(false);
			break;

		case RGBLED_MODE_ON:
			_brightness = 1.0f;
			send_led_rgb();
			send_led_enable(true);
			break;

		case RGBLED_MODE_BLINK_SLOW:
			_should_run = true;
			_counter = 0;
			_led_interval = 2000;
			_brightness = 1.0f;
			send_led_rgb();
			break;

		case RGBLED_MODE_BLINK_NORMAL:
			_should_run = true;
			_counter = 0;
			_led_interval = 500;
			_brightness = 1.0f;
			send_led_rgb();
			break;

		case RGBLED_MODE_BLINK_FAST:
			_should_run = true;
			_counter = 0;
			_led_interval = 100;
			_brightness = 1.0f;
			send_led_rgb();
			break;

		case RGBLED_MODE_BREATHE:
			_should_run = true;
			_counter = 0;
			_led_interval = 25;
			send_led_enable(true);
			break;

		case RGBLED_MODE_PATTERN:
			_should_run = true;
			_counter = 0;
			_brightness = 1.0f;
			send_led_enable(true);
			break;

		default:
			warnx("mode unknown");
			break;
		}

		/* if it should run now, start the workq */
		if (_should_run && !_running) {
			_running = true;
			work_queue(LPWORK, &_work, (worker_t)&RGBLED_GPIO::led_trampoline, this, 1);
		}

	}
}

/**
 * Set pattern for PATTERN mode, but don't change current mode
 */
void
RGBLED_GPIO::set_pattern(rgbled_pattern_t *pattern)
{
	memcpy(&_pattern, pattern, sizeof(rgbled_pattern_t));
}

/**
 * Sent ENABLE flag to LED driver
 */
int
RGBLED_GPIO::send_led_enable(bool enable)
{
    if (enable)
    {
        send_led_rgb();
    }
    else
    {
        rCCR1(2) = 0;
        rCCR1(1) = 0;
        rCCR2(0) = 0;
    }
    _enable = enable;
    return 0;
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED_GPIO::send_led_rgb()
{
	/* To scale from 0..255 -> 0..15 shift right by 4 bits */
    rCCR1(2) = _b * RGBLED_PWM_SCALE;
    rCCR1(1) = _g * RGBLED_PWM_SCALE;
    rCCR2(0) = _r * RGBLED_PWM_SCALE;

    return OK;
}

int
RGBLED_GPIO::get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b)
{
	int ret = OK;

    on = _enable;
    powersave = 0;
    /* XXX check, looks wrong */
    r = _r;
    g = _g;
    b = _b;

	return ret;
}

void
RGBLED_GPIO::update_params()
{
	int32_t maxbrt = 15;
	param_get(param_find("LED_RGB_MAXBRT"), &maxbrt);
	maxbrt = maxbrt > 15 ? 15 : maxbrt;
	maxbrt = maxbrt <  0 ?  0 : maxbrt;

	// A minimum of 2 "on" steps is required for breathe effect
	if (maxbrt == 1) {
		maxbrt = 2;
	}

	_max_brightness = maxbrt / 15.0f;
}

void
rgbled_usage()
{
	warnx("missing command: try 'start', 'test', 'info', 'off', 'stop', 'rgb 30 40 50'");
	warnx("options:");
	warnx("    -b i2cbus (%d)", PX4_I2C_BUS_LED);
	warnx("    -a addr (0x%x)", ADDR);
}

int
rgbled_main(int argc, char *argv[])
{
	int i2cdevice = -1;
	int rgbledadr = ADDR; /* 7bit */

	int ch;

	/* jump over start/off/etc and look at options first */
	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "a:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			rgbledadr = strtol(myoptarg, NULL, 0);
			break;

		case 'b':
			i2cdevice = strtol(myoptarg, NULL, 0);
			break;

		default:
			rgbled_usage();
			return 1;
		}
	}

	if (myoptind >= argc) {
		rgbled_usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	int fd;
	int ret;

	if (!strcmp(verb, "start")) {
		if (g_rgbled != nullptr) {
			warnx("already started");
			return 1;
		}

		if (i2cdevice == -1) {
			// try the external bus first
			i2cdevice = PX4_I2C_BUS_EXPANSION;
			g_rgbled = new RGBLED_GPIO(PX4_I2C_BUS_EXPANSION, rgbledadr);

			if (g_rgbled != nullptr && OK != g_rgbled->init()) {
				delete g_rgbled;
				g_rgbled = nullptr;
			}

			if (g_rgbled == nullptr) {
				// fall back to default bus
				if (PX4_I2C_BUS_LED == PX4_I2C_BUS_EXPANSION) {
					warnx("no RGB led on bus #%d", i2cdevice);
					return 1;
				}

				i2cdevice = PX4_I2C_BUS_LED;
			}
		}

		if (g_rgbled == nullptr) {
			g_rgbled = new RGBLED_GPIO(i2cdevice, rgbledadr);

			if (g_rgbled == nullptr) {
				warnx("new failed");
				return 1;
			}

			if (OK != g_rgbled->init()) {
				delete g_rgbled;
				g_rgbled = nullptr;
				warnx("no RGB led on bus #%d", i2cdevice);
				return 1;
			}
		}

		return 0;
	}

	/* need the driver past this point */
	if (g_rgbled == nullptr) {
		warnx("not started");
		rgbled_usage();
		return 1;
	}

	if (!strcmp(verb, "test")) {
		fd = px4_open(RGBLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			warnx("Unable to open " RGBLED0_DEVICE_PATH);
			return 1;
		}

		rgbled_pattern_t pattern = { {RGBLED_COLOR_RED, RGBLED_COLOR_GREEN, RGBLED_COLOR_BLUE, RGBLED_COLOR_WHITE, RGBLED_COLOR_OFF, RGBLED_COLOR_OFF},
			{500, 500, 500, 500, 1000, 0 }	// "0" indicates end of pattern
		};

		ret = px4_ioctl(fd, RGBLED_SET_PATTERN, (unsigned long)&pattern);
		ret = px4_ioctl(fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_PATTERN);

		px4_close(fd);
		return ret;
	}

	if (!strcmp(verb, "info")) {
		g_rgbled->info();
		return 0;
	}

	if (!strcmp(verb, "off") || !strcmp(verb, "stop")) {
		fd = px4_open(RGBLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			warnx("Unable to open " RGBLED0_DEVICE_PATH);
			return 1;
		}

		ret = px4_ioctl(fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_OFF);
		px4_close(fd);

		/* delete the rgbled object if stop was requested, in addition to turning off the LED. */
		if (!strcmp(verb, "stop")) {
			delete g_rgbled;
			g_rgbled = nullptr;
			return 0;
		}

		return ret;
	}

	if (!strcmp(verb, "rgb")) {
		if (argc < 5) {
			warnx("Usage: rgbled rgb <red> <green> <blue>");
			return 1;
		}

		fd = px4_open(RGBLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			warnx("Unable to open " RGBLED0_DEVICE_PATH);
			return 1;
		}

		rgbled_rgbset_t v;
		v.red   = strtol(argv[2], NULL, 0);
		v.green = strtol(argv[3], NULL, 0);
		v.blue  = strtol(argv[4], NULL, 0);
		ret = px4_ioctl(fd, RGBLED_SET_RGB, (unsigned long)&v);
		ret = px4_ioctl(fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_ON);
		px4_close(fd);
		return ret;
	}

	rgbled_usage();
	return 1;
}
