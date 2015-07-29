/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
 * @file servo_ctl.c
 *
 * gripper servo via GPIO driver.
 *
 * @author Vinh Nguyen
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <poll.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_pwm_output.h>
#include <modules/px4iofirmware/protocol.h>

struct servo_ctl_s {
	struct work_s work;
	int gpio_fd;
	bool use_io;
	int pin;
};

static struct servo_ctl_s *servo_ctl_data;
static int servo_ctl_pos = 0;
bool use_io;
int pin;

int		set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate);
int		_pwm_alt_rate_channels;
int 	_pwm_default_rate;
int 	_pwm_alt_rate;

__EXPORT int servo_ctl_main(int argc, char *argv[]);

void servo_ctl_pos1(FAR void *arg);

void servo_ctl_pos2(FAR void *arg);

void servo_ctl_run(FAR void *arg);

void servo_ctl_stop(FAR void *arg);

int servo_ctl_main(int argc, char *argv[])
{
	if (argc < 2) {

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
		errx(1, "usage: servo_ctl {pos1|pos2|stop} [-p <n>]\n"
		     "\t-p <n>\tUse specified AUX OUT pin number (default: 1)"
		    );
#endif

	} else {

		if (!strcmp(argv[1], "pos1")) {
			if (servo_ctl_pos == 1) {
				errx(1, "already at pos1");
			}

			use_io = false;

			/* by default use GPIO_EXT_1 on FMUv1 and servo_ctl_1 on FMUv2 */
			pin = 1;

			/* pin name to display */

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
			char pin_name[] = "AUX OUT 1";
#endif

			if (argc > 2) {
				if (!strcmp(argv[2], "-p")) {

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
					unsigned int n = strtoul(argv[3], NULL, 10);

					if (n >= 1 && n <= 6) {
						use_io = false;
						pin = 1 << (n - 1);
						snprintf(pin_name, sizeof(pin_name), "AUX OUT %d", n);

					} else {
						errx(1, "unsupported pin: %s", argv[3]);
					}

#endif
				}
			}

			servo_ctl_data = malloc(sizeof(struct servo_ctl_s));
			memset(servo_ctl_data, 0, sizeof(struct servo_ctl_s));
			servo_ctl_data->use_io = use_io;
			servo_ctl_data->pin = pin;
			warnx("HPWORK is queue %d", HPWORK);
			warnx("LPWORK is queue %d", LPWORK);
			//int ret = work_queue(LPWORK, &servo_ctl_data->work, servo_ctl_pos1, servo_ctl_data, 0);
			servo_ctl_pos1(servo_ctl_data);

			/*if (ret != 0) {
				errx(1, "failed to queue work: %d", ret);

			} else {
				servo_ctl_pos = 1;
				warnx("start, using pin: %s", pin_name);
				exit(0);
			}*/
			servo_ctl_pos = 1;
			warnx("pos1, using pin: %s", pin_name);
			exit(0);

		} else if (!strcmp(argv[1], "pos2")) {
			if (servo_ctl_pos == 2) {
				errx(1, "already at pos2");
			}
			use_io = false;
			pin = 1;


			servo_ctl_data = malloc(sizeof(struct servo_ctl_s));
			memset(servo_ctl_data, 0, sizeof(struct servo_ctl_s));
			servo_ctl_data->use_io = use_io;
			servo_ctl_data->pin = pin;
			warnx("HPWORK is queue %d", HPWORK);
			warnx("LPWORK is queue %d", LPWORK);
			servo_ctl_pos2(servo_ctl_data);
/*
			if (ret != 0) {
				errx(1, "failed to queue work: %d", ret);

			} else {
				servo_ctl_pos = 2;
				warnx("start, using pin: %d", pin);
				exit(0);
			}*/
			servo_ctl_pos = 2;
			warnx("pos2, using pin: %d", pin);
			exit(0);

		} else if (!strcmp(argv[1], "stop")) {
			if (servo_ctl_pos != 0) {
				servo_ctl_pos = 0;
				warnx("stop");

				use_io = false;
				pin = 1;
				servo_ctl_data = malloc(sizeof(struct servo_ctl_s));
				memset(servo_ctl_data, 0, sizeof(struct servo_ctl_s));
				servo_ctl_data->use_io = use_io;
				servo_ctl_data->pin = pin;
				warnx("HPWORK is queue %d", HPWORK);
				warnx("LPWORK is queue %d", LPWORK);
				servo_ctl_stop(servo_ctl_data);
				exit(0);

			} else {
				errx(1, "not running");
			}

		} else {
			errx(1, "unrecognized command '%s', only supporting 'pos1', 'pos2', or 'stop'", argv[1]);
		}
	}
}

int set_pwm_rate(uint32_t rate_map, unsigned default_rate, unsigned alt_rate)
{
	for (unsigned pass = 0; pass < 2; pass++) {
		for (unsigned group = 0; group < 6; group++) {

			// get the channel mask for this rate group
			uint32_t mask = up_pwm_servo_get_rate_group(group);

			if (mask == 0)
				continue;

			// all channels in the group must be either default or alt-rate
			uint32_t alt = rate_map & mask;

			if (pass == 0) {
				// preflight
				if ((alt != 0) && (alt != mask)) {
					warn("rate group %u mask %x bad overlap %x", group, mask, alt);
					// not a legal map, bail
					return 0;
				}

			} else {
				// set it - errors here are unexpected
				if (alt != 0) {
					if (up_pwm_servo_set_rate_group_update(group, _pwm_alt_rate) != OK) {
						warn("rate group set alt failed");
						return 0;
					}

				} else {
					if (up_pwm_servo_set_rate_group_update(group, _pwm_default_rate) != OK) {
						warn("rate group set default failed");
						return 0;
					}
				}
			}
		}
	}

	_pwm_alt_rate_channels = rate_map;
	_pwm_default_rate = default_rate;
	_pwm_alt_rate = alt_rate;

	return OK;
}

//takes a pointer to another servo_ctl_s struct
void servo_ctl_pos1(FAR void *arg)
{
	warnx("trying to start servo_ctl");
	FAR struct servo_ctl_s *priv = (FAR struct servo_ctl_s *)arg;

	char *gpio_dev;

	// sets all actions to the FMU/AUX pins
	gpio_dev = PX4FMU_DEVICE_PATH;

	// open GPIO device
	priv->gpio_fd = open(gpio_dev, 0);

	if (priv->gpio_fd < 0) {
		// TODO find way to print errors
		printf("servo_ctl: GPIO device \"%s\" open fail\n", gpio_dev);
		servo_ctl_pos = 0;
		return;
	}

	// configure GPIO pin
	// px4fmu only, px4io doesn't support GPIO_SET_OUTPUT and will ignore
	//ioctl(priv->gpio_fd, GPIO_SET_OUTPUT, priv->pin);

	// sets PWM values

	// initializes all FMU ports as servos, w/ magic numbers (mask value), taken from fmu.cpp
	up_pwm_servo_init(0xff);

	// set initial rate
	_pwm_default_rate = 50;
	_pwm_alt_rate = 50;
	set_pwm_rate(0, _pwm_default_rate, _pwm_alt_rate);

	int ret = ioctl(priv->gpio_fd, PWM_SERVO_SET_ARM_OK, 0);

	if (ret != OK) {
		err(1, "PWM_SERVO_SET_ARM_OK");
	}
	else {
		warnx("Set arm is good");
	}

	// tell IO that the system is armed (it will output values if safety is off)
	ret = ioctl(priv->gpio_fd, PWM_SERVO_ARM, 0);

	if (ret != OK) {
		err(1, "PWM_SERVO_ARM");
	}
	else{
		warnx("Armed and ready");
	}

	// sets PWM values and activates servo
	ret = ioctl(priv->gpio_fd, PWM_SERVO_SET((priv->pin)-1), 2000);

	if (ret != OK) {
		err(1, "PWM_SERVO_SET(%d)", priv->pin);
	}
	else{
		warnx("should have pwm of 2000");
	}

	if (ret != 0) {
		// TODO find way to print errors
		//printf("servo_ctl: failed to queue work: %d\n", ret);
		servo_ctl_pos = 0;
		return;
	}
}

void servo_ctl_pos2(FAR void *arg)
{
	FAR struct servo_ctl_s *priv = (FAR struct servo_ctl_s *)arg;

	char *gpio_dev;

	// sets all actions to the FMU/AUX pins
	gpio_dev = PX4FMU_DEVICE_PATH;

	// open GPIO device
	priv->gpio_fd = open(gpio_dev, 0);

	if (priv->gpio_fd < 0) {
		// TODO find way to print errors
		printf("servo_ctl: GPIO device \"%s\" open fail\n", gpio_dev);
		servo_ctl_pos = 0;
		return;
	}

	// configure GPIO pin
	// px4fmu only, px4io doesn't support GPIO_SET_OUTPUT and will ignore
	//ioctl(priv->gpio_fd, GPIO_SET_OUTPUT, priv->pin);

	// sets PWM values

	// initializes all FMU ports as servos, w/ magic numbers (mask value), taken from fmu.cpp
	up_pwm_servo_init(0xff);

	// set initial rate
	_pwm_default_rate = 50;
	_pwm_alt_rate = 50;
	set_pwm_rate(0, _pwm_default_rate, _pwm_alt_rate);

	int ret = ioctl(priv->gpio_fd, PWM_SERVO_SET_ARM_OK, 0);

	if (ret != OK) {
		err(1, "PWM_SERVO_SET_ARM_OK");
	}
	else{
		warnx("servo set arm is a go");
	}

	// tell IO that the system is armed (it will output values if safety is off)
	ret = ioctl(priv->gpio_fd, PWM_SERVO_ARM, 0);

	if (ret != OK) {
		err(1, "PWM_SERVO_ARM");
	}
	else{
		warnx("servo legged");
	}

	// sets PWM values and activates servo
	ret = ioctl(priv->gpio_fd, PWM_SERVO_SET((priv->pin)-1), 800);

	if (ret != OK) {
		err(1, "PWM_SERVO_SET(%d)", priv->pin);
	}
	else{
		warnx("the wheel keeps on turning");
	}
}

void servo_ctl_stop(FAR void *arg)
{
	FAR struct servo_ctl_s *priv = (FAR struct servo_ctl_s *)arg;
	//ioctl(priv->gpio_fd, GPIO_CLEAR, priv->pin);

	char *gpio_dev;

	// sets all actions to the FMU/AUX pins
	gpio_dev = PX4FMU_DEVICE_PATH;

	// open GPIO device
	priv->gpio_fd = open(gpio_dev, 0);

	if (priv->gpio_fd < 0) {
		// TODO find way to print errors
		printf("servo_ctl: GPIO device \"%s\" open fail\n", gpio_dev);
		servo_ctl_pos = 0;
		return;
	}

	//returns to original position
	int ret = ioctl(priv->gpio_fd, PWM_SERVO_SET(priv->pin),1100);
	if (ret != OK) {
			err(1, "PWM_SERVO_SET(%d)", priv->pin);
		}
	else {
		warnx("hold your horses");
	}

	ret = ioctl(priv->gpio_fd, PWM_SERVO_DISARM, 0);

	if (ret != OK) {
		err(1, "PWM_SERVO_DISARM");
	}
	else {
		warnx("go to sleep");
	}
}
