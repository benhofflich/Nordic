/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <nrfx_pdm.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif

#define _pin_clk 26
#define _pin_din 27

static int16_t buff1[32767];
static int16_t buffer_length = 32767;
int16_t *p_buff1 = &buff1[0];

static void drv_pdm_hand(const nrfx_pdm_evt_t *evt)
{
  nrfx_err_t error = 0;
  if((*evt).buffer_requested){
    error = nrfx_pdm_buffer_set(p_buff1, buffer_length);
  }
}

static void audio_init()
{
  nrfx_err_t err;
  nrfx_pdm_config_t config1 = NRFX_PDM_DEFAULT_CONFIG(_pin_clk, _pin_din);
  config1.gain_l = 0x01;
  config1.gain_r = 0x01;
  
  
  err = nrfx_pdm_init(&config1, drv_pdm_hand);

  err = nrfx_pdm_start();

}

void main(void)
{
	const struct device *dev;
	bool led_is_on = true;
	int ret;

	audio_init();



	dev = device_get_binding(LED0);
	if (dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}

	while (1) {
		gpio_pin_set(dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		k_msleep(SLEEP_TIME_MS);
	}
}