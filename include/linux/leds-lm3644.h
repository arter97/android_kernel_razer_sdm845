#ifndef _LEDS_LM3644_H
#define _LEDS_LM3644_H

/* Add prepare function for cam_flash_core */
#include <linux/leds.h>
#define LM3644_ENABLE_REGULATOR		BIT(0)
#define LM3644_DISABLE_REGULATOR	BIT(1)
#define LM3644_QUERY_MAX_CURRENT	BIT(2)
#define LM3644_PREPARE_OPTIONS_MASK	GENMASK(3, 0)
int lm3644_flash_led_prepare(struct led_trigger *trig, int options, int *max_current);

#endif /* end of _LEDS_LM3644_H */
