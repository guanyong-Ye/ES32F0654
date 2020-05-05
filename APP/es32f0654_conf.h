
#ifndef __ES32F0654_CONF_H__
#define __ES32F0654_CONF_H__

#include "utils.h"

#define xKEY_ANFULAI
#define KEY_INTERUPT

#ifdef ALD_CMU
	#include "ald_cmu.h"
#endif
#ifdef ALD_GPIO
	#include "ald_gpio.h"
#endif

#ifdef ALD_DMA
	#include "ald_dma.h"
#endif

#endif
