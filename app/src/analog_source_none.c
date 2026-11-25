/* The analog source on a SoC with no backend: every start is refused. */

#include <errno.h>

#include <zephyr/sys/util.h>

#include "analog_source.h"

static int none_prepare(const struct analog_source_params *params, struct analog_source_info *info)
{
	ARG_UNUSED(params);
	ARG_UNUSED(info);
	return -ENOTSUP;
}

static int none_start(analog_source_block_cb cb, void *ctx)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(ctx);
	return -ENOTSUP;
}

static void none_stop(void)
{
}

const struct analog_source_backend analog_source_backend = {
	.prepare = none_prepare,
	.start = none_start,
	.stop = none_stop,
};
