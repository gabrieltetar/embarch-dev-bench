/* A second log module, registered under the *exact* name main.c uses
 * (DEV_BENCH_LOG_APP_MODULE), so the app-module floor in
 * dev_bench_log_set_level can be tested for what it actually does rather than
 * only for the code path being reachable.
 *
 * A separate translation unit because LOG_MODULE_REGISTER is one-per-file, and
 * the suite's own module name has to be the one on trial here: the floor is
 * matched by string (dev_bench_log.h explains why), so a test that registered
 * some other name would exercise the comparison and never the match.
 */
#include <zephyr/logging/log.h>

#include "dev_bench_log.h"

/* DBG, matching main.c: registered at INF, this file's own LOG_DBG would be
 * compiled out and the floor test's debug half would pass without testing
 * anything -- which is exactly what happened in main.c until hardware said so. */
LOG_MODULE_REGISTER(dev_bench, LOG_LEVEL_DBG);

void app_module_emit_inf(void)
{
	LOG_INF("app module speaking");
}

void app_module_emit_dbg(void)
{
	LOG_DBG("app module debug");
}
