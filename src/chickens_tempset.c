/* SPDX-License-Identifier: MIT */

#include "cpu_regs.h"
#include "utils.h"

void init_common_tempset(void)
{
    /*
     * "Prevent ordered loads from being dispatched from LSU until all prior loads have completed."
     * "AF2 ordering rules allow ARM device ordering violations"
     */
    reg_set(SYS_IMP_APL_EHID4, EHID4_FORCE_NS_ORD_LD_REQ_NO_OLDER_LD);

    // "Poisoned younger load is not redirected by older load-acquire"
    reg_set(SYS_IMP_APL_EHID3, EHID3_DISABLE_COLOR_OPT);

    // "Disable the extension of prefetcher training pipe clock gating, revert to default gating"
    reg_set(SYS_IMP_APL_EHID10, EHID10_DISABLE_RCC_PWR_SAVE_PRF_CLK_OFF);
}
