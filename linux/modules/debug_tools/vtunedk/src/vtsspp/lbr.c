/*
  Copyright (C) 2010-2012 Intel Corporation.  All Rights Reserved.

  This file is part of SEP Development Kit

  SEP Development Kit is free software; you can redistribute it
  and/or modify it under the terms of the GNU General Public License
  version 2 as published by the Free Software Foundation.

  SEP Development Kit is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with SEP Development Kit; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA

  As a special exception, you may use this file as part of a free software
  library without restriction.  Specifically, if other files instantiate
  templates or use macros or inline functions from this file, or you compile
  this file and link it with other files to produce an executable, this
  file does not by itself cause the resulting executable to be covered by
  the GNU General Public License.  This exception does not however
  invalidate any other reasons why the executable file might be covered by
  the GNU General Public License.
*/
#include "vtss_config.h"
#include "lbr.h"
#include "globals.h"

#define DEBUGCTL_MSR        0x01d9
#define LBR_ENABLE_MASK_P4  0x0021
#define LBR_ENABLE_MASK_P6  0x0201  ///0x0001

static int vtss_lbr_no       = 0;
static int vtss_lbr_msr_ctl  = 0;
static int vtss_lbr_msr_from = 0;
static int vtss_lbr_msr_to   = 0;
static int vtss_lbr_msr_tos  = 0;
static int vtss_lbr_msr_sel  = 0;

void* vtss_lbr_correct_ip(void* ip)
{
/* TODO: Temporary turn off for investigation */
#if 0
    int lbr_idx;
    long long msr_val; /* Should be signed for ((val << 1) >> 1) */

    if (vtss_lbr_no && !vtss_lbr_msr_ctl && (reqcfg.trace_cfg.trace_flags & VTSS_CFGTRACE_LASTBR)) {
        rdmsrl(vtss_lbr_msr_tos, msr_val);
        lbr_idx = msr_val ? (int)msr_val - 1 : vtss_lbr_no - 1;
        rdmsrl(vtss_lbr_msr_to + lbr_idx, msr_val);
        TRACE("ip=0x%p, to=0x%llX", ip, msr_val);
        if ((size_t)ip == (size_t)msr_val) {
            rdmsrl(vtss_lbr_msr_from + lbr_idx, msr_val);
            TRACE("from=0x%llX", msr_val);
            return (void*)(size_t)((msr_val << 1) >> 1);
        } else
            return (void*)((char*)ip - 1);
    }
#endif
    return ip;
}

/* start LBR collection on the processor */
void vtss_lbr_enable(void)
{
    unsigned long long msr_val;

    if ((hardcfg.family == 0x06 || hardcfg.family == 0x0f) && vtss_lbr_no) {
        if (vtss_lbr_msr_sel) {
            wrmsrl(vtss_lbr_msr_sel, 0ULL);
        }
        rdmsrl(DEBUGCTL_MSR, msr_val);
        msr_val |= (hardcfg.family == 0x0f) ? LBR_ENABLE_MASK_P4 : LBR_ENABLE_MASK_P6;
        wrmsrl(DEBUGCTL_MSR, msr_val);
    }
}

/* stop LBR collection on the processor */
void vtss_lbr_disable(void)
{
    unsigned long long msr_val;

    if (hardcfg.family == 0x06 || hardcfg.family == 0x0f) {
        rdmsrl(DEBUGCTL_MSR, msr_val);
        msr_val &= (hardcfg.family == 0x0f) ? ~LBR_ENABLE_MASK_P4 : ~LBR_ENABLE_MASK_P6;
        wrmsrl(DEBUGCTL_MSR, msr_val);
    }
}

/* initialize the architectural LBR parameters */
int vtss_lbr_init(void)
{
    /* zero the LBR configuration by default */
    vtss_lbr_no       = 0;
    vtss_lbr_msr_ctl  = 0;
    vtss_lbr_msr_from = 0;
    vtss_lbr_msr_to   = 0;
    vtss_lbr_msr_tos  = 0;
    vtss_lbr_msr_sel  = 0;

    /* test the current architecture */
    if (hardcfg.family == 0x06) {
        switch (hardcfg.model) {
        /* NHM/SNB/IVB */
        case 0x1a:
        case 0x1e:
        case 0x1f:
        case 0x2e:
        case 0x25:
        case 0x2c:
        case 0x2a:
        case 0x2d:
        case 0x3a:
            vtss_lbr_no       = 16;
            vtss_lbr_msr_from = 0x0680;
            vtss_lbr_msr_to   = 0x06c0;
            vtss_lbr_msr_tos  = 0x01c9;
            vtss_lbr_msr_sel  = 0x01c8;
            break;
        /* Atoms */
        case 0x1c:
        case 0x35:
        case 0x36:
            vtss_lbr_no       = 8;
            vtss_lbr_msr_from = 0x40;
            vtss_lbr_msr_to   = 0x60;
            vtss_lbr_msr_tos  = 0x01c9;
            break;
        /* Core2s */
        case 0x1d:
        case 0x17:
        case 0x0f:
            vtss_lbr_no       = 4;
            vtss_lbr_msr_from = 0x40;
            vtss_lbr_msr_to   = 0x60;
            vtss_lbr_msr_tos  = 0x01c9;
            break;
        default:
            if (hardcfg.model >= 0x02 && hardcfg.model < 0x0f) {
                vtss_lbr_no       = 8;
                vtss_lbr_msr_ctl  = 0x40;
                vtss_lbr_msr_tos  = 0x01c9;
            }
            break;
        }
    } else if (hardcfg.family == 0x0f) {
        if (hardcfg.model >= 0x03) {
            vtss_lbr_no       = 16;
            vtss_lbr_msr_from = 0x0680;
            vtss_lbr_msr_to   = 0x06c0;
            vtss_lbr_msr_tos  = 0x01da;
        } else {
            vtss_lbr_no       = 4;
            vtss_lbr_msr_ctl  = 0x01db;
            vtss_lbr_msr_tos  = 0x01da;
        }
    }
    TRACE("no=%d, ctl=0x%X, from=0x%X, to=0x%X, tos=0x%X",
          vtss_lbr_no, vtss_lbr_msr_ctl, vtss_lbr_msr_from, vtss_lbr_msr_to, vtss_lbr_msr_tos);
    return 0;
}

static void vtss_lbr_on_each_cpu_func(void* ctx)
{
    vtss_lbr_disable();
}

void vtss_lbr_fini(void)
{
    on_each_cpu(vtss_lbr_on_each_cpu_func, NULL, SMP_CALL_FUNCTION_ARGS);
}
