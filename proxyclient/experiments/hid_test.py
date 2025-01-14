#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
import sys, pathlib, time, random, array
sys.path.append(str(pathlib.Path(__file__).resolve().parents[1]))

from m1n1.setup import *
from m1n1 import asm


code = u.malloc(0x1000)

util = asm.ARMAsm(f"""
mrs_hid6:
    mrs x0, S3_0_c15_c6_0
    ret
                  
msr_hid6:
    msr S3_0_c15_c6_0, x0
    ret
""", code)

iface.writemem(code, util.data)
p.dc_cvau(code, len(util.data))
p.ic_ivau(code, len(util.data))

p.cpufreq_init()
p.smp_start_secondaries()

p.set_exc_guard(GUARD.SKIP)

hid6 = p.smp_call_sync(1, util.mrs_hid6 | REGION_RX_EL1)

p.set_exc_guard(GUARD.OFF)

print(hex(hid6))