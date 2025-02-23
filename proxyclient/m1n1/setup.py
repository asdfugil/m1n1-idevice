# SPDX-License-Identifier: MIT
import os, struct, sys, time

from .hv import HV
from .proxy import *
from .proxyutils import *
from .sysreg import *
from .tgtypes import *
from .utils import *
from .hw.pmu import PMU

# Create serial connection
iface = UartInterface()
# Construct m1n1 proxy layer over serial connection
p = M1N1Proxy(iface, debug=False)
# Customise parameters of proxy and serial port
# based on information sent over the connection
bootstrap_port(iface, p)

# Initialise the Proxy interface from values fetched from
# the remote end
u = ProxyUtils(p)
# Build a Register Monitoring object on Proxy Interface
mon = RegMonitor(u)
hv = HV(iface, p, u)

fb = u.ba.video.base

chip_id = u.adt["/chosen"].chip_id

if chip_id not in (0x8960, 0x7000, 0x7001, 0x8000, 0x8001, 0x8003, 0x8010, 0x8012, 0x8015):
	PMU(u).reset_panic_counter()

print(f"m1n1 base: 0x{u.base:x}")

if chip_id not in (0x8960, 0x7000, 0x7001, 0x8000, 0x8001, 0x8003, 0x8010, 0x8012, 0x8015):
	PMU(u).reset_panic_counter()
