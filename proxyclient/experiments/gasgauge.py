#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
import sys, pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parents[1]))

from m1n1.setup import *
from m1n1 import asm

ULCON    = 0x000
UCON     = 0x004
UFCON    = 0x008
UMCON    = 0x00c
UTRSTAT  = 0x010
UFSTAT   = 0x018
UTXH     = 0x020
URXH     = 0x024
UBRDIV   = 0x028
UFRACVAL = 0x02c


UCON_TXTHRESH_ENA = (1 << 13)
UCON_RXTHRESH_ENA = (1 << 12)
UCON_RXTO_ENA_L   = (1 << 11)
UCON_RXTO_ENA     = (1 << 9)
UCON_BREAK        = (1 << 4)
UCON_STOPBIT      = (1 << 2)

UCON_MODE_OFF = 0
UCON_MODE_IRQ = 1

UTRSTAT_RXTO     = (1 << 9)
UTRSTAT_TXTHRESH = (1 << 5)
UTRSTAT_RXTHRESH = (1 << 4)
UTRSTAT_RXTO_L   = (1 << 3)
UTRSTAT_TXE      = (1 << 2)
UTRSTAT_TXBE     = (1 << 1)
UTRSTAT_RXD      = (1 << 0)

UFSTAT_TXFULL    = (1 << 9)
UFSTAT_RXFULL    = (1 << 8)

UART_CLOCK = 24000000

GG_BAUD_RATE = 57600
GG_BIT_SET   = 0xFE
GG_BIT_UNSET = 0xC0

# corellium reg defintions
GG_Temperature          = 0x06
GG_Voltage              = 0x08
GG_Flags                = 0x0a
GG_RemainingCapacity    = 0x10
GG_FullChargeCapacity   = 0x12
GG_AverageCurrent       = 0x14
GG_CycleCount           = 0x2a

# nick chan's reg definitions
GG_DesignCapacity       = 0x3c


GG_Flags_OTC            = (1 << 15)
GG_Flags_OTD            = (1 << 14)
GG_Flags_BATHI          = (1 << 13)
GG_Flags_BATLOW         = (1 << 12)
GG_Flags_CHGINH         = (1 << 11)
GG_Flags_FC             = (1 << 9)
GG_Flags_CHG            = (1 << 8)
GG_Flags_OCVTAKEN       = (1 << 7)
GG_Flags_ISD            = (1 << 6)
GG_Flags_TDD            = (1 << 5)
GG_Flags_REV            = (1 << 4)
GG_Flags_SOC1           = (1 << 2)
GG_Flags_SOCF           = (1 << 1)
GG_Flags_DSG            = (1 << 0)

# implement uart in python until we are sure how gas gauge works to avoid infinite looping

uart_base = u.adt["arm-io/uart5"].get_reg(0)[0]

def uart_init():
    ()
    #p.mask32(uart_base + UCON, 0x78af, 0x5)
    #p.write32(uart_base + UMCON, 0x1)

def uart_putbyte(c):
    while (not (p.read32(uart_base + UTRSTAT) & UTRSTAT_TXBE)): ()
    p.write32(uart_base + UTXH, c)

def uart_getbyte():
    while (not(p.read32(uart_base + UTRSTAT) & UTRSTAT_RXD)): ()
    return p.read32(uart_base + URXH)

def uart_write(l):
    for c in l:
        uart_putbyte(c)

def uart_read(len):
    list = []
    for i in range(len):
        list.append(uart_getbyte())
    return list

def uart_flush():
    while (not (p.read32(uart_base + UTRSTAT) & UTRSTAT_TXE)): ()

def uart_setbaud(baud):
    p.write32(uart_base + UBRDIV, ((UART_CLOCK // baud + 7) // 16) - 1)

def uart_enable_stopbit(enable):
    if (enable):
        p.set32(uart_base + UCON, UCON_STOPBIT)
    else:
        p.clear32(uart_base + UCON, UCON_STOPBIT)

def uart_enable_break(enable):
    if (enable):
        p.set32(uart_base + UCON, UCON_BREAK)
    else:
        p.clear32(uart_base + UCON, UCON_BREAK)

def uart_drain():
    while (p.read32(uart_base + UTRSTAT) & UTRSTAT_RXD):
        p.read32(uart_base + URXH)

def gg_xfer(tx, txbit, rxbit):
    tx_buf = []
    for i in range(txbit):
        if (tx & (1 << i)):
            tx_buf.append(GG_BIT_SET)
        else:
            tx_buf.append(GG_BIT_UNSET)

    uart_enable_break(True)
    p.udelay(250)

    uart_drain()

    uart_enable_break(False)
    p.udelay(150)

    print("sending", txbit, "byte buffer", tx_buf)

    uart_write(tx_buf)
    rx_buf = uart_read(txbit + rxbit)

    print("received", txbit + rxbit, "byte buffer", rx_buf)

    # gg will always echo
    for i in range(txbit):
        if (tx & (1 << i)):
            current = GG_BIT_SET
        else:
            current = GG_BIT_UNSET

        if rx_buf[i] != current:
            raise Exception("gg echo mismatch:", current, "!=", rx_buf[i])

    # if not trying to read, we can just return
    if not rxbit:
        return

    retval = 0
    for i in range(rxbit):
        if rx_buf[i + txbit] > 0xf8:
            retval |= (1 << i)

    return retval

def gg_read8(reg):
    return gg_xfer(reg, 8, 8)

def gg_write8(reg, data):
    reg &= 0xff
    reg |= 0x80
    reg |= ((data & 0xFF) << 8)
    return gg_xfer(reg, 16, 0)

def gg_read16(reg):
    hi = gg_read8(reg + 1)
    lo = gg_read8(reg)

    return (hi << 8 | lo)

p.pmgr_adt_clocks_enable("/arm-io/uart5")

uart_init()
uart_setbaud(GG_BAUD_RATE)
uart_enable_stopbit(True)
uart_drain()

Temperature = gg_read16(GG_Temperature)
Voltage = gg_read16(GG_Voltage)
Flags = gg_read16(GG_Flags)
RemainingCapacity = gg_read16(GG_RemainingCapacity)
FullChargeCapacity = gg_read16(GG_FullChargeCapacity)
DesignCapacity = gg_read16(GG_DesignCapacity)
AverageCurrent = gg_read16(GG_AverageCurrent)
CycleCount = gg_read16(GG_CycleCount)


print("Temperature", (Temperature - 2732) / 10, "Celsius")
print("Voltage", Voltage, "mV")
print(f"Flags 0x{Flags:x}")
print(f"RemainingCapacity {RemainingCapacity} mAh")
print(f"FullChargeCapacity {FullChargeCapacity} mAh")
print(f"DesignCapacity {DesignCapacity} mAh")
print("AverageCurrent", AverageCurrent)
print("CycleCount", CycleCount)
