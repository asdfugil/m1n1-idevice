/* SPDX-License-Identifier: MIT */

#include "usb.h"
#include "adt.h"
#include "dart.h"
#include "i2c.h"
#include "iodev.h"
#include "malloc.h"
#include "pmgr.h"
#include "soc.h"
#include "string.h"
#include "tps6598x.h"
#include "types.h"
#include "usb_complex.h"
#include "usb_dwc2.h"
#include "usb_dwc3.h"
#include "usb_dwc3_regs.h"
#include "utils.h"
#include "vsprintf.h"

struct usb_drd_regs {
    uintptr_t drd_regs;
    uintptr_t drd_regs_unk3;
    uintptr_t atc;
};

#if USB_IODEV_COUNT > 100
#error "USB_IODEV_COUNT is limited to 100 to prevent overflow in ADT path names"
#endif

// length of the format string is is used as buffer size
// limits the USB instance numbers to reasonable 2 digits
#define FMT_DART_PATH        "/arm-io/dart-usb%u"
#define FMT_DART_MAPPER_PATH "/arm-io/dart-usb%u/mapper-usb%u"
#define FMT_ATC_PATH         "/arm-io/atc-phy%u"
#define FMT_DRD_PATH         "/arm-io/usb-drd%u"
#define FMT_HPM_PATH         "/arm-io/i2c0/hpmBusManager/hpm%u"

static tps6598x_irq_state_t tps6598x_irq_state[USB_IODEV_COUNT];
static bool usb_is_initialized = false;

static dart_dev_t *usb_dart_init(u32 idx)
{
    int mapper_offset;
    char path[sizeof(FMT_DART_MAPPER_PATH)];

    snprintf(path, sizeof(path), FMT_DART_MAPPER_PATH, idx, idx);
    mapper_offset = adt_path_offset(adt, path);
    if (mapper_offset < 0) {
        // Device not present
        return NULL;
    }

    u32 dart_idx;
    if (ADT_GETPROP(adt, mapper_offset, "reg", &dart_idx) < 0) {
        printf("usb: Error getting DART %s device index/\n", path);
        return NULL;
    }

    snprintf(path, sizeof(path), FMT_DART_PATH, idx);
    return dart_init_adt(path, 1, dart_idx, false);
}

static int usb_drd_get_regs(u32 idx, struct usb_drd_regs *regs)
{
    int adt_drd_path[8];
    int adt_drd_offset;
    int adt_phy_path[8];
    int adt_phy_offset;
    char phy_path[sizeof(FMT_ATC_PATH)];
    char drd_path[sizeof(FMT_DRD_PATH)];

    snprintf(drd_path, sizeof(drd_path), FMT_DRD_PATH, idx);
    adt_drd_offset = adt_path_offset_trace(adt, drd_path, adt_drd_path);
    if (adt_drd_offset < 0) {
        // Nonexistent device
        return -1;
    }

    snprintf(phy_path, sizeof(phy_path), FMT_ATC_PATH, idx);
    adt_phy_offset = adt_path_offset_trace(adt, phy_path, adt_phy_path);
    if (adt_phy_offset < 0) {
        printf("usb: Error getting phy node %s\n", phy_path);
        return -1;
    }

    if (adt_get_reg(adt, adt_phy_path, "reg", 0, &regs->atc, NULL) < 0) {
        printf("usb: Error getting reg with index 0 for %s.\n", phy_path);
        return -1;
    }
    if (adt_get_reg(adt, adt_drd_path, "reg", 0, &regs->drd_regs, NULL) < 0) {
        printf("usb: Error getting reg with index 0 for %s.\n", drd_path);
        return -1;
    }
    if (adt_get_reg(adt, adt_drd_path, "reg", 3, &regs->drd_regs_unk3, NULL) < 0) {
        printf("usb: Error getting reg with index 3 for %s.\n", drd_path);
        return -1;
    }

    return 0;
}

int usb_phy_bringup(u32 idx)
{
    char path[24];

    if (idx >= USB_IODEV_COUNT)
        return -1;

    struct usb_drd_regs usb_regs;
    if (usb_drd_get_regs(idx, &usb_regs) < 0)
        return -1;

    snprintf(path, sizeof(path), FMT_ATC_PATH, idx);
    if (pmgr_adt_power_enable(path) < 0)
        return -1;

    snprintf(path, sizeof(path), FMT_DART_PATH, idx);
    if (pmgr_adt_power_enable(path) < 0)
        return -1;

    snprintf(path, sizeof(path), FMT_DRD_PATH, idx);
    if (pmgr_adt_power_enable(path) < 0)
        return -1;

    write32(usb_regs.atc + 0x08, 0x01c1000f);
    write32(usb_regs.atc + 0x04, 0x00000003);
    write32(usb_regs.atc + 0x04, 0x00000000);
    write32(usb_regs.atc + 0x1c, 0x008c0813);
    write32(usb_regs.atc + 0x00, 0x00000002);

    write32(usb_regs.drd_regs_unk3 + 0x0c, 0x00000002);
    write32(usb_regs.drd_regs_unk3 + 0x0c, 0x00000022);
    write32(usb_regs.drd_regs_unk3 + 0x1c, 0x00000021);
    write32(usb_regs.drd_regs_unk3 + 0x20, 0x00009332);

    return 0;
}

dwc3_dev_t *usb_iodev_bringup(u32 idx)
{
    dart_dev_t *usb_dart = usb_dart_init(idx);
    if (!usb_dart)
        return NULL;

    struct usb_drd_regs usb_reg;
    if (usb_drd_get_regs(idx, &usb_reg) < 0)
        return NULL;

    return usb_dwc3_init(usb_reg.drd_regs, usb_dart);
}

#define USB_IODEV_WRAPPER(name, pipe)                                                              \
    static ssize_t usb_##name##_can_read(void *dev)                                                \
    {                                                                                              \
        return usb_dwc3_can_read(dev, pipe);                                                       \
    }                                                                                              \
                                                                                                   \
    static bool usb_##name##_can_write(void *dev)                                                  \
    {                                                                                              \
        return usb_dwc3_can_write(dev, pipe);                                                      \
    }                                                                                              \
                                                                                                   \
    static ssize_t usb_##name##_read(void *dev, void *buf, size_t count)                           \
    {                                                                                              \
        return usb_dwc3_read(dev, pipe, buf, count);                                               \
    }                                                                                              \
                                                                                                   \
    static ssize_t usb_##name##_write(void *dev, const void *buf, size_t count)                    \
    {                                                                                              \
        return usb_dwc3_write(dev, pipe, buf, count);                                              \
    }                                                                                              \
                                                                                                   \
    static ssize_t usb_##name##_queue(void *dev, const void *buf, size_t count)                    \
    {                                                                                              \
        return usb_dwc3_queue(dev, pipe, buf, count);                                              \
    }                                                                                              \
                                                                                                   \
    static void usb_##name##_handle_events(void *dev)                                              \
    {                                                                                              \
        usb_dwc3_handle_events(dev);                                                               \
    }                                                                                              \
                                                                                                   \
    static void usb_##name##_flush(void *dev)                                                      \
    {                                                                                              \
        usb_dwc3_flush(dev, pipe);                                                                 \
    }

#define USB_IODEV_DWC2_WRAPPER(name, pipe)                                                         \
    static ssize_t usb_##name##_can_read(void *dev)                                                \
    {                                                                                              \
        return usb_dwc2_can_read(dev, pipe);                                                       \
    }                                                                                              \
                                                                                                   \
    static bool usb_##name##_can_write(void *dev)                                                  \
    {                                                                                              \
        return usb_dwc2_can_write(dev, pipe);                                                      \
    }                                                                                              \
                                                                                                   \
    static ssize_t usb_##name##_read(void *dev, void *buf, size_t count)                           \
    {                                                                                              \
        return usb_dwc2_read(dev, pipe, buf, count);                                               \
    }                                                                                              \
                                                                                                   \
    static ssize_t usb_##name##_write(void *dev, const void *buf, size_t count)                    \
    {                                                                                              \
        return usb_dwc2_write(dev, pipe, buf, count);                                              \
    }                                                                                              \
                                                                                                   \
    static ssize_t usb_##name##_queue(void *dev, const void *buf, size_t count)                    \
    {                                                                                              \
        return usb_dwc2_queue(dev, pipe, buf, count);                                              \
    }                                                                                              \
                                                                                                   \
    static void usb_##name##_handle_events(void *dev)                                              \
    {                                                                                              \
        usb_dwc2_handle_events(dev);                                                               \
    }                                                                                              \
                                                                                                   \
    static void usb_##name##_flush(void *dev)                                                      \
    {                                                                                              \
        usb_dwc2_flush(dev, pipe);                                                                 \
    }

USB_IODEV_DWC2_WRAPPER(0, CDC_ACM_PIPE_0)
USB_IODEV_WRAPPER(1, CDC_ACM_PIPE_1)

static struct iodev_ops iodev_usb_ops = {
    .can_read = usb_0_can_read,
    .can_write = usb_0_can_write,
    .read = usb_0_read,
    .write = usb_0_write,
    .queue = usb_0_queue,
    .flush = usb_0_flush,
    .handle_events = usb_0_handle_events,
};

static struct iodev_ops iodev_usb_sec_ops = {
    .can_read = usb_1_can_read,
    .can_write = usb_1_can_write,
    .read = usb_1_read,
    .write = usb_1_write,
    .queue = usb_1_queue,
    .flush = usb_1_flush,
    .handle_events = usb_1_handle_events,
};

struct iodev iodev_usb_vuart = {
    .ops = &iodev_usb_sec_ops,
    .usage = 0,
    .lock = SPINLOCK_INIT,
};

static tps6598x_dev_t *hpm_init(i2c_dev_t *i2c, const char *hpm_path)
{
    tps6598x_dev_t *tps = tps6598x_init(hpm_path, i2c);
    if (!tps) {
        printf("usb: tps6598x_init failed for %s.\n", hpm_path);
        return NULL;
    }

    if (tps6598x_powerup(tps) < 0) {
        printf("usb: tps6598x_powerup failed for %s.\n", hpm_path);
        tps6598x_shutdown(tps);
        return NULL;
    }

    return tps;
}

void usb_spmi_init(void)
{
    for (int idx = 0; idx < USB_IODEV_COUNT; ++idx)
        usb_phy_bringup(idx); /* Fails on missing devices, just continue */

    usb_is_initialized = true;
}

int usb_complex_init(void)
{
    // bring_up, we do have one usb port
    int otgphyctrl_path[8], usbComplex_path[8];
    u64 USBComplexBase, USBComplex_CtrlRegBase = 0, DWC2Base;

    int otgctl_offset = adt_path_offset_trace(adt, "/arm-io/otgphyctrl", otgphyctrl_path);

    uint64_t reg[4];
    if (adt_getprop_copy(adt, otgctl_offset, "reg", &reg, 0x20) < 0) {
        printf("usb: Error getting USBComplex_CtrlRegBase Reg\n");
        return -1;
    }

    for (uint32_t i = 0, max = 2; i < max; ++i) {
        if (reg[2 * i + 1] == 0x20) {
            USBComplex_CtrlRegBase = reg[2 * i];
            break;
        }
    }

    if (adt_path_offset_trace(adt, "/arm-io/usb-complex", usbComplex_path) < 0) {
        if (chip_id != S5L8960X) {
            printf("usb: failed to get /arm-io/usbcomplex node\n");
            return -1;
        }
        // iOS 7 does not have this in ADT
        USBComplexBase = 0x20c900000;

    } else if (adt_get_reg(adt, usbComplex_path, "reg", 0, &USBComplexBase, NULL) < 0) {
        printf("usb: Error getting USBComplexBase Reg\n");
        return -1;
    }

    USBComplex_CtrlRegBase += 0x200000000;

    // Can't trust ADT /arm-io/usb-complex/usb-device , it can be usb3 on some devices
    DWC2Base = (USBComplex_CtrlRegBase & ~0xfffULL) + 0x00100000;

    // Reset all of these because we have no idea what the previous stage is and what it did
    // USB complex
    pmgr_power_on(0, "USB");

    // USB complex OTG control registers
    pmgr_power_on(0, "USBCTLREG");
    pmgr_power_on(0, "USBCTRL");

    // DWC2 device
    pmgr_power_on(0, "USBOTG");
    pmgr_power_on(0, "USBDEV");
    pmgr_power_on(0, "USB2DEV");

    pmgr_reset(0, "USB");

    pmgr_reset(0, "USBCTLREG");
    pmgr_reset(0, "USBCTRL");

    pmgr_reset(0, "USBOTG");
    pmgr_reset(0, "USBDEV");
    pmgr_reset(0, "USB2DEV");

    // u32 otg_irq = 243;
    // DMA TBD
    // disable interrupt. set irq TBD
    // bringup now
    u32 cfg0, cfg1;
    if (ADT_GETPROP(adt, otgctl_offset, "cfg0-device", &cfg0) < 0) {
        printf("usb: Error getting CFG0 from otgctl \n");
        return -1;
    }
    if (ADT_GETPROP(adt, otgctl_offset, "cfg1-device", &cfg1) < 0) {
        printf("usb: Error getting CFG1 from otgctl \n");
        return -1;
    }
    switch (chip_id) {
        case 0x8011:
            write32(USBComplexBase, 1);
            write32(USBComplexBase + 0x24, 0x3000088);
            break;
        case 0x8015:
            write32(USBComplexBase, 1);
            write32(USBComplexBase + 0x48, 0x3000088);
            break;
        default:
            write32(USBComplexBase + 0x1c, 0x108);
            write32(USBComplexBase + 0x5c, 0x108);
            break;
    }
    // end
    write32(USBComplex_CtrlRegBase + USB_COMPLEX_CTRLREG_CFG0, cfg0);
    write32(USBComplex_CtrlRegBase + USB_COMPLEX_CTRLREG_CFG1, cfg1);
    set32(USBComplex_CtrlRegBase, 1);
    udelay(20);
    clear32(USBComplex_CtrlRegBase, 0xc);
    udelay(20);
    clear32(USBComplex_CtrlRegBase, 0x1);
    udelay(20);
    clear32(USBComplex_CtrlRegBase + 0x4, 0x2);
    udelay(1500);

    dwc2_dev_t *opaque;
    struct iodev *usb_iodev;

    opaque = usb_dwc2_init(DWC2Base);
    if (!opaque)
        return -1;

    usb_iodev = memalign(SPINLOCK_ALIGN, sizeof(*usb_iodev));
    if (!usb_iodev)
        return -1;
    set32(USBComplex_CtrlRegBase + 0x4, 2);
    usb_iodev->ops = &iodev_usb_ops;
    usb_iodev->opaque = opaque;
    usb_iodev->usage = USAGE_CONSOLE | USAGE_UARTPROXY;
    spin_init(&usb_iodev->lock);

    iodev_register_device(IODEV_USB0, usb_iodev);
    printf("USB/iDevice: initialized at %p\n", opaque);

    return 0;
}

void usb_init(void)
{
    char hpm_path[sizeof(FMT_HPM_PATH)];

    if (usb_is_initialized)
        return;

    /*
     * M3 models do not use i2c, but instead SPMI with a new controller.
     * We can get USB going for now by just bringing up the phys.
     */
    if (adt_path_offset(adt, "/arm-io/nub-spmi-a0/hpm0") > 0) {
        usb_spmi_init();
        return;
    }

    /*
     * A7-A11 uses a custom internal otg controller with the peripheral part
     * being dwc2.
     */
    if (adt_path_offset(adt, "/arm-io/otgphyctrl") > 0 &&
        adt_path_offset(adt, "/arm-io/usb-complex") > 0) {
        usb_complex_init();
        return;
    }

    i2c_dev_t *i2c = i2c_init("/arm-io/i2c0");
    if (!i2c) {
        printf("usb: i2c init failed.\n");
        return;
    }

    for (u32 idx = 0; idx < USB_IODEV_COUNT; ++idx) {
        snprintf(hpm_path, sizeof(hpm_path), FMT_HPM_PATH, idx);
        if (adt_path_offset(adt, hpm_path) < 0)
            continue; // device not present
        tps6598x_dev_t *tps = hpm_init(i2c, hpm_path);
        if (!tps) {
            printf("usb: failed to init hpm%d\n", idx);
            continue;
        }

        if (tps6598x_disable_irqs(tps, &tps6598x_irq_state[idx]))
            printf("usb: unable to disable IRQ masks for hpm%d\n", idx);

        tps6598x_shutdown(tps);
    }

    i2c_shutdown(i2c);

    for (int idx = 0; idx < USB_IODEV_COUNT; ++idx)
        usb_phy_bringup(idx); /* Fails on missing devices, just continue */

    usb_is_initialized = true;
}

void usb_hpm_restore_irqs(bool force)
{
    char hpm_path[sizeof(FMT_HPM_PATH)];

    i2c_dev_t *i2c = i2c_init("/arm-io/i2c0");
    if (!i2c) {
        printf("usb: i2c init failed.\n");
        return;
    }

    for (u32 idx = 0; idx < USB_IODEV_COUNT; ++idx) {
        if (iodev_get_usage(IODEV_USB0 + idx) && !force)
            continue;

        if (tps6598x_irq_state[idx].valid) {
            snprintf(hpm_path, sizeof(hpm_path), FMT_HPM_PATH, idx);
            if (adt_path_offset(adt, hpm_path) < 0)
                continue; // device not present
            tps6598x_dev_t *tps = hpm_init(i2c, hpm_path);
            if (!tps)
                continue;

            if (tps6598x_restore_irqs(tps, &tps6598x_irq_state[idx]))
                printf("usb: unable to restore IRQ masks for hpm%d\n", idx);

            tps6598x_shutdown(tps);
        }
    }

    i2c_shutdown(i2c);
}

void usb_iodev_init(void)
{
    if (adt_path_offset(adt, "/arm-io/otgphyctrl") > 0 &&
        adt_path_offset(adt, "/arm-io/usb-complex") > 0) {
        return; // already init in usb_init() since we do have only 1 usb port
    }
    for (int i = 0; i < USB_IODEV_COUNT; i++) {
        dwc3_dev_t *opaque;
        struct iodev *usb_iodev;

        opaque = usb_iodev_bringup(i);
        if (!opaque)
            continue;

        usb_iodev = memalign(SPINLOCK_ALIGN, sizeof(*usb_iodev));
        if (!usb_iodev)
            continue;

        usb_iodev->ops = &iodev_usb_ops;
        usb_iodev->opaque = opaque;
        usb_iodev->usage = USAGE_CONSOLE | USAGE_UARTPROXY;
        spin_init(&usb_iodev->lock);

        iodev_register_device(IODEV_USB0 + i, usb_iodev);
        printf("USB%d: initialized at %p\n", i, opaque);
    }
}

void usb_iodev_shutdown(void)
{
    for (int i = 0; i < USB_IODEV_COUNT; i++) {
        struct iodev *usb_iodev = iodev_unregister_device(IODEV_USB0 + i);
        if (!usb_iodev)
            continue;

        printf("USB%d: shutdown\n", i);
        if (adt_path_offset(adt, "/arm-io/otgphyctrl") > 0 &&
            adt_path_offset(adt, "/arm-io/usb-complex") > 0) {
            usb_dwc2_shutdown(usb_iodev->opaque);
            return;
        } else {
            usb_dwc3_shutdown(usb_iodev->opaque);
        }
        free(usb_iodev);
    }
}

void usb_iodev_vuart_setup(iodev_id_t iodev)
{
    if (iodev < IODEV_USB0 || iodev >= IODEV_USB0 + USB_IODEV_COUNT)
        return;

    iodev_usb_vuart.opaque = iodev_get_opaque(iodev);
}
