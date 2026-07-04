# PEAK PCAN-USB SocketCAN Setup on Jetson AGX Orin

## Overview

This guide explains how to build and install the missing PEAK PCAN-USB SocketCAN driver on a Jetson AGX Orin.

The issue found was that Jetson Linux had:

```text
CONFIG_CAN_PEAK_USB=m
```

but the actual module:

```text
peak_usb.ko
```

was not installed.

---

## Tested Environment

| Item | Value |
|---|---|
| Platform | NVIDIA Jetson AGX Orin |
| Jetson Linux | 36.4.7 |
| Ubuntu | 22.04 |
| Kernel | 5.15.148-tegra |
| GCC | 11.4 |
| CAN Adapter | PEAK PCAN-USB |
| USB ID | `0c72:000c` |
| Working Interface | `can2` |

---

## 1. Check the Kernel and Driver State

Check the running kernel:

```bash
uname -r
```

Expected:

```text
5.15.148-tegra
```

Check whether the PEAK USB driver is available:

```bash
modinfo peak_usb
```

If it is missing, you may see:

```text
modinfo: ERROR: Module peak_usb not found
```

Check whether the kernel config expects the driver as a module:

```bash
grep CONFIG_CAN_PEAK_USB /lib/modules/$(uname -r)/build/.config
```

Expected:

```text
CONFIG_CAN_PEAK_USB=m
```

Check whether the module file exists:

```bash
find /lib/modules/$(uname -r) -name 'peak_usb.ko*'
```

If nothing is returned, the module needs to be built.

---

## 2. Install Build Tools

```bash
sudo apt update
sudo apt install -y linux-source-5.15.0 build-essential bc flex bison libssl-dev libelf-dev dwarves
```

---

## 3. Extract the Linux 5.15 Source

Check that the source tarball exists:

```bash
ls -lh /usr/src/linux-source-5.15.0*
```

Extract it:

```bash
mkdir -p ~/linux-source-extract
tar -xf /usr/src/linux-source-5.15.0.tar.bz2 -C ~/linux-source-extract
```

Find the PEAK USB driver source files:

```bash
find ~/linux-source-extract -path '*/drivers/net/can/usb/peak_usb/*' -type f | sort
```

You should see files such as:

```text
pcan_usb.c
pcan_usb_core.c
pcan_usb_core.h
pcan_usb_fd.c
pcan_usb_pro.c
```

---

## 4. Create an External Module Build Folder

```bash
mkdir -p ~/peak_usb_external
cd ~/peak_usb_external
```

Copy the PEAK USB driver source files:

```bash
cp ~/linux-source-extract/linux-source-5.15.0/drivers/net/can/usb/peak_usb/*.[ch] .
```

Create a `Makefile`:

```bash
cat > Makefile <<'EOF'
obj-m += peak_usb.o
peak_usb-y := pcan_usb_core.o pcan_usb.o pcan_usb_pro.o pcan_usb_fd.o
EOF
```

---

## 5. Build the Driver Against the Jetson Kernel Headers

```bash
make -C /lib/modules/$(uname -r)/build M=$PWD modules
```

If successful, this should create:

```text
peak_usb.ko
```

Check it:

```bash
ls -lh peak_usb.ko
```

Check that it was built for the correct kernel:

```bash
modinfo ./peak_usb.ko | grep vermagic
```

Expected output should include:

```text
5.15.148-tegra
```

---

## 6. Install the Driver

Create the module directory:

```bash
sudo mkdir -p /lib/modules/$(uname -r)/kernel/drivers/net/can/usb/peak_usb
```

Copy the module:

```bash
sudo cp peak_usb.ko /lib/modules/$(uname -r)/kernel/drivers/net/can/usb/peak_usb/
```

Update the module database:

```bash
sudo depmod -a
```

Load the driver:

```bash
sudo modprobe peak_usb
```

Verify installation:

```bash
modinfo peak_usb
```

Expected output should include:

```text
description: CAN driver for PEAK-System USB adapters
depends: can-dev
vermagic: 5.15.148-tegra
```

---

## 7. Plug In the PEAK PCAN-USB Adapter

Check that the adapter is detected:

```bash
lsusb | grep -i -E 'peak|0c72'
```

Expected output:

```text
ID 0c72:000c PEAK System PCAN-USB
```

Check available CAN interfaces:

```bash
ip link show type can
```

Identify which interface is the PEAK adapter:

```bash
for i in can0 can1 can2; do
  echo "=== $i ==="
  ip -details link show "$i"
  ethtool -i "$i" 2>/dev/null || true
done
```

On the tested system:

| Interface | Driver |
|---|---|
| `can0` | `mttcan` |
| `can1` | `mttcan` |
| `can2` | `peak_usb` |

So the PEAK adapter was:

```text
can2
```

---

## 8. Bring Up the CAN Interface

For a 500 kbit/s CAN bus:

```bash
sudo ip link set can2 down 2>/dev/null
sudo ip link set can2 up type can bitrate 500000
```

Check the interface:

```bash
ip -details link show can2
```

---

## 9. Test with can-utils

In one terminal, listen for CAN frames:

```bash
candump can2
```

In another terminal, send a CAN frame:

```bash
cansend can2 123#11223344
```

If another CAN node is connected at the same bitrate, frames should appear in `candump`.

---

## 10. Optional Loopback Test

If there is no physical CAN bus connected, loopback mode can be used for a local test:

```bash
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 500000 loopback on
sudo ip link set can2 up
```

Then run:

```bash
candump can2
```

In another terminal:

```bash
cansend can2 123#11223344
```

Expected output:

```text
can2  123   [4]  11 22 33 44
```

---

## 11. Shut Down Safely

Stop `candump` with:

```text
Ctrl+C
```

Bring the interface down:

```bash
sudo ip link set can2 down
```

It is then safe to unplug the PEAK USB adapter.

---

## Troubleshooting

### `modprobe peak_usb` says module not found

Check whether the module exists:

```bash
find /lib/modules/$(uname -r) -name 'peak_usb.ko*'
```

If nothing is returned, the module has not been installed.

### `CONFIG_CAN_PEAK_USB=m` exists but the module is missing

This means the kernel was configured to build the driver as a module, but the compiled module was not shipped.

### The PEAK adapter is not `can2`

The interface number can change. Always check the driver:

```bash
ethtool -i can2
```

The PEAK adapter should show:

```text
driver: peak_usb
```

If not, check `can0`, `can1`, `can3`, etc.

### `dmesg` gives permission denied

Use:

```bash
sudo dmesg | grep -i -E 'peak|pcan|can'
```

---

## Notes

The Jetson AGX Orin also has internal CAN interfaces using the `mttcan` driver. These may appear as `can0` and `can1`.

The PEAK PCAN-USB adapter appears as a SocketCAN network interface using the `peak_usb` driver.