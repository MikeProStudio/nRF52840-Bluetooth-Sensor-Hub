# Implementation Plan: XIAO BLE Sense USB & Boot Fix

## Objective
Restore USB CDC (Serial) functionality and ensure reliable booting while maintaining BLE Beacon features.

## Step 1: Create `app.overlay`
**Reason:** The default board configuration for `xiao_ble` likely routes the console to the physical UART pins (P0.06/P0.07), not the USB CDC ACM interface. We must explicitly reroute it.

**Action:** Create `app.overlay` in the project root.
```dts
/ {
	chosen {
		zephyr,console = &cdc_acm_uart0;
		zephyr,shell-uart = &cdc_acm_uart0;
	};
};

&zephyr_udc0 {
	cdc_acm_uart0: cdc_acm_uart0 {
		compatible = "zephyr,cdc-acm-uart";
	};
};
```

## Step 2: Optimize `prj.conf`
**Reason:** Ensure the USB stack and UART console are correctly configured to work together, and enable necessary debugging aids.

**Action:** Update `prj.conf`.
*   **Add/Verify:**
    *   `CONFIG_USB_DEVICE_STACK=y`
    *   `CONFIG_USB_DEVICE_PRODUCT="XIAO BLE Beacon"`
    *   `CONFIG_UART_CONSOLE=y`
    *   `CONFIG_CONSOLE=y`
    *   `CONFIG_UART_LINE_CTRL=y` (Critical for DTR detection)
    *   `CONFIG_LOG=y`
    *   `CONFIG_LOG_MODE_DEFERRED=y` (Prevents ISR blocking)
    *   `CONFIG_LOG_PROCESS_THREAD_STARTUP_DELAY_MS=1000` (Wait for USB init)

## Step 3: Refactor `src/main.c`
**Reason:** The current `main.c` does not wait for the USB connection to be established before printing or proceeding. This causes early boot logs to be lost and can lead to race conditions if the host hasn't enumerated the device yet.

**Action:**
1.  **Include Headers:** Ensure `<zephyr/drivers/uart.h>` and `<zephyr/usb/usb_device.h>` are present.
2.  **Add Wait for DTR:** Implement a loop that waits for the Data Terminal Ready (DTR) signal from the host (PC) before continuing. This ensures the console is open on the host side.
3.  **Heartbeat LED:** Initialize the onboard LED (Red/Blue/Green) and blink it. This provides immediate visual feedback that the kernel has booted and the main loop is running, distinguishing a "silent failure" from a "no USB" failure.
4.  **Error Handling:** Check return codes for `usb_enable` and `bt_enable`. Flash an error pattern on the LED if they fail.

## Step 4: Verify CMakeLists.txt
**Reason:** Ensure the project structure supports the overlay and new configuration.

**Action:** No major changes expected, but verify `target_sources` includes the new `main.c`.

## Execution Order
1.  Create `app.overlay`
2.  Update `prj.conf`
3.  Refactor `src/main.c`
4.  Build and Verify (User to perform)
