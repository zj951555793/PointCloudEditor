package com.jmengine.sdk;

import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.Build;

import java.util.ArrayList;
import java.util.List;

public final class UsbCameraDevices {
    public static final String ACTION_USB_PERMISSION = "com.jmengine.sdk.USB_PERMISSION";
    public static final int REALTEK_VENDOR_ID = 0x0BDA;
    public static final int CAMERA_A_PRODUCT_ID = 0x300A;
    public static final int CAMERA_B_PRODUCT_ID = 0x300B;

    private UsbCameraDevices() {}

    public static String describe(Context context) {
        try {
            UsbManager manager = usbManager(context);
            if (manager == null) {
                return "USB Host unavailable: UsbManager is null";
            }

            List<String> lines = new ArrayList<>();
            for (UsbDevice device : manager.getDeviceList().values()) {
                StringBuilder line = new StringBuilder();
                line.append(String.format("VID=%04X PID=%04X", device.getVendorId(), device.getProductId()));
                line.append(" class=").append(device.getDeviceClass());
                line.append(" ifaces=").append(device.getInterfaceCount());
                line.append(" permission=").append(safeHasPermission(manager, device));
                if (isVideoDevice(device)) {
                    line.append(" UVC");
                }
                lines.add(line.toString());
            }
            if (lines.isEmpty()) {
                return "No USB device detected. Check OTG/host mode, power, cable and hub.";
            }
            return String.join("\n", lines);
        } catch (Throwable t) {
            return "USB describe failed: " + t.getClass().getSimpleName() + ": " + t.getMessage();
        }
    }

    public static int requestVideoPermissions(Context context) {
        try {
            UsbManager manager = usbManager(context);
            if (manager == null) {
                return 0;
            }

            int requested = 0;
            int flags = PendingIntent.FLAG_UPDATE_CURRENT;
            if (Build.VERSION.SDK_INT >= 31) {
                flags |= PendingIntent.FLAG_MUTABLE;
            }
            PendingIntent intent = PendingIntent.getBroadcast(
                    context.getApplicationContext(), 0, new Intent(ACTION_USB_PERMISSION), flags);
            for (UsbDevice device : manager.getDeviceList().values()) {
                if (isTargetCamera(device) && !safeHasPermission(manager, device)) {
                    try {
                        manager.requestPermission(device, intent);
                        ++requested;
                    } catch (Throwable ignored) {
                    }
                }
            }
            return requested;
        } catch (Throwable ignored) {
            return 0;
        }
    }

    public static int targetCameraCount(Context context) {
        try {
            UsbManager manager = usbManager(context);
            if (manager == null) {
                return 0;
            }
            boolean hasA = false;
            boolean hasB = false;
            for (UsbDevice device : manager.getDeviceList().values()) {
                if (!isTargetCamera(device)) {
                    continue;
                }
                if (device.getProductId() == CAMERA_A_PRODUCT_ID) {
                    hasA = true;
                } else if (device.getProductId() == CAMERA_B_PRODUCT_ID) {
                    hasB = true;
                }
            }
            return (hasA ? 1 : 0) + (hasB ? 1 : 0);
        } catch (Throwable ignored) {
            return 0;
        }
    }

    public static String targetSummary(Context context) {
        try {
            UsbManager manager = usbManager(context);
            if (manager == null) {
                return "USB Host unavailable: UsbManager is null";
            }
            boolean hasA = false;
            boolean hasB = false;
            boolean permA = false;
            boolean permB = false;
            for (UsbDevice device : manager.getDeviceList().values()) {
                if (!isTargetCamera(device)) {
                    continue;
                }
                if (device.getProductId() == CAMERA_A_PRODUCT_ID) {
                    hasA = true;
                    permA = safeHasPermission(manager, device);
                } else if (device.getProductId() == CAMERA_B_PRODUCT_ID) {
                    hasB = true;
                    permB = safeHasPermission(manager, device);
                }
            }
            return "cameraA detected=" + hasA + " permission=" + permA
                    + "\ncameraB detected=" + hasB + " permission=" + permB;
        } catch (Throwable t) {
            return "USB target summary failed: " + t.getClass().getSimpleName() + ": " + t.getMessage();
        }
    }

    private static UsbManager usbManager(Context context) {
        return (UsbManager) context.getApplicationContext().getSystemService(Context.USB_SERVICE);
    }

    public static boolean hasBothTargetCameraPermissions(Context context) {
        try {
            UsbManager manager = usbManager(context);
            if (manager == null) {
                return false;
            }
            boolean hasA = false;
            boolean hasB = false;
            for (UsbDevice device : manager.getDeviceList().values()) {
                if (!isTargetCamera(device) || !safeHasPermission(manager, device)) {
                    continue;
                }
                if (device.getProductId() == CAMERA_A_PRODUCT_ID) {
                    hasA = true;
                } else if (device.getProductId() == CAMERA_B_PRODUCT_ID) {
                    hasB = true;
                }
            }
            return hasA && hasB;
        } catch (Throwable ignored) {
            return false;
        }
    }

    private static boolean isTargetCamera(UsbDevice device) {
        return device != null
                && device.getVendorId() == REALTEK_VENDOR_ID
                && (device.getProductId() == CAMERA_A_PRODUCT_ID || device.getProductId() == CAMERA_B_PRODUCT_ID)
                && isVideoDevice(device);
    }

    private static boolean isVideoDevice(UsbDevice device) {
        try {
            if (device.getDeviceClass() == 14) {
                return true;
            }
            for (int i = 0; i < device.getInterfaceCount(); ++i) {
                UsbInterface iface = device.getInterface(i);
                if (iface != null && iface.getInterfaceClass() == 14) {
                    return true;
                }
            }
        } catch (Throwable ignored) {
        }
        return false;
    }

    private static boolean safeHasPermission(UsbManager manager, UsbDevice device) {
        try {
            return manager != null && device != null && manager.hasPermission(device);
        } catch (Throwable ignored) {
            return false;
        }
    }
}
