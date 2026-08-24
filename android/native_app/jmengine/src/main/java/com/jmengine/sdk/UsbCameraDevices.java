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
            line.append(" permission=").append(manager.hasPermission(device));
            if (isVideoDevice(device)) {
                line.append(" UVC");
            }
            lines.add(line.toString());
        }
        if (lines.isEmpty()) {
            return "No USB device detected. Check OTG/host mode, power, cable and hub.";
        }
        return String.join("\n", lines);
    }

    public static int requestVideoPermissions(Context context) {
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
                context, 0, new Intent(ACTION_USB_PERMISSION), flags);
        for (UsbDevice device : manager.getDeviceList().values()) {
            if (isTargetCamera(device) && !manager.hasPermission(device)) {
                manager.requestPermission(device, intent);
                ++requested;
            }
        }
        return requested;
    }

    public static int targetCameraCount(Context context) {
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
    }

    public static String targetSummary(Context context) {
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
                permA = manager.hasPermission(device);
            } else if (device.getProductId() == CAMERA_B_PRODUCT_ID) {
                hasB = true;
                permB = manager.hasPermission(device);
            }
        }
        return "cameraA detected=" + hasA + " permission=" + permA
                + "\ncameraB detected=" + hasB + " permission=" + permB;
    }

    private static UsbManager usbManager(Context context) {
        return (UsbManager) context.getApplicationContext().getSystemService(Context.USB_SERVICE);
    }

    public static boolean hasBothTargetCameraPermissions(Context context) {
        UsbManager manager = usbManager(context);
        if (manager == null) {
            return false;
        }
        boolean hasA = false;
        boolean hasB = false;
        for (UsbDevice device : manager.getDeviceList().values()) {
            if (!isTargetCamera(device) || !manager.hasPermission(device)) {
                continue;
            }
            if (device.getProductId() == CAMERA_A_PRODUCT_ID) {
                hasA = true;
            } else if (device.getProductId() == CAMERA_B_PRODUCT_ID) {
                hasB = true;
            }
        }
        return hasA && hasB;
    }

    private static boolean isTargetCamera(UsbDevice device) {
        return device != null
                && device.getVendorId() == REALTEK_VENDOR_ID
                && (device.getProductId() == CAMERA_A_PRODUCT_ID || device.getProductId() == CAMERA_B_PRODUCT_ID)
                && isVideoDevice(device);
    }

    private static boolean isVideoDevice(UsbDevice device) {
        if (device.getDeviceClass() == 14) {
            return true;
        }
        for (int i = 0; i < device.getInterfaceCount(); ++i) {
            UsbInterface iface = device.getInterface(i);
            if (iface.getInterfaceClass() == 14) {
                return true;
            }
        }
        return false;
    }
}
