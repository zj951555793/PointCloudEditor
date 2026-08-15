# Production dual-camera scan integration

## Thread ownership

- UI thread: buttons, source selector, camera list, exposure widgets, preview rendering only.
- `ScanSource` QThread: source orchestration, virtual file reads, camera discovery, synchronized-pair delivery.
- Camera A std::thread: owns and calls only Camera A `cv::VideoCapture`.
- Camera B std::thread: owns and calls only Camera B `cv::VideoCapture`.
- `RulerMvsPipeline` QThread: rulermvs decode, RGBDFusion submission/stop, offline optimize/fuse.

No `VideoCapture::open/read/grab/retrieve/set`, rulermvs decode, Fusion stop, or offline optimize is executed by the UI thread.

## A/B synchronization

Each camera has a continuously running capture thread and a bounded queue. The host timestamp is recorded immediately after successful `VideoCapture::grab()`. The synchronizer compares the oldest A/B frames:

- `abs(timestampA - timestampB) <= cameraSyncToleranceMs`: pair accepted and sent to rulermvs.
- Otherwise, only the older frame is discarded and the newer frame is retained for comparison with the next frame.
- Each camera queue is bounded (default 3), so SLAM backpressure cannot cause unbounded camera memory growth.

This is **host-side software synchronization**. OpenCV does not expose a portable UVC exposure-start hardware timestamp and cannot guarantee that two USB cameras started exposure at the same instant. If the product requires exposure-level synchronization, use camera hardware trigger / shared trigger wiring / vendor SDK timestamps. The same `ScanSourceWorker -> RawScanFrame` boundary can keep the rest of the pipeline unchanged.

## Camera A/B roles

- Camera A -> structured-light code image; it is converted to grayscale (`RawScanFrame::code`).
- Camera B -> RGB/color image (`RawScanFrame::rgb`) and is also used for the top-right live camera preview.
- The raw camera dimensions must match the dimensions in `calib.txt`. Frames with a different size are rejected instead of being resized, because resizing before structured-light decoding invalidates calibration.

## Camera VID/PID model JSON

Runtime default path:

`<exe>/config/camera_models.json`

Example:

```json
{
  "version": 1,
  "cameras": [
    {
      "vid": "1234",
      "pid": "5678",
      "model": "JM Camera 1200P",
      "width": 1920,
      "height": 1200,
      "fps": 10,
      "fourcc": "MJPG",
      "exposure": {
        "min": -13,
        "max": 0,
        "step": 1,
        "default": -6,
        "manualAutoValue": 0.25
      }
    }
  ]
}
```

On Windows, cameras are enumerated through DirectShow. VID/PID is parsed from the DirectShow device path, then matched against the JSON. The selected device path is persisted rather than only the OpenCV integer index. At scan start, devices are enumerated again and the current OpenCV/DirectShow index is resolved.

## Exposure

The UI exposure spin boxes are usable during camera scanning. A UI change only updates an atomic requested value; the owning camera capture thread performs `CAP_PROP_AUTO_EXPOSURE` and `CAP_PROP_EXPOSURE`, avoiding concurrent access to `cv::VideoCapture` from another thread.


## Stop semantics

Stopping no longer waits for the controller-side `inflight_` callback counter to become zero.
After input acceptance is disabled, `RGBDFusion::stop()` runs on `RulerMvsPipeline` and is treated as the SDK queue-drain boundary.
This avoids a permanent `Stopping` state when an accepted input does not produce a trace callback. The UI thread never calls the blocking SDK stop.

## Vocabulary default

If the UI setting is empty, the first `*.yml.gz` file in the executable directory is selected automatically.
