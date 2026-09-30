# Source attribution

Thermal160 telemetry layout, temperature mapping and reset/baud switching are
adapted from Sipeed Ltd's MaixPy project:

https://github.com/sipeed/MaixPy/blob/a36c0f0a85ba8b1a4f7eee7ab74af13afcd89bb0/projects/app_thermal160_camera/main.py

Copyright Sipeed Ltd. Licensed under the Apache License, Version 2.0.
The upstream license is included as LICENSE_SIPEED.txt. The application icon
assets/thermal.json is copied without modification from the same upstream project.

Local changes: bounded parser with two-frame synchronization, finite UART reads,
latest-frame thread mailbox and reconnect, raw-pixel projected ROI thermometry,
YOLO/visible-light composition, touch alignment and a TCP RTSP/RTP-JPEG server.
The pinned upstream reference is available at the GitHub URL above; this
repository contains the license and the adapted code, not the full snapshot.

RTSP/RTP-JPEG implementation is locally authored against RFC 2326 and RFC 2435.
No firmware, model weights or external media server is bundled.
