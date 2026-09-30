# 融合应用推流接口核查（2026-09-17）

核查 MaixCDK main，提交 `30f4b8b7e3f66ded9cfa64fb081b46e30bc27248`：

- [MaixCAM2 RTSP 实现](https://github.com/sipeed/MaixCDK/blob/30f4b8b7e3f66ded9cfa64fb081b46e30bc27248/components/vision/port/maixcam2/maix_rtsp_maixcam2.cpp)：start必须绑定相机，write(video::Frame&) 抛 ERR_NOT_IMPL。
- [MaixCAM2 WebRTC 实现](https://github.com/sipeed/MaixCDK/blob/30f4b8b7e3f66ded9cfa64fb081b46e30bc27248/components/vision/port/maixcam2/maix_webrtc_maixcam2.cpp)：同样必须绑定相机，write未实现，add_region返回NULL。
- API网页列出接口，并不代表此平台对应实现可用。

因此本地新应用使用纯Python RTSP服务，把已经画好热像、检测框、温度和状态的RGB画面经OpenCV编码为baseline JPEG，按[RFC 2435](https://www.rfc-editor.org/rfc/rfc2435)封装RTP/JPEG，并通过RTSP TCP interleaved发送。控制协议参考[RFC 2326](https://www.rfc-editor.org/rfc/rfc2326)。这是真正的RTSP/JPEG视频流，不是把HTTP MJPEG改名为RTSP，也不是WebRTC/H.264。

优点：不调用未实现的SDK入口、不需要ffmpeg/MediaMTX服务端、不需要板上pip额外安装；屏幕和流使用相同合成画面。
边界：仅TCP传输、无音频、无认证、局域网使用；JPEG带宽高于H.264，默认限8fps、质量70、640×480。VLC/ffplay应显式使用RTSP over TCP；浏览器不能直接播放RTSP。
