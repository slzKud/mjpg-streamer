mjpg-streamer output plugin: output_vnc
==========================================

这是一个简易的将MJPEG视频流转换为VNC服务器的插件，基于NeatVNC。在output_viewer基础上改的。

仅在input_file下测试通过，后续待继续更改。

Usage
=====

    mjpg_streamer [input plugin options] -o 'output_vnc.so'