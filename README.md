#文件介绍

robotdog_ws_client.py 启动websocket服务，接收前端消息启动程序

rtk_move_pos.py 未使用zed相机，纯rtk进行导航，使用rtk的航向角以及坐标

rtk_zed_heading.py 结合了zed相机，rtk进行初次定位，后续使用zed相机算航向角

zed_rtk_move.py 结合了zed相机的imu和rtk进行在无fix精度时的位置定位

rtk.py 结合前端控制程序

config.json ip以及websocket的端口的配置文件

#注意

实际使用时根据串口实际名称调整串口

需要等待rtk进入fix状态

