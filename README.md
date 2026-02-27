#文件介绍
rtk_move_pos.py 未使用zed相机，纯rtk进行导航，使用rtk的航向角以及坐标
combined_navigation.py 结合了zed相机，rtk进行初次定位，后续使用zed相机算航向角
zed_rtk_move.py 结合了zed相机的imu和rtk进行在无fix精度时的位置定位

#文件使用

#环境启动
cd /home/nvidia/unitree_sdk2_python/example/b2/high_level
source ~/anaconda3/bin/activate
conda activate unitree

#启动程序
python xx.py eno1 

#注意
实际使用时根据串口实际名称调整串口
需要等待rtk进入fix状态
