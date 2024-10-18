本项目在windows系统上运行 webots+matlab的四足机器人MPC控制 13.5kg模型 


只用到QPOASES库 其他均为显式代码 供初学者入门使用 控制实时性暂时难以保证


视频：
https://www.bilibili.com/video/BV13ntQebEPy/?spm_id_from=333.999.0.0&vd_source=ee1fee6807dcef3c3a7f9010f242334c


1.首先确保你的电脑安装了webots和matlab 作者版本为webots R2021b和matlab2023a  尽量选择同版本或新版本 新版本可能会需要自行调整坐标系


2.建立matlab与webots的通讯：
需要给matlab安装 MATLAB MinGW-w64 C/C++ Compiler


地址：https://ww2.mathworks.cn/matlabcentral/fileexchange/52848-matlab-support-for-mingw-w64-c-c-compiler


安装后


配置windows环境变量


据自己的matlab和webots的安装路径相应的添加到系统环境变量中。我的设置如下所示（来自网络）：


第一个
![image](https://github.com/user-attachments/assets/7c745909-ef65-4cb9-81a1-082750c2b3a9)


第二个
![image](https://github.com/user-attachments/assets/215d55ae-8c07-4a03-8e79-b25165425d18)


第三个
![image](https://github.com/user-attachments/assets/934c61ac-9f33-4923-965e-f0f10a900e31)


只要将webots和matlab建立通讯 后面基本没有问题  大家可以在网上多找些帖子配置下webots和matlab通讯


3.运行


下载压缩包，解压缩


打开webots 文件->打开世界->找到解压缩文件夹Quadruped_MPC_matlab\worlds->打开empty.wbt 或者直接双击文件夹的empty.wbt 若成功建立webots和matlab通讯


则点击webots运行仿真或ctrl+2即可开始仿真，算法代码在controllers\my_BIGDOG4_MPC下 用matlab可以查看修改


原始URDF:Quadruped_MPC_matlab\protos\robot-Bigdog-A.SLDASM\urdf\robot-Bigdog-A.SLDASM.urdf


4.介绍


控制频率500hz  MPC频率33hz 


仿真运行5s后 鼠标点击下webots界面即可开始键盘控制

W S A D：前后左右


T：加速冲刺


U：切换到 Trotting 步态


I：切换到 Bounding 步态


O: 切换到 Pronking 步态


P: 切换到 Galloping 步态


J: 切换到 Standing 步态


K: 切换到 Trot Running 步态


L: 切换到 Walking 步态


Z X:变化滚转角


C V:变化俯仰角


B N:变化航向角

有时间会更新更多代码和功能。。。



