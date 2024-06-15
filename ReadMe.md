本项目基于c++复现港科大沈发表于IROS-2023上的文章《Online Monocular Lane Mapping Using Catmull-Rom Spline》，第一作者是乔志健博士。
论文的代码是开源的，本仓库仅属于个人学习用途。
---
代码已初步实现，完全开源还需润色一些时间，知乎笔记链接：https://zhuanlan.zhihu.com/p/699785810  
代码依赖库：
gtsam,Eigen
1. 代码大致逻辑设计：
<div align=center><img width = '850' height ='500' src =./doc/umi.png></div> 
2. 控制点选取验证：
<div align=center><img width = '900' height ='300' src =./doc/ctrpts.png></div> 
3. 车道线关联验证：
<div align=center><img width = '850' height ='350' src =./doc/associa.png></div> 
4. 优化示意:  

![image](https://github.com/Shane1911/Monlanemapping_C_version/blob/main/doc/opt.gif)  


基于slam的方法对车道线的建图精度受限于传感器输入以及优化后过程中的不确定性误差，缺少高精地图校准的情况下，gtsam优化后的车道线控制点理论上为相邻几帧的最优估计值，个人感觉精度误差强依赖与上游感知结果，slam的方法能保证结果一定的鲁棒性。
