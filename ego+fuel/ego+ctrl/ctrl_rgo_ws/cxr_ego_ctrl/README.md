本代码必须要在飞控能够进入position模式才行，也就是飞控必须要收到外部的里程计信息才行，所以这里需要进行vins fusion或者lio的里程计对PX4飞控进行融合。融合的代码在这里：https://gitee.com/Canada-a/vins_to_mavros

#### 使用说明
- 方法一
1.  cd到主目录
2.  `git clone https://gitee.com/Canada-a/cxr_ego_ctrl.git`
3.  `cd cxr_ego_ctrl`
4.  `catkin_make`
- 方法二
1. 可以直接把里面的控制cpp单独复制到egoplanner里的plan_manage下的src里面，然后增加cmakelist编译内容直接编译就行
编译完成后去cxr_egoctrl_v1.cpp的代码里面看用法就行，都写到注释里了。

### 