# 文件组织：

* raybot_charging:json配置
* charge:自动充电算法逻辑代码

# 状态机说明：

charge/charge.cpp、charge/charge.hpp代码中状态机各个状态含义如下：

* kStateReqCharge: 请求充电
* kStateGetDistance: 测量底盘与充电桩的距离
* kStateSearchPile: 寻找并对准充电桩
* kStateHandshake: 握手（判断是否对准）
* kStateCharging: 充电中
* kStateFinishCharge: 完成充电
* kStateReSearch: 重新对准

# 系统修改

清除串口缓存:
isaac/engine/gems/coms/serial.*
