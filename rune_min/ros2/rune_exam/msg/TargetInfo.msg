# 解算节点输出：单个目标的位置与距离
std_msgs/Header header

bool valid
# 解算节点分配的**跟踪 id**（不是检测器的瞬时序号）。换目标时会变，
# 云台据此清掉上一帧的速度状态，避免把换靶当成目标高速运动。
uint16 id

# 是否请求开火。只在选中的靶面被判定为**待击打**（hole_count >= pending_min_holes）时为 true。
# 例外：所有目标都被判成“已激活”（状态量不可靠）时也会置 true，否则会出现永远不开火的死局。
# 云台节点还要等瞄准收敛才会真正打出去。
bool fire

# 图像坐标系下的靶子坐标（像素，原点左上）
geometry_msgs/Point pixel

# 相机光心到靶面的**欧氏距离**（米）：|position_cam|
float32 distance_m

# 光轴方向的深度 Z（米）。distance_m = |position_cam| >= depth_m。
# 比例法直接算出来的是 depth，别把两者混用。
float32 depth_m

# 相机坐标系下的靶心（x 右、y 下、z 前），单位米。
# 后续做弹道/云台时直接用这个。
geometry_msgs/Point position_cam
