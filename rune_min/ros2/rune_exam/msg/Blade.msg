# 一块亮起的靶面（能量机关扇叶末端的发光靶环）
uint8 id                    # 本帧内按面积排序的序号（不稳定，仅调试用；跟踪号见 TargetInfo.id）
float32 confidence          # 0~1

geometry_msgs/Point center  # 靶面中心，图像坐标 (x=u, y=v)
float32 radius_px           # 靶面半径（椭圆半长轴，像素）
float32 area_px             # 红色区域面积（像素²）

# 拟合椭圆的完整描述（PnP 用）。
# angle_rad 已归一化成**长轴**方向。
float32 semi_minor_px
float32 angle_rad

# 靶面内部被红色围住的暗区个数（原始掩码）。
# 未击打的靶面是几圈同心环 -> 多个暗区；击打后变实心 -> 1 个或 0 个。
int32 hole_count

# 5 个关键点，图像坐标，顺序固定：
#   [0]=中心  [1]=上  [2]=右  [3]=下  [4]=左
geometry_msgs/Point[] keypoints
