# 识别节点输出：当前帧所有亮靶的关键点
std_msgs/Header header

# 按面积从大到小排列（大的通常是真靶，小的多半是箭头灯效/杂光）
Blade[] blades

# 传统视觉：机关圆心（原图坐标）和扇叶到圆心的像素半径。
# rune_valid=false
geometry_msgs/Point rune_center
float32 rune_radius_px
bool rune_valid
