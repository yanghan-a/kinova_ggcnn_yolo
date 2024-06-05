import pyrealsense2 as rs  # Intel RealSense cross-platform open-source API 英特尔实感跨平台开源API

depth_to_disparity = rs.disparity_transform(True)
disparity_to_depth = rs.disparity_transform(False)

# 创建抽取过滤器
decimation = rs.decimation_filter()
# # 您可以通过滤波器幅度选项来控制抽取量（线性比例因子）。
# # 注意不断变化的图像分辨率
decimation.set_option(rs.option.filter_magnitude, 4)

# [2、空间过滤器]
# Spatial Filter
# Spatial Filter is a fast implementation of Domain-Transform Edge Preserving Smoothing
# 空间滤波器是域转换边缘保留平滑的快速实现
spatial = rs.spatial_filter()
# We can emphesize the effect of the filter by cranking-up smooth_alpha and smooth_delta options:
# 我们可以通过增加smooth_alpha和smooth_delta选项来强调滤镜的效果：
spatial.set_option(rs.option.filter_magnitude, 5)
spatial.set_option(rs.option.filter_smooth_alpha, 1)
spatial.set_option(rs.option.filter_smooth_delta, 50)
# The filter also offers some basic spatial hole filling capabilities:
# 该过滤器还提供一些基本的空间孔填充功能：
spatial.set_option(rs.option.holes_fill, 3)

# Next, we need to "feed" the frames to the filter one by one:
# 接下来，我们需要将帧逐一“馈入”到过滤器：
temporal = rs.temporal_filter()

# 孔填充过滤器提供了附加的深度外推层：
hole_filling = rs.hole_filling_filter()

def image_processing(frame):
    # frame = decimation.process(frame)
    frame = depth_to_disparity.process(frame)
    frame = spatial.process(frame)
    frame = temporal.process(frame)
    frame = disparity_to_depth.process(frame)
    frame = hole_filling.process(frame)
    return frame