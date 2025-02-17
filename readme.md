## `detector` 模块

包含 `detector` 和 `classifier`

## `transform`

### `pnp_solver`

将 `cv::Mat` 上的二维点，投射到三维。

假设甲板长宽为 $w\times h$，那么甲板的四个角点投射到的三维坐标是（从左上开始顺时针）

$$
(0,-w/2,h/2)\\
(0, w/2, h/2)\\
(0,w/2,-h/2)\\
(0,-w/2,- h/2)\\
$$
