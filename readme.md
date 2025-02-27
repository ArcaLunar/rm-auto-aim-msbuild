# Progress

- [x] Camera Capture
  - [x] Open camera
  - [x] Grab images and tag time stamp
- [x] Serial port
  - [x] Connection
  - [x] Read and send
  - [ ] Reconection
- [ ] Detector
  - [x] Data flow
  - [ ] Accuracy
- [ ] Policy
  - [ ] armor selection
- [ ] Transform
  - [ ] Cordinate Transform
  - [ ] Solve PNP
- [ ] Tracker
  - [x] EKF

# Overview

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

## `detector.publisher`

订阅 `detector` 发送的（二维）装甲板信息，通过 `transform` 模块转化为三维装甲板，然后发送出去。

## `policy`

通过 `work_queue` 订阅 `detector.publisher` 发送的（三维、已标注）装甲板信息

1. 直接将甲板转发到各自的 `tracker`
2. 筛选要击打的甲板，释放对应的 `mutex`（允许攻击）

## `tracker`

通过 `work_queue` 订阅 `policy` 发送的装甲板信息，对车进行建模