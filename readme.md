# Progress

- [x] Camera Capture
  - [x] Basic
    - <del>Open camera</del>
    - <del>Set configs</del>
  - [ ] (Buggy) Image Capture
    - <del>Pixel Format</del>
    - Grab Image
- [x] Serial port
  - [x] Connection
  - [x] Read
  - [x] Send
  - [ ] Reconection
- [x] Detector
  - [x] Basic
    - <del>Data flow tested</del>
    - Able to return 2D coordinate
  - [ ] Accuracy?
    - Traditional CV Method
- [x] Policy
  - [ ] Armor Selection
    - <del>Simple</del>
    - Complicated
- [ ] Transform
  - **[Warning] Untested**
  - **(Part of)**
  - [x] Util functions
  - [x] Cordinate Transform
  - [x] Solve PNP
- [ ] Tracker
  - **[Warning] Untested**
  - **(Part of)**
  - [x] KF
  - [x] **(Part of)** EKF
- [ ] Firing
  - **Untested**
  - [ ] Simple control
  - [ ] Serial Port Control

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