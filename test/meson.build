# 测试相机是否可以正常打开
init_cam = executable(
    'init_cam',
    'init_cam.cpp',
    dependencies: [
        all_dep,
        cam_capture_dep,
    ],
)

# 测试能否正常读取配置文件
read_config = executable(
    'read_config',
    'read_config.cpp',
    dependencies: [
        all_dep,
    ],
)

# 测试循环缓冲区是否正常工作
cbuffer_test = executable(
    'cbuffer_test',
    'cbuffer_test.cpp',
    dependencies: [
        all_dep,
        utils_dep,
        work_queue_dep,
    ],
)

# 测试 DataTransmitter 是否可以正常工作
wq_test = executable(
    'wq_test',
    'wq_test.cpp',
    dependencies: [
        all_dep,
        utils_dep,
        work_queue_dep,
    ],
)

# 测试 TransformManager
tf_graph = executable(
    'tf_graph',
    'tf_graph.cpp',
    dependencies: [
        all_dep,
        utils_dep,
        work_queue_dep,
        transform_dep,
    ],
)

# 测试 DataFlow (port + camera + DataTransmitter --> image/RawImageInfo)
df_img_test = executable(
    'dataflow_img',
    'dataflow_cam2image_test.cpp',
    dependencies: [
        all_dep,
        utils_dep,
        transform_dep,
        work_queue_dep,
        cam_capture_dep,
        serial_port_dep,
        detector_dep,
    ],
)

# 测试串口通信
serial_port_test = executable(
    'serialport_test',
    'serial_port_test.cpp',
    dependencies: [
        all_dep,
        serial_port_dep,
    ],
)

detector_test = executable(
    'detector_test',
    'detector_test.cpp',
    dependencies: [
        all_dep,
        utils_dep,
        detector_dep,
    ],
)

#! set tests
test('read_config', read_config)
test('init_cam', init_cam)
test('cbuffer_test', cbuffer_test)
test('wq_test', wq_test)
test('tf_graph', tf_graph)
test('dataflow_img', df_img_test)
test('sport_test', serial_port_test)
test('detector_test', detector_test)