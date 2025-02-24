init_cam = executable(
    'init_cam',
    'init_cam.cpp',
    dependencies: [
        all_dep,
        cam_capture_dep,
    ],
)

read_config = executable(
    'read_config',
    'read_config.cpp',
    dependencies: [
        all_dep,
    ],
)

cbuffer_test = executable(
    'cbuffer_test',
    'cbuffer_test.cpp',
    dependencies: [
        all_dep,
        utils_dep,
        work_queue_dep,
    ],
)

wq_test = executable(
    'wq_test',
    'wq_test.cpp',
    dependencies: [
        all_dep,
        utils_dep,
        work_queue_dep,
    ],
)

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

df_img_test = executable(
    'dataflow_img',
    'dataflow_cam2image_test.cpp',
    dependencies: [
        all_dep,
        utils_dep,
        work_queue_dep,
        cam_capture_dep,
        serial_port_dep,
    ],
)
serial_port_test = executable(
    'serialport_test',
    'serial_port_test.cpp',
    dependencies: [
        all_dep,
        serial_port_dep,
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