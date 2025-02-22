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
        structs_dep,
        work_queue_dep,
    ],
)

wq_test = executable(
    'wq_test',
    'wq_test.cpp',
    dependencies: [
        all_dep,
        structs_dep,
        work_queue_dep,
    ],
)

#! set tests
test('read_config', read_config)
test('init_cam', init_cam)
test('cbuffer_test', cbuffer_test)
test('wq_test', wq_test)