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

#! set tests
test('read_config', read_config)
test('init_cam', init_cam)