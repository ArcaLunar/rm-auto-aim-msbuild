cam_capture = shared_library(
    'cam_capture',
    'cam_capture.cpp',
    dependencies: [
        all_dep,
        config_dep,
    ],
    install: true,
    install_dir: join_paths(get_option('prefix'), 'bin'),
)

cam_capture_dep = declare_dependency(
    include_directories: [
        include_directories('./'),
    ],
    link_with: cam_capture,
)