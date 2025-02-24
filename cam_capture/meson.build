cam_capture = static_library(
    'cam_capture',
    'cam_capture.cpp',
    include_directories: [
        include_directories('./'),
        headers,
    ],
    dependencies: [
        all_dep,
        utils_dep,
    ],
    install: true,
    # install_dir: join_paths(get_option('prefix'), 'bin'),
)

cam_capture_dep = declare_dependency(
    include_directories: [
        include_directories('./'),
        headers,
    ],
    link_with: cam_capture,
)