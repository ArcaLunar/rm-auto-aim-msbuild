cam_capture = library(
    'cam_capture',
    'cam_capture.cpp',
    include_directories: [
        include_directories('./'),
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
    ],
    link_with: cam_capture,
)

all_dep += cam_capture_dep