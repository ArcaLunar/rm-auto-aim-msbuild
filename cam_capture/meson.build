cam_capture = shared_library(
    'cam_capture',
    'cam_capture.cpp',
    dependencies: [
        opencv,
        spdlog,
        mvs_dep,
        config_dep,
    ],
    install: true,
)

cam_capture_dep = declare_dependency(
    include_directories: [
        include_directories('./'),
    ],
    link_with: cam_capture,
)