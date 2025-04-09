camera = library(
    'camera',
    'camera.cpp',
    include_directories: [
        include_directories('./'),
    ],
    dependencies: all_dep,
)

camera_dep = declare_dependency(
    include_directories: include_directories('./'),
    link_with: camera,
)

all_dep += camera_dep