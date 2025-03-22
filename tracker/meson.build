ekf_lib = library(
    'ekf',
    'filters/ekf.cpp',
    include_directories: [
        include_directories('./filters'),
    ],
    dependencies: [
        all_dep,
    ],
)
ekf_dep = declare_dependency(
    include_directories: ['./filters'],
    link_with: ekf_lib,
)

tracker_lib = library(
    'tracker',
    'tracker.cpp',
    include_directories: [
        include_directories('./'),
    ],
    dependencies: [
        all_dep,
        ekf_dep,
        utils_dep,
    ],
)
tracker_dep = declare_dependency(
    include_directories: ['./'],
    link_with: tracker_lib,
)

all_dep += [tracker_dep, ekf_dep]