firing_inc = include_directories('.')

firing_lib = static_library(
    'firing',
    'firing.cpp',
    include_directories: [
        firing_inc,
    ],
    dependencies: [
        all_dep,
    ],
)

firing_dep = declare_dependency(
    include_directories: firing_inc,
    link_with: firing_lib,
)

all_dep += firing_dep