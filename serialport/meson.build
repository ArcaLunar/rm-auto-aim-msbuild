serial_port_lib = static_library(
    'serial_port',
    ['serial_port.cpp', 'serial_port.hpp'],
    dependencies: [
        all_dep,
        structs_dep,
    ],
    install: true,
)

serial_port_dep = declare_dependency(
    include_directories: include_directories('.'),
    link_with: serial_port_lib,
)