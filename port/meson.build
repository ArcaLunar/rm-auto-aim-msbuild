port_lib = library(
    'port',
    'port.cpp',
    include_directories: include_directories('.'),
    dependencies: all_dep,
)

port_dep = declare_dependency(
    include_directories: include_directories('.'),
    link_with: port_lib,
)

all_dep += port_dep