debug = library(
    'debug_opt',
    'debug_options.cpp',
    include_directories: include_directories('.'),
    dependencies: all_dep,
)

debug_dep = declare_dependency(
    include_directories: include_directories('.'),
    link_with: debug,
)

all_dep += debug_dep