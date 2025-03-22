policy_lib = static_library(
    'policy',
    'policy.cpp',
    include_directories: include_directories('.'),
    dependencies: [all_dep],
)

policy_dep = declare_dependency(
    include_directories: include_directories('.'),
    link_with: policy_lib,
)

all_dep += policy_dep