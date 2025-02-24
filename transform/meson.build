pnp_solver_lib = static_library(
    'pnp_solver',
    'pnp_solver.cpp',
    include_directories: include_directories('.'),
    dependencies: [
        all_dep,
        utils_dep,
    ],
)

pnp_solver_dep = declare_dependency(
    include_directories: include_directories('.'),
    link_with: pnp_solver_lib,
)

transform_lib = static_library(
    'transform',
    'transform.cpp',
    include_directories: [
        include_directories('.'),
        headers,
    ],
    dependencies: [
        all_dep,
        pnp_solver_dep,
        utils_dep,
    ],
)
transform_dep = declare_dependency(
    include_directories: include_directories('.'),
    link_with: transform_lib,
)