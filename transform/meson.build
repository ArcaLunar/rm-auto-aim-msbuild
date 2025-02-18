pnp_solver_lib = static_library(
    'pnp_solver',
    'pnp_solver.cpp',
    include_directories: include_directories('.'),
    dependencies: [
        all_dep,
        structs_dep,
    ],
)

pnp_solver_dep = declare_dependency(
    include_directories: include_directories('.'),
    link_with: pnp_solver_lib,
)