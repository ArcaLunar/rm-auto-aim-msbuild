pnp_solver_lib = static_library(
    'pnp_solver',
    'src/pnp_solver.cpp',
    include_directories: include_directories('./include'),
    dependencies: [
        all_dep,
        utils_dep,
    ],
)

pnp_solver_dep = declare_dependency(
    include_directories: include_directories('./include'),
    link_with: pnp_solver_lib,
)

transform_lib = static_library(
    'transform',
    'src/transform.cpp',
    include_directories: [
        include_directories('./include'),
        headers,
    ],
    dependencies: [
        all_dep,
        pnp_solver_dep,
        utils_dep,
    ],
)
transform_dep = declare_dependency(
    include_directories: include_directories('./include'),
    link_with: transform_lib,
)

pose_cvt_lib = static_library(
    'pose_cvt',
    'src/pose_convert.cpp',
    include_directories: include_directories('./include'),
    dependencies: [
        all_dep,
        transform_dep,
        utils_dep,
    ],
)
pose_cvt_dep = declare_dependency(
    include_directories: include_directories('./include'),
    link_with: pose_cvt_lib,
)

all_dep += [
    transform_dep,
    pnp_solver_dep,
    pose_cvt_dep,
]