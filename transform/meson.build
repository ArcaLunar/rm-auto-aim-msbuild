pnp_solver_lib = library(
    'pnp_solver',
    'src/pnp_solver.cpp',
    include_directories: include_directories('./include'),
    dependencies: [
        all_dep,
    ],
)

pnp_solver_dep = declare_dependency(
    include_directories: include_directories('./include'),
    link_with: pnp_solver_lib,
)

all_dep += pnp_solver_dep

transform_lib = library(
    'transform',
    'src/transform.cpp',
    include_directories: [
        include_directories('./include'),
    ],
    dependencies: [
        all_dep,
    ],
)
transform_dep = declare_dependency(
    include_directories: include_directories('./include'),
    link_with: transform_lib,
)

all_dep += [
    transform_dep,
]

pose_cvt_lib = library(
    'pose_cvt',
    'src/pose_convert.cpp',
    include_directories: include_directories('./include'),
    dependencies: [
        all_dep,
    ],
)
pose_cvt_dep = declare_dependency(
    include_directories: include_directories('./include'),
    link_with: pose_cvt_lib,
)

all_dep += [
    pose_cvt_dep,
]