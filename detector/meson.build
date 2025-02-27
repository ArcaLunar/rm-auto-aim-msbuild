detector_include = include_directories('./include')

headers = files(
    'include/classifier.hpp',
    'include/detector.hpp',
    # 'include/publisher.hpp',
)
sources = files(
    'src/armor.cpp',
    'src/classifier.cpp',
    'src/detector.cpp',
)

detector_lib = static_library(
    'detector',
    headers + sources,
    include_directories: [
        detector_include,
    ],
    dependencies: [
        utils_dep,
        all_dep,
        pnp_solver_dep,
    ],
)

detector_dep = declare_dependency(
    include_directories: detector_include,
    link_with: detector_lib,
)

all_dep += detector_dep