detector_include = include_directories('./include')

headers = files(
    'include/armor.hpp',
    'include/detector.hpp',
    'include/classifier.hpp',
)
sources = files(
    'src/armor.cpp',
    'src/detector.cpp',
    'src/classifier.cpp',
)

detector_lib = shared_library(
    'detector',
    headers + sources,
    include_directories: [
        detector_include,
    ],
    dependencies: [
        structs_dep,
        config_dep,
        all_dep,
    ],
)

detector_dep = declare_dependency(
    include_directories: detector_include,
    link_with: detector_lib,
)