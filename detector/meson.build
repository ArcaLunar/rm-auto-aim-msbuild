detector_include = include_directories('./include')

detector_lib = shared_library(
    'detector',
    'src/detector.cpp',
    include_directories: [
        detector_include,
    ],
    dependencies: [
        structs_dep,
        config_dep,
    ],
)