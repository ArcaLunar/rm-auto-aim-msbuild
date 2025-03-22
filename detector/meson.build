detector_include = include_directories('./include')

headers = files(
    'include/classifier.hpp',
    'include/detector.hpp',
    'include/publisher.hpp',
)
sources = files(
    'src/armor.cpp',
    'src/classifier.cpp',
    'src/detector.cpp',
    'src/publisher.cpp',
)

detector_lib = library(
    'detector',
    headers + sources,
    include_directories: [
        detector_include,
    ],
    dependencies: [
        all_dep,
    ],
)

detector_dep = declare_dependency(
    include_directories: detector_include,
    link_with: detector_lib,
)

all_dep += detector_dep