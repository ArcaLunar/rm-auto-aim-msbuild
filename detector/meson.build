sources = files(
    'src/armor_config.cpp',
    'src/lightbar_config.cpp',
    'src/armor.cpp',
    'src/lightbar.cpp',
    'src/classifier.cpp',
)

include_headers = include_directories('./include')

libdetector = library(
    'detector',
    sources,
    include_directories: [
        include_headers,
    ],
    dependencies: all_dep,
)

depdetector = declare_dependency(
    include_directories: include_headers,
    link_with: libdetector,
)

all_dep += depdetector