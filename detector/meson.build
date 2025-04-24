sources = files(
    'src/armor.cpp',
)
headers = files(
    'include/armor.hpp'
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