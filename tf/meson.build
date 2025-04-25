sources = files(
    'src/pose_convert.cpp',
    'src/transform.cpp',
)

inc_dir = include_directories('./include')
libposecvt = library(
    'posecvt',
    sources,
    include_directories: inc_dir,
    dependencies: all_dep,
)

depposecvt = declare_dependency(
    include_directories: inc_dir,
    link_with: libposecvt,
)

all_dep += depposecvt