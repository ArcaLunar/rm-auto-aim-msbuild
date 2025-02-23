project(
    'tf3d',
    'cpp',
)

srcs = files(
    'src/pcdtool.cpp',
    'src/trans_forms.cpp',
    'src/trans_forms_group.cpp',
)

eigen = dependency('eigen3')

tf_lib = static_library(
    'tf3d',
    srcs,
    include_directories: include_directories('include'),
    dependencies: [
        eigen,
    ],
)

tf_dep = declare_dependency(
    include_directories: include_directories('include'),
    link_with: tf_lib,
)