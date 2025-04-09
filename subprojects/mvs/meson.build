project(
    'mvs',
    'cpp',
)

cxx = meson.get_compiler('cpp')
mvs_lib_path = '/opt/MVS/lib/64'

mvs_dep = declare_dependency(
    include_directories: include_directories('include'),
    dependencies: [
        cxx.find_library('MvCameraControl', dirs: mvs_lib_path),
    ],
)

set_variable('mvs_dep', mvs_dep)