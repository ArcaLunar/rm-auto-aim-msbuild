project(
    'AutoAim',
    'cpp',
    version: '0.1',
    default_options: [
        'cpp_std=c++20',
        'prefix=/home/arca/Documents/codebases/pred_v2/output',
    ],
)

### Dependency declaration
spdlog = dependency('spdlog')
opencv = dependency('opencv4')
eigen = dependency('eigen3')

mvs = subproject('mvs')
mvs_dep = mvs.get_variable('mvs_dep')

### Directories
binary_dir = './output'
# public_headers = include_directories('include')

### Structs
subdir('structs')
subdir('config')

subdir('cam_capture')

# subdir('include')
subdir('detector')
# subdir('test')

subdir('app')