project(
    'AutoAim',
    'cpp',
    version: '0.1',
    default_options: [
        'cpp_std=c++20',
        'prefix=/home/arca/Documents/codebases/pred_v2/output',
        'cpp_args=-w',
    ],
)

##
headers = include_directories('./structs')

### Dependency declaration
spdlog = dependency('spdlog')
opencv = dependency('opencv4')
eigen = dependency('eigen3')
toml = dependency('tomlplusplus')

mvs = subproject('mvs')
mvs_dep = mvs.get_variable('mvs_dep')

all_dep = [
    spdlog,
    opencv,
    eigen,
    mvs_dep,
    toml,
]

### Directories
binary_dir = './output'

# INSTALL Configs
# install_subdir(
#     'config',
#     install_dir: get_option('prefix'),
# )

### Structs
subdir('structs')
subdir('config')
# subdir('work_queue')
subdir('cam_capture')
subdir('serialport')
subdir('transform')

subdir('detector')

subdir('test')
subdir('app')