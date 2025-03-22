project(
    'AutoAim',
    'cpp',
    version: '0.1',
    default_options: [
        'cpp_std=c++20',
        'cpp_args=-w',
        'default_library=shared',
    ],
)

##
headers = [include_directories('./structs')]

### Dependency declaration
spdlog = dependency('spdlog')
opencv = dependency('opencv4')
eigen = dependency('eigen3')
toml = subproject('tomlplusplus', default_options: ['default_library=static']).get_variable('tomlplusplus_dep')
boost = dependency(
    'boost',
    modules: [
        'system',
    ],
)

mvs = subproject('mvs')
mvs_dep = mvs.get_variable('mvs_dep')
# tf = subproject('tf3d')
# tf_dep = tf.get_variable('tf_dep')

all_dep = [
    spdlog,
    opencv,
    eigen,
    mvs_dep,
    boost,
    toml,
    # tf_dep,
]

### Directories
binary_dir = './output'

# INSTALL Configs
install_subdir(
    'config',
    install_dir: get_option('prefix'),
)

### Structs
subdir('structs')
subdir('work_queue')
subdir('detector')
subdir('cam_capture')
subdir('serialport')
subdir('transform')
subdir('tracker')
subdir('policy')
subdir('firing')

subdir('test')
subdir('app')