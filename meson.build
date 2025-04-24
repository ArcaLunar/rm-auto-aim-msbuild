project(
    'AutoAim',
    'cpp',
    version: '0.1',
    default_options: [
        'cpp_std=c++20',
        'cpp_args=-w',
        'default_library=static',
    ],
)

##

### Dependency declaration
spdlog = dependency('spdlog')
opencv = dependency('opencv4')
eigen = dependency('eigen3')
toml = subproject('tomlplusplus', default_options: ['default_library=static']).get_variable('tomlplusplus_dep')
boost = dependency('boost')

mvs = subproject('mvs').get_variable('mvs_dep')

all_dep = [
    spdlog,
    opencv,
    eigen,
    mvs,
    boost,
    toml,
]

subdir('debug')
subdir('structs')

subdir('camera')
subdir('port')
subdir('detector')

subdir('app')
subdir('test')