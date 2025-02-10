project(
    'AutoAim',
    'cpp',
    version: '0.1',
    default_options: [
        'cpp_std=c++20',
        'prefix=/home/arca/Documents/codebases/prediction/output',
    ],
)

### Dependency declaration
spdlog = dependency('spdlog')
opencv = dependency('opencv4')
eigen = dependency('eigen3')
# tomlpp = dependency('tomlplusplus')

### Directories
binary_dir = './output'
public_headers = include_directories('include')

### Structs
subdir('structs')
subdir('config')

# subdir('include')
subdir('detector')
# subdir('test')