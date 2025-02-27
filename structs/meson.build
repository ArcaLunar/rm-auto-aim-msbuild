structs_declaration_include = include_directories('.')

utils_dep = declare_dependency(
    include_directories: [
        structs_declaration_include,
    ],
    dependencies: [
        opencv,
    ],
)

all_dep += utils_dep