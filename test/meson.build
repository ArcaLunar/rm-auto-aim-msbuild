gdebug = executable('test_debug', 'gdebug.cpp', dependencies: all_dep)
tcamera = executable('test_camera', 'test_camera.cpp', dependencies: all_dep)
tport = executable('test_port', 'test_port.cpp', dependencies: all_dep)

# generate tests
test('test_debug', gdebug)
test('test_camera', tcamera)
test('test_port', tport)