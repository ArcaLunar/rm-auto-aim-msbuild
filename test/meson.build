gdebug = executable('test_debug', 'gdebug.cpp', dependencies: all_dep)
tcamera = executable('test_camera', 'test_camera.cpp', dependencies: all_dep)
tport = executable('test_port', 'test_port.cpp', dependencies: all_dep)
tportcam = executable('test_portcam', 'test_portcam.cpp', dependencies: all_dep)
tlac = executable('test_lac', 'test_lightbar_armor_config.cpp', dependencies: all_dep)

# generate tests
test('test_debug', gdebug)
test('test_camera', tcamera)
test('test_port', tport)
test('test_portcam', tportcam)
test('test_lac', tlac)