# Copyright (C) 2014 ARM Limited. All rights reserved.  

# the actual library we export
add_library(lesense
    lesense.mm
)

# no proper CMake support for objective-c flags :(
set_target_properties(lesense PROPERTIES
    COMPILE_FLAGS "${CMAKE_OBJC_FLAGS}"
)

target_link_libraries(lesense
    objc2  # must always be first!
    emlib
    yottos
    nsassert
    yottos_platform
    yterror
)

