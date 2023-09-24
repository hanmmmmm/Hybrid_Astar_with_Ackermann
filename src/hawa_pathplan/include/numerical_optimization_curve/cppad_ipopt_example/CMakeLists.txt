
project(cppad_example_try)

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

SET(source_list main.cpp)

SET(cppad_lib /usr/local/lib/libcppad_lib.so)

SET(ipopt /usr/include )

ADD_EXECUTABLE(example_get_started  ${source_list})

MESSAGE(source_list="${source_list}")
MESSAGE(cppad_lib="${cppad_lib}")
MESSAGE(ipopt="${ipopt}")
MESSAGE(colpack_libs="${colpack_libs}")


TARGET_LINK_LIBRARIES(example_get_started ${cppad_lib} ${colpack_libs} ipopt )
