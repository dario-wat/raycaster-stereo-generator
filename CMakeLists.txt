project (MyProject)
cmake_minimum_required(VERSION 2.8)

find_package(PythonLibs 2.7 REQUIRED)
include_directories (${PYTHON_INCLUDE_DIRS})

find_package(Boost 1.54.0 COMPONENTS python REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})

#PYTHON_ADD_MODULE(teststuff geometry.cpp)

add_library (
    teststuff SHARED  
    geometry.cpp
)

target_link_libraries (teststuff
    boost_python
    ${PYTHON_LIBRARIES}
    ${Boost_LIBRARIES}
)