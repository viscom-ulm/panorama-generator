cmake_minimum_required(VERSION 3.2)
project(PanoramaGenerator)

find_package( OpenCV REQUIRED )
find_package(ImmersightTools REQUIRED HINTS extern/immersight-tools)

add_subdirectory("extern/immersight-tools")

include_directories("extern/immersight-tools")

add_executable(PanoramaGenerator src/main.cpp)

target_link_libraries( PanoramaGenerator ${OpenCV_LIBS} ImmersightTools)
target_include_directories(PanoramaGenerator PUBLIC ${ImmersightTools_INCLUDE_DIRS} extern/immersight-tools extern/immersight-tools/extern/glm)