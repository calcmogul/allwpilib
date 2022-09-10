cmake_minimum_required(VERSION 3.3.0)

project(googletest-download NONE)

include(ExternalProject)
ExternalProject_Add(googletest
  GIT_REPOSITORY    https://github.com/google/googletest.git
  GIT_TAG           e2239ee6043f73722e7aa812a459f54a28552929 # 1.11.0
  SOURCE_DIR        "/Users/joshuanichols/CLionProjects/allwpilib/cmake-build-debug/googletest/googletest-src"
  BINARY_DIR        "/Users/joshuanichols/CLionProjects/allwpilib/cmake-build-debug/googletest/googletest-build"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
)
