if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()
find_package(Boost CONFIG REQUIRED COMPONENTS
        filesystem
)
# See ocs2_core/cmake/ocs2_cxx_flags.cmake for the rationale: Boost::system
# became header-only in 1.69 and was dropped as a separately packaged component
# in Boost 1.90 (Ubuntu 26). Probe so consumers can fall back to Boost::headers.
find_package(Boost CONFIG QUIET COMPONENTS system)