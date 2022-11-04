cmake_minimum_required(VERSION 3.24.0)
project(vimba_libs)

add_library(VimbaC SHARED IMPORTED)
SET_PROPERTY(TARGET VimbaC PROPERTY IMPORTED_LOCATION /opt/ros/melodic/lib/libVimbaC.so)

add_library(VimbaCPP SHARED
    ../include/VimbaCPP/Source/AncillaryData.cpp
    ../include/VimbaCPP/Source/BaseFeature.cpp
    ../include/VimbaCPP/Source/BasicLockable.cpp
    ../include/VimbaCPP/Source/BoolFeature.cpp
    ../include/VimbaCPP/Source/Camera.cpp
    ../include/VimbaCPP/Source/Clock.cpp
    ../include/VimbaCPP/Source/CommandFeature.cpp
    ../include/VimbaCPP/Source/Condition.cpp
    ../include/VimbaCPP/Source/ConditionHelper.cpp
    ../include/VimbaCPP/Source/DefaultCameraFactory.cpp
    ../include/VimbaCPP/Source/EnumEntry.cpp
    ../include/VimbaCPP/Source/EnumFeature.cpp
    ../include/VimbaCPP/Source/FeatureContainer.cpp
    ../include/VimbaCPP/Source/Feature.cpp
    ../include/VimbaCPP/Source/FileLogger.cpp
    ../include/VimbaCPP/Source/FloatFeature.cpp
    ../include/VimbaCPP/Source/Frame.cpp
    ../include/VimbaCPP/Source/FrameHandler.cpp
    ../include/VimbaCPP/Source/Interface.cpp
    ../include/VimbaCPP/Source/IntFeature.cpp
    ../include/VimbaCPP/Source/Mutex.cpp
    ../include/VimbaCPP/Source/MutexGuard.cpp
    ../include/VimbaCPP/Source/RawFeature.cpp
    ../include/VimbaCPP/Source/Semaphore.cpp
    ../include/VimbaCPP/Source/StringFeature.cpp
    ../include/VimbaCPP/Source/VimbaSystem.cpp
)

target_include_directories(VimbaCPP PUBLIC
    ../include
)

target_link_libraries(VimbaCPP
    VimbaC
)