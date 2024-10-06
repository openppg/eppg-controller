// Copyright <Zach Whitehead>
#pragma once

#ifdef CAN_PIO
    #define VERSION_MAJOR 7
    #define VERSION_MINOR 0
#else
    #define VERSION_MAJOR 6
    #define VERSION_MINOR 5
#endif


// Define a version string
#define VERSION_STRING STRINGIFY(VERSION_MAJOR) "." STRINGIFY(VERSION_MINOR)

// Helper macro for stringification
#define STRINGIFY(x) STRINGIFY_HELPER(x)
#define STRINGIFY_HELPER(x) #x
