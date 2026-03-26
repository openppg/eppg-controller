// Copyright <Zach Whitehead>
#pragma once

#define VERSION_MAJOR 8
#define VERSION_MINOR 1

#ifndef VERSION_BUILD
#define VERSION_BUILD 0
#endif

// Define a version string
#define VERSION_STRING STRINGIFY(VERSION_MAJOR) "." STRINGIFY(VERSION_MINOR)

// Helper macro for stringification
#define STRINGIFY(x) STRINGIFY_HELPER(x)
#define STRINGIFY_HELPER(x) #x
