// Copyright <Zach Whitehead>
#pragma once

#define VERSION_MAJOR 8
#define VERSION_MINOR 0

// Monotonic build counter, independent of MAJOR/MINOR. Injected at build time
// by extra_script.py (git commit count) so it climbs on every release. The
// fallback is only used for IDE/intellisense or builds outside a git checkout.
#ifndef VERSION_BUILD
#define VERSION_BUILD 0
#endif

// Define a version string
#define VERSION_STRING STRINGIFY(VERSION_MAJOR) "." STRINGIFY(VERSION_MINOR)

// Helper macro for stringification
#define STRINGIFY(x) STRINGIFY_HELPER(x)
#define STRINGIFY_HELPER(x) #x
