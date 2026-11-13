# Native controller regressions

Build the actual controller throttle/diagnostic implementations and inline
monitoring classes on the host, without ESP-IDF, PlatformIO or Arduino installed:

```sh
cmake -S tools/host-native-tests -B build-host-native-tests
cmake --build build-host-native-tests --parallel
ctest --test-dir build-host-native-tests --output-on-failure
```

GoogleTest v1.15.2 is fetched on first configuration, or set `GTEST_DIR` to an
existing source tree for an offline build. The host tests vendor cJSON from
ESP-IDF v5.5.3 (`test/vendor/cjson`, MIT license) so diagnostic JSON executes the
same serializer/parser used by the firmware. `test/native_stubs/platform` mocks
only hardware/time/console boundaries; the code under test is production code.

The six executables contain 63 tests covering throttle response/filter/cruise,
monitor thresholds/hysteresis and BMS validity helpers, logger fan-out,
diagnostic history/reset classification/JSON round trips, and wireless packet
CRC/range/sequence behavior. All test executables propagate failed assertions as
nonzero exit codes; CI must use CTest's result.

`PROJECT_ROOT` can point to another controller source checkout while keeping
`HOST_TEST_ROOT` at this checkout, which supports before/after migration
comparisons. It defaults to the current repository.
