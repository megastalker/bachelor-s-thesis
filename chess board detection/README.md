# Chess board detection

## Develop in VS Code

1. Open the repository folder (`bachelor's thesis`) in VS Code.
2. Install the recommended CMake Tools and Microsoft C/C++ extensions.
3. Run **CMake: Select Configure Preset** and select **local-dev**.
4. Run **CMake: Build**, then **CMake: Debug** or **CMake: Run Without Debugging**.
   Select `chess_board_detection` if prompted for a launch target.
5. Focus the OpenCV window and press a key to finish the program.

CMake Tools supplies IntelliSense and selects GDB from the configured compiler's
directory. Use its build/run commands so the executable matches the active preset.

`local-dev` uses RelWithDebInfo: application debug symbols with optimization and
OpenCV release libraries. Some variables or source steps may be optimized away.
`local-release` uses Release. Each preset has its own directory under `build/`.
The old executable directly under `build/` belongs to the previous configuration;
the new development executable is `build/local-dev/chess_board_detection.exe`.

## Command line

From this directory, in PowerShell:

```powershell
cmake --preset local-dev
cmake --build --preset local-dev --parallel
& ./build/local-dev/chess_board_detection.exe
```

For Release, substitute `local-release` in all three commands.

## Toolchain and dependencies

Requires CMake 3.21+, w64devkit MinGW, and OpenCV installed with vcpkg's
`x64-mingw-dynamic` triplet. The current installation uses GCC 14.2.0 and
OpenCV 4.12.0. OpenCV and the application must use compatible MinGW toolchains.

Machine paths live in the ignored `CMakeUserPresets.json`, already configured for
this computer. On another computer, copy `CMakeUserPresets.json.example` to
`CMakeUserPresets.json` and edit `VCPKG_ROOT` and `MINGW_ROOT`.
Alternatively, set those environment variables and use the shared `mingw-dev`
or `mingw-release` presets directly.

The vcpkg toolchain is selected before `project()` through the preset. Its normal
application-local dependency deployment copies the matching DLLs and transitive
dependencies beside the executable after linking. Do not manually combine DLLs
from different configurations or toolchains in these output directories.
No global vcpkg PATH change is needed for the verified installation.

To add another OpenCV module later, add it to the `find_package` component list
and link its imported target, for example `opencv_imgproc`.

## Diagnosis and verification (2026-10-03)

The original executable crashes inside `imshow()` before `waitKey(0)`.
GDB captured SIGSEGV in the installed debug `libopencv_imgproc4d.dll`:

```text
cv::imshow
  -> Win32UIWindow::imshow -> showImage_ -> convertToShow
  -> cv::cvtColor -> AVX2 RGB2RGB -> cv::hal_AVX2::v_load_deinterleave

intrin_avx.hpp:2445
vmovdqa %ymm0,0x70(%rsp)
rsp = 0x3ccf220
```

The store destination is `0x3ccf290`, only 16-byte aligned; this 256-bit aligned
store requires 32-byte alignment. This is a fault in the installed debug library's
generated AVX2 code. The exact upstream compiler defect has not been established.
Both OpenCV's build records and the application identify w64devkit GCC 14.2.0,
so the evidence does not point to different compilers being selected here.

The old CMake cache had an empty `CMAKE_BUILD_TYPE`. Its actual link command
selected the debug OpenCV import libraries. An empty build type is not Release.
The new presets select configurations explicitly; direct single-configuration
CMake use also defaults to RelWithDebInfo when no build type is supplied.

The initial sandboxed run failed before startup with `0xc0000135`; running outside
the sandbox with the specified DLL PATH reproduced the actual AVX2 crash above.
The startup failure was not used as the diagnosis of the user's GUI problem.

Validation:

- Both final presets configured and built successfully.
- The final `local-dev` executable loaded the deployed release DLLs and reached
  `cv::waitKey` after printing `Before waitKey`.
- For the automated debugger run only, the waitKey argument was changed to 1 ms
  at function entry. It returned -1 and the process exited normally. The source
  still uses `waitKey(0)`; interactive keyboard input was not automated.
- No SIMD, threading, or GUI-backend override was added to the application.

Using RelWithDebInfo provides a working development configuration, but it does
not repair the installed OpenCV debug binaries. Full Debug use requires rebuilding
OpenCV with a toolchain/build that fixes the alignment fault and validating that
build. Merely selecting `CMAKE_BUILD_TYPE=Debug` would still use the faulty DLL.

References:

- [CMake build configurations](https://cmake.org/cmake/help/v4.3/manual/cmake-buildsystem.7.html#build-configurations)
- [CMake Tools presets](https://github.com/microsoft/vscode-cmake-tools/blob/main/docs/cmake-presets.md)
- [vcpkg MinGW guidance](https://learn.microsoft.com/en-us/vcpkg/users/platforms/mingw)
