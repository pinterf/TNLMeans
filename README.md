# TNLMeans
TNLMeans is an implementation of the NL-means denoising algorithm in an Avisynth filter (by tritical)

### Requirements

This filter requires AviSynth 2.6.0 or AviSynth+ as well as the Visual C++ Redistributable Package for Visual Studio 2015-19.

## Links

Avisynth wiki: http://avisynth.nl/index.php/TNLMeans

Project on github: https://github.com/pinterf/TNLMeans

### Plugin history

Based on tritical's v1.03 (2007)
Additional works by pinterf
- move to github
- move to Visual Studio 2019
- C++ syntax updates
- Update to be an Avisynth v2.6 interface plugin
- x64 build
- Linux/GCC build

see CHANGELOG.md or the documentation

Build instructions
------------------
### Windows Visual Studio MSVC

use IDE

### Windows GCC

(mingw installed by msys2)
 From the 'build' folder under project root:

```
del ..\CMakeCache.txt
cmake .. -G "MinGW Makefiles" -DENABLE_INTEL_SIMD:bool=on
@rem test: cmake .. -G "MinGW Makefiles" -DENABLE_INTEL_SIMD:bool=off
cmake --build . --config Release
```

### Linux

Note: ENABLE_INTEL_SIMD is automatically off for non-x86 architectures

Clone repo and build

```
git clone https://github.com/pinterf/TNLMeans
cd TNLMeans
cmake -B build -S .
cmake --build build
```

Useful hints:

build after clean:

```
cmake --build build --clean-first
```

Force no Intel x86-assembler support:

```
cmake -B build -S . -DENABLE_INTEL_SIMD:bool=off
```

delete CMake cache:

```
rm build/CMakeCache.txt
```



* Find binaries at
  
        build/TNLMeans/libtnlmeans.so

* Install binaries

        cd build
        sudo make install

