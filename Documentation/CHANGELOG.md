# TNLMeans Changelog

**v1.1 (20210410 - pinterf)**

- project moved to github: https://github.com/pinterf/TNLMeans
- project update to Visual Studio 2019
- Change to Avisynth v2.6 interface (from 2.5)
- pass frame properties for AviSynth+ interface version >= 8
- Source code syntax updates: gcc, clang
- Add planar Y, YUV and RGB formats (but kept YUY2 by on-the-fly pre-post conversion)
- Add x64 config
- linux/gcc build, CMake build environment
- 10-16 bit support

**v1.0.3 (20070828  tritical's last version)**

- Removed fast exp() approximation that was used for sse=false.  Turns out it was quite inaccurate and had overflow problems resulting in artifacts.

**v1.0.2 (20060730)** 

- Fixed a problem with small weights causing artifacts

**v1.0.1 (20060619)**

- Fixed a bug that caused a crash when ms=true was used with yuy2 input

**v1.0 Final (20060531)**

- Fixed always creating the downsampled clip unless ms=false was explicitly specified

**v1.0 Beta 2 (20060525)**

- Added multiscale version (parameters ms/rm)
- Added sse parameter (sum of squared errors)
- optimized non-block based routines by buffering (100% speed increase)
- removed b parameter
- fixed a bug in the block based routines that caused some blocks in the search window not to be tested
- changed defaults for ax/ay/sx/sy/h

**v1.0 Beta 1 (20060517 - tritical)**

- Initial Release

