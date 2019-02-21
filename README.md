# RXPconvert

This is Richie's old RXPconvert code, compilable on Windows or Linux.

## Windows

Use the [Windows CMake GUI](https://cmake.org/download/) to generate the project file, then open the project in Visual Studio and compile.

## Linux

```bash
mkdir build
cd build
cmake .. -DRiVLib_DIR=/path/to/rivlib-2_5_y-x86-linux-gcc55/cmake
make
```
