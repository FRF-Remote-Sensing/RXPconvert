# RXPconvert

This is Richie's old RXPconvert code, compilable on Windows or Linux

## Windows

Open ```RXPconvert.vcxproj``` to compile

## Linux

```bash
mkdir build
cd build
cmake .. -DCMAKE_PREFIX_PATH=/path/to/rivlib-2_0_1-x86-linux-gcc44/
make
```

#### Compiling Issues

- If you run into errors with missing ```bits/c++config.h``` headers, install ```gcc-multilib``` and ```g++-multilib```
- If you get errors that say there is an incompatibility with ```i386 output```, open the ```CMakeLists.txt``` file and comment out the line that deals with 32-bit architecture
