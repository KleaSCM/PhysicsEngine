@echo off
echo Building tests...

:: Create build directory if it doesn't exist
if not exist build mkdir build
cd build

:: Compile test file
g++ ../tests/test_mathutils.cpp ../src/MathUtils.cpp -o test_mathutils.exe

:: Run tests
echo Running tests...
.\test_mathutils.exe

cd .. 