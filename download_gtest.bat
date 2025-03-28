@echo off
echo Downloading Google Test...

:: Create third_party directory if it doesn't exist
if not exist third_party mkdir third_party
cd third_party

:: Clone Google Test repository
git clone https://github.com/google/googletest.git

:: Create build directory
cd googletest
mkdir build
cd build

:: Configure and build
cmake ..
cmake --build . --config Release

cd ../../..
echo Google Test setup complete! 