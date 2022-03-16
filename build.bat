mkdir build
cd build

cmake -G "Visual Studio 16 2019" -A "x64" -T "host=x64" ^
   -DCMAKE_BUILD_TYPE=Release ^
   ../