@echo off

cd /d "%~dp0"

if not exist bin mkdir bin

cl /std:c++17 /O2 /EHsc /nologo src/main.cpp src/lbm.cpp src/setup.cpp src/graphics.cpp src/info.cpp src/kernel.cpp src/lodepng.cpp src/shapes.cpp /Fe:bin\FluidX3D.exe /Fobin\ /I. /Isrc /Isrc\OpenCL\include /link /LIBPATH:src\OpenCL\lib OpenCL.lib User32.lib Gdi32.lib Shell32.lib

if %errorlevel% neq 0 exit /b %errorlevel%