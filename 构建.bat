@echo off
rem ===========================================================================
rem  FH4 vehicle free-move tool -- build script (MSVC x64)
rem  Just run this file (double click, or "build.bat" in this folder).
rem  NOTE: keep this file ASCII-only; cmd.exe reads .bat as ANSI/GBK.
rem ===========================================================================
setlocal

set "VCVARS=C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"

if not exist "%VCVARS%" set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"

if not exist "%VCVARS%" (
    echo [X] vcvars64.bat not found.
    echo     Open "x64 Native Tools Command Prompt" and run:
    echo     cl /utf-8 /EHsc /O2 /std:c++17 /Fe:fh4_coord.exe fh4_coord.cpp
    pause
    exit /b 1
)

call "%VCVARS%" >nul
if errorlevel 1 (
    echo [X] failed to init MSVC environment
    pause
    exit /b 1
)

rem /MT = statically link the C/C++ runtime, so the target PC does NOT need
rem       VC++ Redistributable (MSVCP140.dll / VCRUNTIME140.dll).
rem       This toolset already defaults to static, but we pin /MT explicitly.
if not exist obj mkdir obj
cl /nologo /utf-8 /EHsc /O2 /std:c++17 /MT /W3 /I"imgui" /I"imgui\backends" ^
    fh4_coord.cpp imgui\imgui.cpp imgui\imgui_draw.cpp imgui\imgui_tables.cpp imgui\imgui_widgets.cpp ^
    imgui\backends\imgui_impl_win32.cpp imgui\backends\imgui_impl_dx11.cpp ^
    /Fe:fh4_coord.exe /Fo:obj\
if errorlevel 1 (
    echo.
    echo [X] build FAILED
    pause
    exit /b 1
)

rem keep obj files; they speed up rebuilds
echo.
echo [+] built: fh4_coord.exe
pause
