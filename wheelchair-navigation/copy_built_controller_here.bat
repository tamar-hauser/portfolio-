@echo off
set SRC=C:\Users\User\Desktop\TrackObject\controllers\wheelchair_cpp_controller\wheelchair_cpp_controller.exe
set DST=%~dp0controllers\wheelchair_cpp_controller\wheelchair_cpp_controller.exe

echo Source: %SRC%
echo Target: %DST%

if not exist "%SRC%" (
  echo ERROR: Built controller exe was not found.
  echo First build the project with:
  echo cmake --build build --config Debug --target wheelchair_cpp_controller
  pause
  exit /b 1
)

copy /Y "%SRC%" "%DST%"
if errorlevel 1 (
  echo ERROR: Copy failed.
  pause
  exit /b 1
)

echo Done. The simulation folder now contains wheelchair_cpp_controller.exe
pause
