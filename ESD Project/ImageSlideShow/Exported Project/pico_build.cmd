@if not defined _echo echo off

goto main

:AddToPath

if exist "%~1" (
  set "PATH=%~1;%PATH%"
)

goto :EOF

:main

for %%i in (sdk examples extras playground) do (
  rem Environment variables in Windows aren't case-sensitive, so we don't need
  rem to bother with uppercasing the env var name.
  if exist "%~dp0pico-%%i" (
    echo PICO_%%i_PATH=%~dp0pico-%%i
    set "PICO_%%i_PATH=%~dp0pico-%%i"
  )
)

if exist "%~dp0tools\openocd-picoprobe" (
  echo OPENOCD_SCRIPTS=%~dp0tools\openocd-picoprobe\scripts
  set "OPENOCD_SCRIPTS=%~dp0tools\openocd-picoprobe\scripts"
  set "PATH=%~dp0tools\openocd-picoprobe;%PATH%"
)

call :AddToPath "%ProgramFiles(x86)%\doxygen\bin"
call :AddToPath "%ProgramFiles%\doxygen\bin"
call :AddToPath "%ProgramW6432%\doxygen\bin"

call :AddToPath "%ProgramFiles(x86)%\Graphviz\bin"
call :AddToPath "%ProgramFiles%\Graphviz\bin"
call :AddToPath "%ProgramW6432%\Graphviz\bin"

call :AddToPath "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer"
call :AddToPath "%ProgramFiles%\Microsoft Visual Studio\Installer"

rem https://github.com/microsoft/vswhere/wiki/Start-Developer-Command-Prompt

for /f "usebackq delims=" %%i in (`vswhere.exe -products * -requires "Microsoft.VisualStudio.Component.VC.CoreIde" -latest -property installationPath`) do (
  if exist "%%i\Common7\Tools\vsdevcmd.bat" (
    call "%%i\Common7\Tools\vsdevcmd.bat"
  )
)
cmake -D PICO_DEFAULT_BOOT_STAGE2:STRING=boot2_generic_03h -G "NMake Makefiles" & nmake
if exist "%~dp0Data\__Flash.bin" (
	if exist "%~dp0uf2conv4eve.py" (
		python uf2conv4eve.py -c --input %~dp0Data/__Flash.bin --keep --firmware BT815 %~dp0BT815/unified.blob --firmware BT817 %~dp0BT817/unified.blob -o %~dp0Bin\__Flash.uf2 
	)
)

