@echo on

rmdir /S /Q build
rmdir /S /Q include
rmdir /S /Q temp
del .\build\sensor.urf
del .\include\scripts.inc
del .\include\scripts.xml
del .\temp\scripts\sensors.amx

.\files\bin\ugobe_project_tool.exe migio.upf rebuild

copy .\build\sensor.urf I:\sensor.urf
copy .\build\sensor.urf .\sensor.urf

pause