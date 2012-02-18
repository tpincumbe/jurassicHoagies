@echo on

#rmdir /S /Q build
#rmdir /S /Q include
#rmdir /S /Q temp
#del build\sensor.urf
#del include\scripts.inc
#del include\scripts.xml
#del temp\scripts\sensors.amx

..\..\bin\ugobe_project_tool.exe migio.upf rebuild

#move F:\sensor.urf F:\sensor.urf.backup

#copy build\sensor.urf F:\sensor.urf