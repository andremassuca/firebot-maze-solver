@echo off
rem Compila a FIREBOT App com o compilador de C# que vem com o Windows (.NET Framework 4), sem instalar nada.
rem O Bluetooth BLE usa os ficheiros .winmd que ja vem com o Windows 10/11.
cd /d "%~dp0"
set FW=%WINDIR%\Microsoft.NET\Framework64\v4.0.30319
set WM=%WINDIR%\System32\WinMetadata
"%FW%\csc.exe" /nologo /target:winexe /optimize+ /out:FirebotApp.exe /win32icon:firebot.ico /r:System.Management.dll /r:System.Windows.Forms.dll /r:System.Drawing.dll "/r:%WM%\Windows.Devices.winmd" "/r:%WM%\Windows.Foundation.winmd" "/r:%WM%\Windows.Storage.winmd" "/r:%FW%\System.Runtime.dll" "/r:%FW%\System.Threading.Tasks.dll" "/r:%FW%\System.Runtime.InteropServices.WindowsRuntime.dll" FirebotApp.cs FirebotBle.cs
if errorlevel 1 (echo. & echo Erro na compilacao. & pause) else (echo FirebotApp.exe criado.)
