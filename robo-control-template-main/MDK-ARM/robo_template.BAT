SET PATH=D:\Keil_v5\ARM\ARMCC\Bin;D:\Keil_v5\ARM\ARMCLANG\Bin;G:\Program Files (x86)\VMware\VMware Workstation\bin\;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\Program Files\Common Files\Oracle\Java\javapath;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files\Pandoc\;C:\Program Files\CMake\bin;C:\Program Files\Git\cmd;G:\Program Files\PuTTY\;D:\Program Files (x86)\PDFtk\bin\;D:\MinGW\MinGW\bin;C:\Program Files\SEGGER\JLink;C:\Users\Administrator\AppData\Roaming\nvm;C:\Program Files\nodejs;C:\Program Files\dotnet\;C:\Program Files\Common Files\Autodesk Shared\;C:\Program Files\Microsoft SQL Server\120\Tools\Binn\;G:\Program Files\MATLAB\R2020b\bin;C:\Program Files\apache-maven-3.8.5\bin;D:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin;C:\Users\Administrator\AppData\Local\Programs\Python\Python39\Scripts\;C:\Users\Administrator\AppData\Local\Programs\Python\Python39\;C:\Users\Administrator\AppData\Local\Microsoft\WindowsApps;D:\Program Files (x86)\Microsoft VS Code\bin;C:\Users\Administrator\AppData\Roaming\npm;C:\Users\Administrator\AppData\Local\GitHubDesktop\bin;G:\ffmpeg-master-latest-win64-gpl\ffmpeg-master-latest-win64-gpl\bin;G:\platform-tools_r32.0.0-windows\platform-tools;C:\Program Files\Java\jdk1.8.0_202\bin;G:\Program Files\JetBrains\PyCharm Community Edition 2021.3.2\bin;G:\intelFPGA\21.1\questa_fse\win64;G:\ProgramData\Anaconda3\bin
SET CPU_TYPE=STM32F407IGHx
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=robo_template
SET CPU_CLOCK=0x00B71B00
"D:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"robo_template\startup_stm32f407xx._ac"
"D:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"robo_template\gpio.__i"
"D:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"robo_template\adc.__i"
"D:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"robo_template\can.__i"
"D:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"robo_template\dma.__i"
"D:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"robo_template\i2c.__i"
"D:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"robo_template\iwdg.__i"
"D:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"robo_template\spi.__i"
"D:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"robo_template\tim.__i"
"D:\Keil_v5\ARM\ARMCLANG\Bin\ArmLink" --Via "robo_template\robo_template.lnp"
"D:\Keil_v5\ARM\ARMCLANG\Bin\fromelf.exe" "robo_template\robo_template.axf" --i32combined --output "robo_template\robo_template.hex"
