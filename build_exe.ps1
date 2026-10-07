# 用已安装的 Python / PyInstaller 构建，不安装或下载依赖。
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$taskCache = [System.IO.Path]::GetFullPath((Join-Path $projectRoot '..\..\..\..\_cache\S100_PID_v38'))
$env:PYINSTALLER_CONFIG_DIR = Join-Path $taskCache 'config'
$temporaryBuild = Join-Path $taskCache ('ROV_ControlStation_v38_build_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $temporaryBuild -Force | Out-Null
foreach ($name in @('pc_main2.py', 'protocol.py', 'navigation.py', 'pid_panel.py', 'task_pid_wire.py')) {
    Copy-Item -LiteralPath (Join-Path $projectRoot $name) -Destination $temporaryBuild
}
$logPath = Join-Path $projectRoot 'build_v38.log'
$ErrorActionPreference = 'Continue'  # Windows PowerShell把原生stderr日志包装为错误记录；按进程退出码判断。
python -m PyInstaller --noconfirm --clean --onefile --windowed `
    --name ROV_ControlStation_v3.8 `
    --hidden-import protocol --hidden-import navigation --hidden-import pid_panel --hidden-import task_pid_wire --hidden-import serial.tools.list_ports `
    --exclude-module PyQt6 --exclude-module PySide6 --exclude-module PySide2 `
    --distpath $projectRoot --workpath (Join-Path $temporaryBuild 'work') --specpath $temporaryBuild `
    (Join-Path $temporaryBuild 'pc_main2.py') *> $logPath
$buildExit = $LASTEXITCODE
$ErrorActionPreference = 'Stop'
if ($buildExit -ne 0) {
    Get-Content -LiteralPath $logPath -Tail 35
    exit $buildExit
}
Get-Content -LiteralPath $logPath -Tail 15
Get-Item -LiteralPath (Join-Path $projectRoot 'ROV_ControlStation_v3.8.exe') | Select-Object Name, Length
