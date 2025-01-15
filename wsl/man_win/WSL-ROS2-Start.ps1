[string]$DistroName = "WSL-ROS2"
[string]$TarBallName = "wsl-ros2-v2425.01.tar"
[string]$TarBallPath = "$env:SystemDrive\WSL-ROS2\$TarBallName"
[string]$DistroTargetPath = "$env:LOCALAPPDATA\$DistroName"
[string]$WinTermSettings = "$env:SystemDrive\WSL-ROS2\settings.json"
[string]$WinTermSettingsTargetPath = "$env:LOCALAPPDATA\Packages\Microsoft.WindowsTerminal_8wekyb3d8bbwe\LocalState"
[string]$WinTermSettingsTargetFilePath = "$env:LOCALAPPDATA\Packages\Microsoft.WindowsTerminal_8wekyb3d8bbwe\LocalState\settings.json"
[string]$WinTermSettingsBackup = "$env:LOCALAPPDATA\prevterminalsettings.json"

If (-not(Get-Process | Where-Object {$_.Name -eq "vcxsrv"}))
{
    Start-Process -FilePath "$env:SystemDrive\ProgramData\Microsoft\AppV\Client\Integration\8FCACB30-2BA0-4AFE-9816-259ED56E59EB\Root\VFS\ProgramFilesX64\VcXsrv\xlaunch.exe" -ArgumentList "-run $env:SystemDrive\WSL-ROS\wsl_ros_config.xlaunch"
}

$console_encoding = ([console]::OutputEncoding)
[console]::OutputEncoding = New-Object System.Text.UnicodeEncoding
Clear
$Distros = wsl --list
[console]::OutputEncoding = $console_encoding

If (-not (Test-Path -Path $WinTermSettingsTargetFilePath))
{
    $WinTermSettingsFolder = New-Item -Path $WinTermSettingsTargetPath -ItemType Directory -Force
    Copy-Item -Path $WinTermSettings -Destination $WinTermSettingsTargetPath -Force
    Write-Host
}

Write-Host
If ($Distros | Where-Object {$_ -eq $DistroName -or $_ -eq ($DistroName + " (Default)")})
{
    Write-Host "You already have a $DistroName distribution installed."
    Write-Host
    $Reply = Read-Host -Prompt "Would you like to carry on using it? (Y/N)"
    Write-Host
    If ($Reply -eq "Y" -or $Reply -eq "YES")
    {

        Start-Process -FilePath "shell:AppsFolder\Microsoft.WindowsTerminal_8wekyb3d8bbwe!App"
        Exit  
    }
    $Reply = Read-Host -Prompt "Is it OK to delete the current distribution and install a fresh copy? (Y/N)"
    If ($Reply -eq "Y" -or $Reply -eq "YES")
    {
        wsl --unregister $DistroName
        Clear
		
    }
    Else
    {
        Write-Host "Exiting..."
        Start-Sleep -Seconds 3
        Exit
    }
}
Else
{
    If ((Get-FileHash -Path $WinTermSettings -Algorithm MD5).Hash -ne `
		(Get-FileHash -Path $WinTermSettingsTargetFilePath -Algorithm MD5).Hash)
		{
    	    Write-Host
    	    Write-Host "Overwriting Windows Terminal settings..."
		    Write-Host "Previous settings will be stored here: $WinTermSettingsBackup"
		    Copy-Item -Path $WinTermSettingsTargetFilePath -Destination $WinTermSettingsBackup -Force
		    Copy-Item -Path $WinTermSettings -Destination $WinTermSettingsTargetPath -Force
    	    Write-Host
            Start-Sleep -Seconds 3
        }
}

Write-Host "Installing... Please wait..."
Write-Host
wsl --import $DistroName $DistroTargetPath $TarBallPath --version 2
Start-Process -FilePath "shell:AppsFolder\Microsoft.WindowsTerminal_8wekyb3d8bbwe!App"