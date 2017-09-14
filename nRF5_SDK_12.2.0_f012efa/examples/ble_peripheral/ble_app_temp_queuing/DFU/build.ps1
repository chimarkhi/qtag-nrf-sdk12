#$adbPath = "E:\Android\sdk\platform-tools\adb.exe"
$adbPath = "adb"

Function install-mqtt-app
{
    Param()
    
    $installApp = $adbPath + " install -r .\Release\mqtt.apk"

    Invoke-Expression $installApp
}

Function install-gateway-app
{
    Param()
    
    #$installApp = $adbPath + " install -r .\Release\gateway.apk"
    #Invoke-Expression $installApp1

    $pushGatewayApp = $adbPath + " push .\Release\gateway.apk /system/app"
    $changeFilePermission = $adbPath + " shell ""su 0 chmod 644 /system/app/gateway.apk"""

    Invoke-Expression $pushGatewayApp
    Invoke-Expression $changeFilePermission
}

Function delete-gateway-app
{
    Param()

    $removeFile = $adbPath + " shell ""su 0 rm /system/app/gateway.apk"""

    Invoke-Expression $removeFile
}

Function delete-monitor-app
{
    Param()

    $step2 = $adbPath + " shell ""su 0 rm /system/app/gatewaymonitor.apk"""

    Invoke-Expression $step2
}

Function install-monitor-app
{
    Param()

    #$pushMonitorApp = $adbPath + " push .\Release\gatewaymonitor.apk /sdcard/Download"
        
    $pushMonitorApp = $adbPath + " push .\Release\gatewaymonitor.apk /system/app"
    $changeFilePermission = $adbPath + " shell ""su 0 chmod 644 /system/app/gatewaymonitor.apk"""

    Invoke-Expression $pushMonitorApp
    Invoke-Expression $changeFilePermission
}

Function reboot-gateway
{
    Param()
    
    Invoke-Expression ($adbPath + " reboot")
}

Function readWriteSystem
{
    Invoke-Expression ($adbPath + " shell ""su 0 mount -o rw,remount /system""")
}

Function adbRoot
{
    Invoke-Expression ($adbPath + " root")
}

Function setup-phone
{
    Invoke-Expression ($adbPath + " shell settings put global data_roaming 1")

    adb shell dumpsys deviceidle whitelist +com.tagbox.gateway
    Start-Sleep -s 1
    adb shell dumpsys deviceidle whitelist +com.tagbox.gatewaymonitor
    Start-Sleep -s 1
    adb shell dumpsys deviceidle whitelist +com.tagbox.mqttconnection
    Start-Sleep -s 1
}

Function grant-permissions
{
    Invoke-Expression ($adbPath + " shell pm grant com.tagbox.gateway android.permission.ACCESS_FINE_LOCATION")
    Invoke-Expression ($adbPath + " shell pm grant com.tagbox.gateway android.permission.ACCESS_COARSE_LOCATION")
    Invoke-Expression ($adbPath + " shell pm grant com.tagbox.gateway android.permission.WRITE_EXTERNAL_STORAGE")
    Invoke-Expression ($adbPath + " shell pm grant com.tagbox.gateway android.permission.ACCESS_NETWORK_STATE")
    Invoke-Expression ($adbPath + " shell pm grant com.tagbox.gateway android.permission.WRITE_SETTINGS")
    Invoke-Expression ($adbPath + " shell pm grant com.tagbox.gateway android.permission.READ_PHONE_STATE")

    Invoke-Expression ($adbPath + " shell pm grant com.tagbox.gatewaymonitor android.permission.INSTALL_PACKAGES")
    Invoke-Expression ($adbPath + " shell pm grant com.tagbox.gatewaymonitor android.permission.DELETE_PACKAGES")
    Invoke-Expression ($adbPath + " shell pm grant com.tagbox.gatewaymonitor android.permission.WRITE_EXTERNAL_STORAGE")
    Invoke-Expression ($adbPath + " shell pm grant com.tagbox.gatewaymonitor android.permission.MODIFY_PHONE_STATE")

    Invoke-Expression ($adbPath + " shell pm grant com.tagbox.mqttconnection android.permission.READ_PHONE_STATE")
    Invoke-Expression ($adbPath + " shell pm grant com.tagbox.mqttconnection android.permission.CHANGE_NETWORK_STATE")
    Invoke-Expression ($adbPath + " shell pm grant com.tagbox.mqttconnection android.permission.MODIFY_PHONE_STATE")
    Invoke-Expression ($adbPath + " shell pm grant com.tagbox.mqttconnection android.permission.WRITE_EXTERNAL_STORAGE")
}

cmd.exe /c 'gradlew.bat clean'
cmd.exe /c 'gradlew.bat assembleRelease'
write-host "The Last Exit Code is:" $LastExitCode

if($LastExitCode -eq 0) {

    New-Item -ItemType Directory -Force -Path Release

    $gatewayApk = ".\gateway\build\outputs\apk"
    $monitoringApk = ".\gatewaymonitor\build\outputs\apk"
    $mqttApk = ".\mqttconnection\build\outputs\apk"

    $destpath = ".\Release"

    Remove-Item .\Release\*

    get-childitem -path $gatewayApk -Filter "Tagbox-Gateway-*.apk" |
        sort-object -Property LastWriteTime |
        select-object -last 1 | copy-item -Destination (join-path $destpath "gateway.apk")

    get-childitem -path $monitoringApk -Filter "Tagbox-GatewayMonitor-*.apk" |
        sort-object -Property LastWriteTime |
        select-object -last 1 | copy-item -Destination (join-path $destpath "gatewaymonitor.apk")

    get-childitem -path $mqttApk -Filter "Tagbox-Mqtt-*.apk" |
        sort-object -Property LastWriteTime |
        select-object -last 1 | copy-item -Destination (join-path $destpath "mqtt.apk")

    $listDevices = $adbPath + " devices"

    $result = Invoke-Expression $listDevices
    
    #Write-Host $result

    $devices = $result -replace "List of devices attached"
    
    Write-Host $devices

    if($devices -like '*device*') {
        adbRoot

        Start-Sleep -s 3
		
		readWriteSystem

        delete-gateway-app

        install-gateway-app
        
        delete-monitor-app
        
        install-monitor-app

        grant-permissions

        install-mqtt-app

        setup-phone
        
        reboot-gateway        
    }
}