$ErrorActionPreference = "Stop"

$ProjectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$SdkDir = "C:\Users\ruler1591\AppData\Local\Android\Sdk"
$JavaHome = "C:\Program Files\Android\Android Studio\jbr"
$GradleBat = "C:\Users\ruler1591\.gradle\wrapper\dists\gradle-9.3.0-bin\79n14ral3mx1ozqr3csh2u872\gradle-9.3.0\bin\gradle.bat"
$OutputApk = Join-Path $ProjectDir "app\build\outputs\apk\debug\app-debug.apk"
$CopyApk = Join-Path $ProjectDir "JMEngine-debug.apk"

if (!(Test-Path $SdkDir)) {
    throw "Android SDK not found: $SdkDir"
}

if (!(Test-Path $JavaHome)) {
    throw "Android Studio JBR not found: $JavaHome"
}

if (!(Test-Path $GradleBat)) {
    throw "Gradle 9.3.0 not found: $GradleBat"
}

$env:ANDROID_HOME = $SdkDir
$env:ANDROID_SDK_ROOT = $SdkDir
$env:JAVA_HOME = $JavaHome

Push-Location $ProjectDir
try {
    & $GradleBat --no-daemon assembleDebug
    if ($LASTEXITCODE -ne 0) {
        throw "Gradle build failed with exit code $LASTEXITCODE"
    }

    if (!(Test-Path $OutputApk)) {
        throw "APK was not generated: $OutputApk"
    }

    Copy-Item $OutputApk $CopyApk -Force
    Write-Host "APK generated:" -ForegroundColor Green
    Write-Host $CopyApk
} finally {
    Pop-Location
}
