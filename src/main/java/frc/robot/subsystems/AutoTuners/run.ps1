param([ValidateSet('All', 'Flywheel', 'Arm', 'Elevator')][string]$Kind = 'All', [string]$ProjectRoot = '', [string]$Jdk = '', [switch]$Offline)
$ErrorActionPreference = 'Stop'
if (!$ProjectRoot) {
    $candidateRoot = Get-Item -LiteralPath $PSScriptRoot
    while ($candidateRoot -and !(Test-Path -LiteralPath (Join-Path $candidateRoot.FullName 'gradlew'))) {
        $candidateRoot = $candidateRoot.Parent
    }
    if (!$candidateRoot) { throw 'No Gradle wrapper found above this folder. Specify -ProjectRoot.' }
    $ProjectRoot = $candidateRoot.FullName
}
$savedJavaHome = $env:JAVA_HOME
if (!$Jdk -and !$env:JAVA_HOME) {
    $wpilibRoots = @()
    if ($env:PUBLIC) { $wpilibRoots += Join-Path $env:PUBLIC 'wpilib' }
    if ($env:USERPROFILE) { $wpilibRoots += Join-Path $env:USERPROFILE 'wpilib' }
    if ($env:HOME) { $wpilibRoots += Join-Path $env:HOME 'wpilib' }
    $Jdk = $wpilibRoots | Where-Object { Test-Path -LiteralPath $_ } |
        ForEach-Object { Get-ChildItem -LiteralPath $_ -Directory } |
        Sort-Object Name -Descending | ForEach-Object { Join-Path $_.FullName 'jdk' } |
        Where-Object { (Test-Path -LiteralPath (Join-Path $_ 'bin/java.exe')) -or (Test-Path -LiteralPath (Join-Path $_ 'bin/java')) } |
        Select-Object -First 1
}
Push-Location (Resolve-Path -LiteralPath $ProjectRoot)
try {
    if ($Jdk) { $env:JAVA_HOME = (Resolve-Path -LiteralPath $Jdk).Path }
    $wrapper = if ($env:OS -eq 'Windows_NT') { './gradlew.bat' } else { './gradlew' }
    $selectedKinds = if ($Kind -eq 'All') { @('Flywheel', 'Arm', 'Elevator') } else { @($Kind) }
    foreach ($selectedKind in $selectedKinds) {
        $taskName = $selectedKind.ToLower() + 'TunerSim'
        $logDirectory = 'build/' + $selectedKind.ToLower() + '-sim'
        $gradleArguments = @('--init-script', "$PSScriptRoot/tuner-simulation.gradle", '--console=plain', $taskName)
        if ($Offline) { $gradleArguments += '--offline' }
        New-Item -ItemType Directory -Force $logDirectory | Out-Null
        & $wrapper @gradleArguments | Tee-Object -FilePath (Join-Path $logDirectory 'results.txt')
        if ($LASTEXITCODE -ne 0) { throw 'Simulation failed; see Gradle output above.' }
    }
} finally {
    $env:JAVA_HOME = $savedJavaHome
    Pop-Location
}
