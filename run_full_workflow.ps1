param(
    [switch]$SkipInstall,
    [switch]$SkipTests,
    [switch]$SkipFlyPT,
    [switch]$SkipApp,
    [switch]$NoPause,
    [string]$FlyPTDir,
    [string]$ProfilePath
)

$ErrorActionPreference = "Stop"

$repoDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$outerRoot = Split-Path -Parent $repoDir
$venvDir = Join-Path $repoDir ".venv"
$requirementsFile = Join-Path $repoDir "requirements.txt"
$markerFile = Join-Path $venvDir ".requirements_installed"

function Write-Step {
    param([string]$Message)
    Write-Host ""
    Write-Host "==> $Message" -ForegroundColor Cyan
}

function Resolve-FirstExistingPath {
    param([string[]]$Candidates)
    foreach ($candidate in $Candidates) {
        if ($candidate -and (Test-Path $candidate)) {
            return $candidate
        }
    }
    return $null
}

function Find-Python {
    if (Get-Command py -ErrorAction SilentlyContinue) {
        return @{ Command = "py"; Args = @("-3") }
    }
    if (Get-Command python -ErrorAction SilentlyContinue) {
        return @{ Command = "python"; Args = @() }
    }
    throw "Python 3 was not found on PATH. Install Python, then rerun this launcher."
}

function Invoke-Python {
    param([Parameter(Mandatory = $true)][string[]]$Arguments)
    & $python.Command @($python.Args + $Arguments) 2>&1 | Write-Host
    if ($LASTEXITCODE -ne 0) {
        throw "Python command failed: $($Arguments -join ' ')"
    }
}

function Invoke-VenvPython {
    param([Parameter(Mandatory = $true)][string[]]$Arguments)
    & $venvPython @Arguments 2>&1 | Write-Host
    if ($LASTEXITCODE -ne 0) {
        throw "Venv Python command failed: $($Arguments -join ' ')"
    }
}

function Ensure-Venv {
    if (-not (Test-Path $venvPython)) {
        Write-Step "Creating virtual environment"
        Invoke-Python -Arguments @("-m", "venv", $venvDir)
    }
}

function Ensure-Requirements {
    $needsInstall = $true
    if ((Test-Path $markerFile) -and ((Get-Item $markerFile).LastWriteTimeUtc -ge (Get-Item $requirementsFile).LastWriteTimeUtc)) {
        $needsInstall = $false
    }
    if ($SkipInstall) {
        Write-Step "Skipping dependency install"
        return
    }
    if ($needsInstall) {
        Write-Step "Installing Python requirements"
        Invoke-VenvPython -Arguments @("-m", "pip", "install", "--upgrade", "pip")
        Invoke-VenvPython -Arguments @("-m", "pip", "install", "-r", $requirementsFile)
        Set-Content -Path $markerFile -Value (Get-Date).ToString("o") -Encoding UTF8
    } else {
        Write-Step "Requirements already installed"
    }
}

function Run-Tests {
    if ($SkipTests) {
        Write-Step "Skipping test run"
        return
    }
    Write-Step "Running test suite"
    Push-Location $repoDir
    try {
        Invoke-VenvPython -Arguments @("-m", "pytest", "-q")
    } finally {
        Pop-Location
    }
}

function Start-FlyPT {
    param(
        [string]$ResolvedFlyPTDir,
        [string]$ResolvedProfilePath
    )
    if ($SkipFlyPT) {
        Write-Step "Skipping FlyPT launch"
        return
    }
    $flyptExe = if ($ResolvedFlyPTDir) { Join-Path $ResolvedFlyPTDir "FlyPT Mover.exe" } else { $null }
    if (-not $flyptExe -or -not (Test-Path $flyptExe)) {
        Write-Warning "FlyPT executable not found. Pass -FlyPTDir or place 'FlyPT Mover 3.5.8' beside the repo or its parent folder."
        return
    }

    Write-Step "Launching FlyPT Mover"
    Start-Process -FilePath $flyptExe -WorkingDirectory $ResolvedFlyPTDir | Out-Null

    if ($ResolvedProfilePath -and (Test-Path $ResolvedProfilePath)) {
        Start-Sleep -Milliseconds 800
        try {
            Write-Step "Opening Mover profile"
            Start-Process -FilePath $ResolvedProfilePath | Out-Null
        } catch {
            Write-Warning "Could not auto-open $ResolvedProfilePath. Load it manually inside FlyPT if needed."
        }
    }
}

function Start-PoseTranslator {
    if ($SkipApp) {
        Write-Step "Skipping pose translator launch"
        return
    }
    Write-Step "Launching pose translator app"
    Start-Process -FilePath $venvPython -ArgumentList @("-m", "app.main") -WorkingDirectory $repoDir | Out-Null
}

if (-not (Test-Path $repoDir)) {
    throw "Repo folder not found: $repoDir"
}

$python = Find-Python
$venvPython = Join-Path $venvDir "Scripts\python.exe"

$resolvedFlyPTDir = Resolve-FirstExistingPath @(
    $FlyPTDir,
    (Join-Path $repoDir "FlyPT Mover 3.5.8"),
    (Join-Path $outerRoot "FlyPT Mover 3.5.8")
)
$resolvedProfilePath = Resolve-FirstExistingPath @(
    $ProfilePath,
    (Join-Path $repoDir "New.Mover"),
    (Join-Path $outerRoot "New.Mover")
)

Ensure-Venv
Ensure-Requirements
Run-Tests
Start-FlyPT -ResolvedFlyPTDir $resolvedFlyPTDir -ResolvedProfilePath $resolvedProfilePath
Start-PoseTranslator

Write-Step "Workflow complete"
Write-Host "Repo folder   : $repoDir"
Write-Host "FlyPT folder  : $resolvedFlyPTDir"
Write-Host "Mover profile : $resolvedProfilePath"
Write-Host "Venv python   : $venvPython"

if (-not $NoPause) {
    Write-Host ""
    Read-Host "Press Enter to close this launcher window"
}
