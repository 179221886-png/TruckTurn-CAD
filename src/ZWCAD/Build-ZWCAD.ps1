param(
    [Parameter(Mandatory = $true)] [string]$SdkRoot,
    [int[]]$Years = @(2022, 2023, 2024, 2025, 2026),
    [Parameter(Mandatory = $true)] [string]$OutputRoot,
    [string]$Dotnet = 'dotnet',
    [string]$NuGetConfig,
    [string]$PackagesPath
)

# Compile only: never start CAD or redistribute its native/managed SDK files.
# SdkRoot layout: 2022/ZwManaged.dll + ZwDatabaseMgd.dll, ... , 2026/...
$ErrorActionPreference = 'Stop'
$sdkPath = [IO.Path]::GetFullPath($SdkRoot)
$outputPath = [IO.Path]::GetFullPath($OutputRoot)
$allowed = @(2022, 2023, 2024, 2025, 2026)
if ($Years.Count -eq 0) { throw 'Select at least one supported year.' }
if (@($Years | Select-Object -Unique).Count -ne $Years.Count) { throw 'Duplicate year selected.' }

# Validate every input before starting any build. A 2025 API cannot stand in for 2022.
foreach ($year in $Years) {
    if ($year -notin $allowed) { throw "No validated SDK target for ZWCAD $year. Supported: $($allowed -join ', ')." }
    $sdk = Join-Path $sdkPath $year.ToString()
    foreach ($name in @('ZwManaged', 'ZwDatabaseMgd')) {
        $file = Join-Path $sdk ($name + '.dll')
        if (-not (Test-Path -LiteralPath $file -PathType Leaf)) { throw "Missing reference: $file" }
        $identity = [Reflection.AssemblyName]::GetAssemblyName($file)
        if ($identity.Name -ne $name -or $identity.Version.Major -ne ($year - 2000)) {
            throw "Wrong reference identity for $year : $($identity.FullName). Supply that year's original Windows x64 API."
        }
    }
    $target = Join-Path $outputPath $year.ToString()
    if (Test-Path -LiteralPath $target) { throw "Existing build is preserved: $target. Choose a fresh OutputRoot." }
}

foreach ($year in $Years) {
    $project = Join-Path $PSScriptRoot ('TruckTurn.ZWCAD' + $year + '.csproj')
    $target = Join-Path $outputPath $year.ToString()
    New-Item -ItemType Directory -Path $target -Force | Out-Null
    $arguments = @('build', $project, '-c', 'Release', '-o', $target, '--nologo', '--disable-build-servers',
        ('-p:ZwcadSdkDir=' + (Join-Path $sdkPath $year.ToString())), '-p:UseSharedCompilation=false', '-p:NuGetAudit=false')
    if ($NuGetConfig) { $arguments += @('--configfile', [IO.Path]::GetFullPath($NuGetConfig)) }
    if ($PackagesPath) { $arguments += ('-p:RestorePackagesPath=' + [IO.Path]::GetFullPath($PackagesPath)) }
    & $Dotnet @arguments
    if ($LASTEXITCODE -ne 0) { throw "Build failed for ZWCAD $year." }
    if (-not (Test-Path -LiteralPath (Join-Path $target ('TruckTurn.ZWCAD' + $year + '.dll')))) {
        throw "Expected TruckTurn DLL missing for $year."
    }
    $hostDlls = Get-ChildItem -LiteralPath $target -File |
        Where-Object { $_.Name -match '^(ZwManaged|ZwDatabaseMgd.*|ZcWindows|ZdWindows|ZcCui|AcMgd|AcDbMgd|AcCoreMgd|GcMgd|GcDbMgd|GcCoreMgd)\.dll$' }
    if ($hostDlls) { throw "Host SDK must not be shipped: $($hostDlls.Name -join ', ')" }
}

Write-Output "Built Windows x64 .NET Framework 4.7 test DLLs. CAD host acceptance is still required. Output: $outputPath"
