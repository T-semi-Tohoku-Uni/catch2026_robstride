param(
    [string]$Compiler = '',
    [string]$BuildDirectory = '',
    [switch]$Sanitizers
)
$ErrorActionPreference = 'Stop'
$repoDirectory = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if (-not $BuildDirectory) { $BuildDirectory = Join-Path $repoDirectory 'build/host-tests' }
if (-not $Compiler) {
    $gccCommand = Get-Command gcc -ErrorAction SilentlyContinue
    if ($gccCommand) { $Compiler = $gccCommand.Source }
    else {
        $Compiler = Join-Path $env:LOCALAPPDATA 'Programs/CLion/bin/mingw/bin/gcc.exe'
    }
}
if (-not (Test-Path -LiteralPath $Compiler)) {
    throw 'Native C compiler not found. Specify -Compiler C:/path/to/gcc.exe or clang.exe.'
}
$Compiler = [IO.Path]::GetFullPath($Compiler).Replace('\', '/')
$env:PATH = (Split-Path -Parent $Compiler) + [IO.Path]::PathSeparator + $env:PATH
$pythonCommand = Get-Command py -ErrorAction SilentlyContinue
if ($pythonCommand) {
    $pythonExecutable = (& $pythonCommand.Source -3 -c 'import sys; print(sys.executable)').Trim().Replace('\', '/')
} else {
    $pythonExecutable = (Get-Command python3 -ErrorAction Stop).Source
}
$sanitizerValue = if ($Sanitizers) { 'ON' } else { 'OFF' }
& cmake -S $PSScriptRoot -B $BuildDirectory -G Ninja "-DCMAKE_C_COMPILER=$Compiler" "-DPython3_EXECUTABLE=$pythonExecutable" "-DCYBERGEAR_SANITIZERS=$sanitizerValue" -DCMAKE_BUILD_TYPE=Debug
if ($LASTEXITCODE -ne 0) { throw 'Host test configuration failed.' }
& cmake --build $BuildDirectory
if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed.' }
& ctest --test-dir $BuildDirectory --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'Host tests failed.' }
