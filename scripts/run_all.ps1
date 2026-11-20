# Builds and runs every testbench in the repository, printing one PASS/FAIL line each and a total.
$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

# Append MSYS2 ucrt64 bin so system Python (with numpy/PIL) takes precedence over MSYS2 python
$msys = "C:\msys64\ucrt64\bin"
if ((Test-Path $msys) -and ($env:Path -notlike "*$msys*")) {
    $env:Path = "$env:Path;$msys"
}

$buildDir = Join-Path $repoRoot "build"
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

function Ensure-Vectors {
    # 1. Pyramid vectors (480x640 test.png)
    $pyrSim = Join-Path $repoRoot "hw\pyramid\sim"
    if (-not (Test-Path (Join-Path $pyrSim "level3_acc.hex"))) {
        Write-Host "[prep] Generating pyramid golden vectors..."
        Push-Location $pyrSim
        python ..\model\orb_pyramid_golden.py --input test.png --outdir . | Out-Null
        if ($LASTEXITCODE -ne 0) { Pop-Location; throw "orb_pyramid_golden.py failed" }
        Pop-Location
    }

    # 2. Frontend vectors (480x640 test.png)
    $feSim = Join-Path $repoRoot "hw\frontend_top\sim"
    if (-not (Test-Path (Join-Path $feSim "level3_kp_count.hex"))) {
        Write-Host "[prep] Generating frontend golden vectors..."
        Push-Location $feSim
        python ..\..\pyramid\model\orb_pyramid_golden.py --input test.png --outdir . | Out-Null
        if ($LASTEXITCODE -ne 0) { Pop-Location; throw "orb_pyramid_golden.py (frontend) failed" }
        python ..\model\orb_frontend_golden.py --dir . --fast-thr 20 | Out-Null
        if ($LASTEXITCODE -ne 0) { Pop-Location; throw "orb_frontend_golden.py failed" }
        Pop-Location
    }

    # 2b. Full-system golden vectors (reads default SIM_IMG_W/SIM_IMG_H/FAST_THR from vslam_cfg_pkg.sv)
    if (-not (Test-Path (Join-Path $feSim "sys_expected_counts.hex"))) {
        Write-Host "[prep] Generating full-system golden vectors..."
        Push-Location $feSim
        python ..\model\orb_system_golden.py gen --input ..\..\rbrief\sim\tes_gambar.jpg --outdir . | Out-Null
        if ($LASTEXITCODE -ne 0) { Pop-Location; throw "orb_system_golden.py gen failed" }
        Pop-Location
    }

    # 3. Orientation vectors
    $oriSim = Join-Path $repoRoot "hw\orientation\sim"
    if (-not (Test-Path (Join-Path $oriSim "expected_top.txt"))) {
        Write-Host "[prep] Generating orientation golden vectors..."
        Push-Location $oriSim
        python gen_vectors.py | Out-Null
        if ($LASTEXITCODE -ne 0) { Pop-Location; throw "gen_vectors.py failed" }
        Pop-Location
    }

    # 4. rBRIEF vectors
    $rbSim = Join-Path $repoRoot "hw\rbrief\sim"
    if (-not (Test-Path (Join-Path $rbSim "vectors\trig_decoder.hex")) -or
        -not (Test-Path (Join-Path $rbSim "vectors\expected.hex"))) {
        Write-Host "[prep] Generating rBRIEF golden vectors..."
        Push-Location $rbSim
        python ..\model\trig_decoder.py | Out-Null
        if ($LASTEXITCODE -ne 0) { Pop-Location; throw "trig_decoder.py failed" }
        python ..\model\rbrief_python.py | Out-Null
        if ($LASTEXITCODE -ne 0) { Pop-Location; throw "rbrief_python.py failed" }
        Pop-Location
    }
}

Ensure-Vectors

$tests = @(
    @{ Name = "tb_image_pyramid";   Top = "TBImagePyramid";    Tb = "hw\pyramid\tb\tb_image_pyramid.sv";       SimDir = "hw\pyramid\sim";      Inc = "";                    Args = "" },
    @{ Name = "tb_orb_pyramid_top"; Top = "tb_orb_pyramid_top";Tb = "hw\pyramid\tb\tb_orb_pyramid_top.sv";     SimDir = "hw\pyramid\sim";      Inc = "";                    Args = "" },
    @{ Name = "tb_frontend";        Top = "TB_Frontend";       Tb = "hw\frontend_top\tb\tb_frontend.sv";      SimDir = "hw\frontend_top\sim"; Inc = "";                    Args = "" },
    @{ Name = "tb_system_top";      Top = "tb_system_top";     Tb = "hw\frontend_top\tb\tb_system_top.sv";    SimDir = "hw\frontend_top\sim"; Inc = "";                    Args = "" },
    @{ Name = "tb_record_writer";   Top = "tb_record_writer";  Tb = "hw\frontend_top\tb\tb_record_writer.sv"; SimDir = "hw\frontend_top\sim"; Inc = "";                    Args = "" },
    @{ Name = "tb_orient_gap0";     Top = "tb_orient";         Tb = "hw\orientation\tb\tb_orient.sv";         SimDir = "hw\orientation\sim";  Inc = "hw\orientation\sim";  Args = "" },
    @{ Name = "tb_orient_gap40";    Top = "tb_orient";         Tb = "hw\orientation\tb\tb_orient.sv";         SimDir = "hw\orientation\sim";  Inc = "hw\orientation\sim";  Args = "+gap=40" },
    @{ Name = "tb_trig_decoder";    Top = "tb_trig_decoder";   Tb = "hw\rbrief\tb\tb_trig_decoder.sv";        SimDir = "hw\rbrief\sim";       Inc = "";                    Args = "" },
    @{ Name = "tb_rbrief";          Top = "tb_rbrief";         Tb = "hw\rbrief\tb\tb_rbrief.sv";              SimDir = "hw\rbrief\sim";       Inc = "";                    Args = "" },
    @{ Name = "tb_rbrief_top";      Top = "tb_rbrief_top";     Tb = "hw\rbrief\tb\tb_rbrief_top.sv";          SimDir = "hw\rbrief\sim";       Inc = "";                    Args = "" }
)

$passed = 0
$failed = 0
$compiled = @{}

foreach ($t in $tests) {
    $outFile = Join-Path $buildDir "$($t.Top).out"
    if (-not $compiled.ContainsKey($t.Top)) {
        $prevEAP = $ErrorActionPreference
        $ErrorActionPreference = "Continue"
        if ($t.Inc -ne "") {
            $compileOut = & iverilog -g2012 -Wall "-I$($t.Inc)" -s $t.Top -o $outFile -c scripts\rtl.f $t.Tb 2>&1 | ForEach-Object { "$_" }
        } else {
            $compileOut = & iverilog -g2012 -Wall -s $t.Top -o $outFile -c scripts\rtl.f $t.Tb 2>&1 | ForEach-Object { "$_" }
        }
        $compileCode = $LASTEXITCODE
        $ErrorActionPreference = $prevEAP
        if ($compileCode -ne 0) {
            Write-Host "$($t.Name): FAIL (compile error)"
            $compileOut | Select-Object -Last 20 | ForEach-Object { Write-Host "  $_" }
            $failed++
            continue
        }
        $compiled[$t.Top] = $true
    }

    Push-Location (Join-Path $repoRoot $t.SimDir)
    $prevEAP = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    if ($t.Args -ne "") {
        $simOut = & vvp $outFile $t.Args 2>&1 | ForEach-Object { "$_" }
    } else {
        $simOut = & vvp $outFile 2>&1 | ForEach-Object { "$_" }
    }
    $vvpCode = $LASTEXITCODE
    $ErrorActionPreference = $prevEAP
    Pop-Location

    $simText = $simOut | Out-String
    $logFile = Join-Path $buildDir "$($t.Name).log"
    Set-Content -Path $logFile -Value $simText

    if (($vvpCode -eq 0) -and ($simText -match "TB_RESULT:\s*PASS")) {
        Write-Host "$($t.Name): PASS"
        $passed++
        if ($t.Name -eq "tb_system_top") {
            Push-Location (Join-Path $repoRoot $t.SimDir)
            python ..\model\orb_system_golden.py plot --dir . | Out-Null
            Pop-Location
        }
    } else {
        Write-Host "$($t.Name): FAIL"
        $simOut | Select-Object -Last 15 | ForEach-Object { Write-Host "  $_" }
        $failed++
    }
}

# Run PS backend verification (depends on frame0_rtl.bin produced by tb_record_writer)
$prevEAP = $ErrorActionPreference
$ErrorActionPreference = "Continue"
$beOut = & python sw\host\test_backend.py 2>&1 | ForEach-Object { "$_" }
$beCode = $LASTEXITCODE
$ErrorActionPreference = $prevEAP
$beText = $beOut | Out-String
Set-Content -Path (Join-Path $buildDir "test_backend.log") -Value $beText
if (($beCode -eq 0) -and ($beText -match "BACKEND_TEST:\s*PASS")) {
    Write-Host "test_backend: PASS"
    $passed++
} else {
    Write-Host "test_backend: FAIL"
    $beOut | Select-Object -Last 15 | ForEach-Object { Write-Host "  $_" }
    $failed++
}

$total = $passed + $failed
Write-Host "TOTAL: $passed/$total PASSED, $failed FAILED"
if ($failed -gt 0) { exit 1 } else { exit 0 }
