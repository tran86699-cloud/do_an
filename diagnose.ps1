# QUICK DIAGNOSTIC - RUN THIS FIRST!
# Save as: diagnose.ps1
# Run: .\diagnose.ps1

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "QUICK DIAGNOSTIC" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

# 1. Check line 192
Write-Host "[1] Checking pc_server_vision_windows.py line 192:" -ForegroundColor Yellow
$line192 = (Get-Content "C:\raspbot_server\pc_server_vision_windows.py")[191]
Write-Host "    $line192" -ForegroundColor White

if ($line192 -match "timeout=60") {
    Write-Host "    ✅ CORRECT: timeout=60" -ForegroundColor Green
} else {
    Write-Host "    ❌ WRONG: Still timeout=30!" -ForegroundColor Red
    Write-Host "    → Need to edit file!" -ForegroundColor Yellow
}

Write-Host ""

# 2. Check running processes
Write-Host "[2] Checking running processes:" -ForegroundColor Yellow

$pythons = Get-Process python -ErrorAction SilentlyContinue
if ($pythons) {
    Write-Host "    Python processes:" -ForegroundColor White
    $pythons | ForEach-Object {
        Write-Host "      PID $($_.Id): $($_.Path)" -ForegroundColor Gray
    }
} else {
    Write-Host "    ✅ No Python running" -ForegroundColor Green
}

Write-Host ""

$ollamas = Get-Process ollama* -ErrorAction SilentlyContinue
if ($ollamas) {
    Write-Host "    Ollama processes:" -ForegroundColor White
    $ollamas | ForEach-Object {
        Write-Host "      PID $($_.Id): $($_.ProcessName)" -ForegroundColor Gray
    }
    
    if ($ollamas.Count -gt 2) {
        Write-Host "    ⚠️  WARNING: Multiple Ollama processes!" -ForegroundColor Yellow
        Write-Host "    → This can cause GPU conflicts!" -ForegroundColor Yellow
    }
} else {
    Write-Host "    ✅ No Ollama running" -ForegroundColor Green
}

Write-Host ""

# 3. Test Ollama if running
if ($ollamas) {
    Write-Host "[3] Testing Ollama speed:" -ForegroundColor Yellow
    Write-Host "    Running: ollama run qwen2.5vl:7b 'hello'" -ForegroundColor Gray
    
    $time = Measure-Command {
        ollama run qwen2.5vl:7b "hello" | Out-Null
    }
    
    $seconds = [math]::Round($time.TotalSeconds, 1)
    Write-Host "    Time: $seconds seconds" -ForegroundColor White
    
    if ($seconds -lt 10) {
        Write-Host "    ✅ FAST: GPU working!" -ForegroundColor Green
    } else {
        Write-Host "    ❌ SLOW: GPU not working!" -ForegroundColor Red
    }
} else {
    Write-Host "[3] Ollama not running - skip test" -ForegroundColor Gray
}

Write-Host ""

# 4. Summary
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "SUMMARY:" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

if ($line192 -match "timeout=60") {
    Write-Host "✅ File has timeout=60" -ForegroundColor Green
} else {
    Write-Host "❌ File still has timeout=30 - EDIT NEEDED!" -ForegroundColor Red
}

if ($ollamas -and $ollamas.Count -gt 2) {
    Write-Host "⚠️  Multiple Ollama processes - STOP NEEDED!" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "Press Enter to continue..." -ForegroundColor Gray
Read-Host