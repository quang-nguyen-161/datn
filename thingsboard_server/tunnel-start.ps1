# Starts Cloudflare quick tunnel, updates Vercel env vars, triggers redeploy

$CLOUDFLARED     = "D:\cloudflared.exe"
$VERCEL_TOKEN    = $env:VERCEL_TOKEN
$PROJECT_ID      = "prj_MeU6nk1DzZt9YZfExNVHk3ezfnRs"

if (-not $VERCEL_TOKEN) {
    Write-Host "ERROR: Set the VERCEL_TOKEN environment variable before running this script." -ForegroundColor Red
    exit 1
}
$LAST_DEPLOY_ID  = "dpl_BgApDU46Y5pqzymkqKjs2SKQ1WVz"

# Env var IDs
$ID_TB_BASE_URL             = "f1pMqMuAFAIUP6ij"
$ID_NEXT_PUBLIC_TB_BASE_URL = "pXvEDKHD5XyqA9MV"
$ID_NEXT_PUBLIC_TB_WS_URL   = "kDcn4Ph0Y9xHap41"

$headers = @{ Authorization = "Bearer $VERCEL_TOKEN"; "Content-Type" = "application/json" }
$logFile = "$env:TEMP\cloudflared-tunnel.log"

# Kill any existing cloudflared
Get-Process -Name "cloudflared" -ErrorAction SilentlyContinue | Stop-Process -Force
Remove-Item $logFile -ErrorAction SilentlyContinue

Write-Host "Starting Cloudflare tunnel..." -ForegroundColor Cyan
Start-Process -FilePath $CLOUDFLARED `
    -ArgumentList "tunnel --url http://localhost:9090" `
    -RedirectStandardError $logFile `
    -NoNewWindow

# Wait for URL to appear in log
Write-Host "Waiting for tunnel URL..." -ForegroundColor Yellow
$url = $null
$attempts = 0
while (-not $url -and $attempts -lt 30) {
    Start-Sleep -Seconds 2
    $attempts++
    if (Test-Path $logFile) {
        $match = Select-String -Path $logFile -Pattern "https://[a-z0-9\-]+\.trycloudflare\.com" | Select-Object -First 1
        if ($match) {
            $url = $match.Matches[0].Value
        }
    }
}

if (-not $url) {
    Write-Host "ERROR: Could not get tunnel URL after 60s" -ForegroundColor Red
    exit 1
}

$wsUrl = $url -replace "^https://", "wss://"
Write-Host ""
Write-Host "Tunnel URL: $url" -ForegroundColor Green
Write-Host "WebSocket:  $wsUrl" -ForegroundColor Green

# Update Vercel env vars
Write-Host ""
Write-Host "Updating Vercel env vars..." -ForegroundColor Cyan

$pairs = @(
    @{ id = $ID_TB_BASE_URL;             value = $url   },
    @{ id = $ID_NEXT_PUBLIC_TB_BASE_URL; value = $url   },
    @{ id = $ID_NEXT_PUBLIC_TB_WS_URL;   value = $wsUrl }
)

foreach ($pair in $pairs) {
    $body = @{ value = $pair.value } | ConvertTo-Json
    try {
        Invoke-RestMethod -Method Patch `
            -Uri "https://api.vercel.com/v9/projects/$PROJECT_ID/env/$($pair.id)" `
            -Headers $headers `
            -Body $body | Out-Null
        Write-Host "  Updated $($pair.id)" -ForegroundColor Gray
    } catch {
        Write-Host "  ERROR updating $($pair.id): $_" -ForegroundColor Red
    }
}

# Trigger redeploy
Write-Host ""
Write-Host "Triggering Vercel redeploy..." -ForegroundColor Cyan
$redeployBody = @{ deploymentId = $LAST_DEPLOY_ID; target = "production"; name = "wearable-server" } | ConvertTo-Json
try {
    $deploy = Invoke-RestMethod -Method Post `
        -Uri "https://api.vercel.com/v13/deployments" `
        -Headers $headers `
        -Body $redeployBody
    Write-Host "Redeploy triggered: https://$($deploy.url)" -ForegroundColor Green

    # Save new deployment ID for next run
    (Get-Content $PSCommandPath) -replace 'dpl_[A-Za-z0-9]+', $deploy.id | Set-Content $PSCommandPath
    Write-Host "Saved new deployment ID for next run" -ForegroundColor Gray
} catch {
    Write-Host "ERROR triggering redeploy: $_" -ForegroundColor Red
}

Write-Host ""
Write-Host "Done! Vercel will be live in ~1 min at:" -ForegroundColor Green
Write-Host "  https://wearable-server.vercel.app" -ForegroundColor White
Write-Host ""
Write-Host "ThingsBoard: $url" -ForegroundColor White
Write-Host "Press Ctrl+C to stop the tunnel" -ForegroundColor Yellow

# Keep script alive so tunnel stays up
while ($true) { Start-Sleep -Seconds 60 }
