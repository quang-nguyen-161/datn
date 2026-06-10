# Start ThingsBoard CE
Write-Host "Starting ThingsBoard CE..." -ForegroundColor Cyan

docker compose up -d

Write-Host ""
Write-Host "Waiting for ThingsBoard to initialize (~2 min)..." -ForegroundColor Yellow
Write-Host "Check status: docker compose logs -f thingsboard"
Write-Host ""
Write-Host "Once ready, open: http://localhost:9090" -ForegroundColor Green
Write-Host "  Login: tenant@thingsboard.org"
Write-Host "  Pass:  tenant"
