# Expose the local MT4 HTTP MCP server through ngrok.
#
# Prereqs (one-time):
#   1. pip install -r requirements.txt
#   2. ngrok config add-authtoken <your-token>   # from https://dashboard.ngrok.com/get-started/your-authtoken
#
# Usage:
#   # Terminal 1
#   python -m mt4_mcp
#
#   # Terminal 2
#   .\scripts\start_ngrok.ps1
#
# ChatGPT / remote MCP clients should use:
#   https://<your-ngrok-host>/mcp

param(
    [int]$Port = 8787,
    [string]$Domain = $env:MT4_NGROK_DOMAIN
)

$ErrorActionPreference = "Stop"

if (-not $Domain) {
    $envFile = Join-Path (Split-Path $PSScriptRoot -Parent) ".env"
    if (Test-Path $envFile) {
        Select-String -Path $envFile -Pattern '^\s*MT4_NGROK_DOMAIN\s*=\s*(.+)\s*$' | ForEach-Object {
            $Domain = $_.Matches[0].Groups[1].Value.Trim()
        }
    }
}

if (-not (Get-Command ngrok -ErrorAction SilentlyContinue)) {
    Write-Error "ngrok not found on PATH. Install with: winget install Ngrok.Ngrok"
}

# Winget ships an old ngrok; free accounts require agent >= 3.20.0.
$ngrokVersion = (ngrok version 2>&1 | Select-String -Pattern '(\d+\.\d+\.\d+)' | ForEach-Object { $_.Matches[0].Value })
if ($ngrokVersion -and ([version]$ngrokVersion -lt [version]'3.20.0')) {
    Write-Host "ngrok $ngrokVersion is too old for free accounts; updating..."
    ngrok update
}

$listener = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue
if (-not $listener) {
    Write-Error "Nothing listening on port $Port. Start the MCP server first:`n  `$env:MT4_MCP_PUBLIC = '1'; python -m mt4_mcp"
}

if (-not $Domain) {
    Write-Warning @"
MT4_NGROK_DOMAIN is not set. ngrok may assign a random URL each restart.

On the free plan you get one fixed dev domain (Gateway -> Domains in the ngrok
dashboard). Set it once, e.g.:

  `$env:MT4_NGROK_DOMAIN = 'your-name.ngrok-free.dev'

Then put the same host in MT4_OAUTH_BASE_URL and your Google redirect URI.
"@
    $ngrokArgs = @("http", $Port)
} else {
    if ($Domain -notmatch '^https?://') {
        $Domain = "https://$Domain"
    }
    Write-Host "Using fixed dev domain: $Domain"
    $ngrokArgs = @("http", $Port, "--url", $Domain)
}

Write-Host "Forwarding local port $Port -> public HTTPS (MCP path /mcp)"
Write-Host ""
Write-Host "WARNING: this exposes robot control to the internet when OAuth is off."
Write-Host "Stop ngrok when done. Do not share the URL publicly."
Write-Host ""

& ngrok @ngrokArgs
