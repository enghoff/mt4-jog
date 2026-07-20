# OAuth 2.1 for ChatGPT via Google + ngrok

The public HTTP MCP server can expose a ChatGPT-compatible OAuth 2.1 flow using
FastMCP's built-in Google provider. FastMCP acts as the authorization server
(DCR + PKCE + protected-resource metadata); users sign in with Google.

## 1. Google Cloud OAuth client

1. Open [Google Cloud Console](https://console.cloud.google.com/) → APIs & Services → Credentials.
2. Create an **OAuth 2.0 Client ID** → **Web application**.
3. Add **Authorized redirect URI** (replace with your ngrok host):

   ```
   https://YOUR-SUBDOMAIN.ngrok-free.dev/auth/callback
   ```

4. Copy the client ID and secret.

## 2. Environment

```powershell
copy .env.example .env
# Edit .env with your ngrok URL and Google credentials
```

Required variables:

| Variable | Example |
|----------|---------|
| `MT4_MCP_PUBLIC` | `1` |
| `MT4_OAUTH_ENABLED` | `1` |
| `MT4_OAUTH_BASE_URL` | `https://abc123.ngrok-free.dev` |
| `MT4_GOOGLE_CLIENT_ID` | `....apps.googleusercontent.com` |
| `MT4_GOOGLE_CLIENT_SECRET` | `GOCSPX-...` |

`MT4_OAUTH_BASE_URL` must match your current ngrok HTTPS origin exactly (no `/mcp` suffix).

## 3. Start services

Find your **assigned dev domain** in the [ngrok dashboard](https://dashboard.ngrok.com/domains)
(Gateway → Domains). On the free plan you get one fixed `*.ngrok-free.dev` URL that
does not change between restarts.

Terminal 1 — MCP server (loads `.env` automatically):

```powershell
python -m mt4_mcp
```

Terminal 2 — ngrok with your fixed dev domain:

```powershell
$env:MT4_NGROK_DOMAIN = 'your-name.ngrok-free.dev'   # or set in .env
.\scripts\start_ngrok.ps1
```

Set `MT4_OAUTH_BASE_URL` and `MT4_NGROK_DOMAIN` to the same host so OAuth and
ChatGPT keep working after restarts without reconfiguring Google or ChatGPT.

## 4. Verify OAuth metadata

```powershell
curl https://YOUR-SUBDOMAIN.ngrok-free.dev/.well-known/oauth-protected-resource/mcp
curl https://YOUR-SUBDOMAIN.ngrok-free.dev/.well-known/oauth-authorization-server
curl https://YOUR-SUBDOMAIN.ngrok-free.dev/.well-known/openid-configuration
```

The protected-resource metadata is scoped under the MCP path (`/mcp`), not the
root — requesting the root path returns 404 by design (RFC 9728).

## 5. ChatGPT connector

1. ChatGPT → **Settings → Connectors → Advanced → Developer mode** (on)
2. **Create** connector
3. MCP URL: `https://YOUR-SUBDOMAIN.ngrok-free.dev/mcp`
4. Complete the Google sign-in when prompted

## Notes

- `mt4_mcp/auth.py` allowlists ChatGPT, Claude.ai/Claude.com hosted callbacks,
  Cursor web/deeplink callbacks, and local loopback
  (`http://localhost:*/callback` for Claude Code / Cursor desktop). Missing a
  client pattern surfaces as `Redirect URI '…' does not match allowed patterns`.
- OAuth applies to the **HTTP** server only. Cursor's stdio MCP (`--stdio`) stays
  local and does not use these settings.
- Only one process can hold `COM6`. Stop `jog.py` and Cursor's MT4 MCP
  before running the public HTTP server.
- Free ngrok URLs rotate on restart — update `.env`, Google redirect URI, and the
  ChatGPT connector when that happens.
- `mt4_mcp/auth.py` monkeypatches two FastMCP 3.4.3 bugs on import:
  1. A double-slash `...//token` URL in the CIMD private-key-JWT client
     assertion (breaks ChatGPT's dynamic client token exchange).
  2. A missing `/.well-known/openid-configuration` route: FastMCP computes
     this OIDC discovery alias in `get_well_known_routes()`, but the HTTP
     app only mounts `get_routes()`, so the alias is never actually served
     and ChatGPT's post-token-exchange discovery request 404s. Re-check
     these patches when upgrading `fastmcp`.
