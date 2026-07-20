"""OAuth 2.1 setup for the public MT4 MCP HTTP server."""



from __future__ import annotations



import os

import re

import secrets



from fastmcp.server.auth import auth as fastmcp_auth

from fastmcp.server.auth.providers.google import GoogleProvider



# MCP clients register their own callback URLs during DCR / authorize.
# ChatGPT / Claude.ai / Cursor each use fixed hosted callbacks; Claude Code
# and Cursor desktop use loopback (any port). FastMCP matches with wildcards
# (see fastmcp.server.auth.redirect_validation).
ALLOWED_CLIENT_REDIRECT_PATTERNS = [
    "https://chatgpt.com/connector/oauth/*",
    "https://chatgpt.com/connector_platform_oauth_redirect",
    "https://claude.ai/api/mcp/auth_callback",
    "https://claude.com/api/mcp/auth_callback",
    "http://localhost:*/callback",
    "http://127.0.0.1:*/callback",
    "https://www.cursor.com/agents/mcp/oauth/callback",
    "cursor://anysphere.cursor-mcp/oauth/callback",
]



MT4_OAUTH_SCOPES = ["openid", "email"]



_TOKEN_ENDPOINT_RE = re.compile(r"^(https?://[^/]+)//token$")





def _patch_fastmcp_cimd_token_endpoint() -> None:

    """Work around FastMCP CIMD token-endpoint URL double-slash bug.



    Pydantic root ``AnyHttpUrl`` values stringify with a trailing slash, so

    FastMCP builds ``...//token`` while OAuth metadata advertises ``.../token``.

    ChatGPT's CIMD client assertion uses the metadata value and fails validation.

    """

    if getattr(fastmcp_auth.PrivateKeyJWTClientAuthenticator, "_mt4_patch", False):

        return



    original_init = fastmcp_auth.PrivateKeyJWTClientAuthenticator.__init__



    def __init__(self, provider, cimd_manager, token_endpoint_url: str):

        match = _TOKEN_ENDPOINT_RE.match(token_endpoint_url)

        if match:

            token_endpoint_url = f"{match.group(1)}/token"

        original_init(self, provider, cimd_manager, token_endpoint_url)



    fastmcp_auth.PrivateKeyJWTClientAuthenticator.__init__ = __init__

    fastmcp_auth.PrivateKeyJWTClientAuthenticator._mt4_patch = True




def _patch_fastmcp_openid_configuration_alias() -> None:
    """Work around FastMCP never mounting the OIDC discovery alias.

    ``OAuthProvider.get_well_known_routes`` (fastmcp/server/auth/auth.py)
    computes a ``/.well-known/openid-configuration`` alias for the
    authorization-server metadata, but the HTTP app builder
    (fastmcp/server/http.py) only mounts whatever ``auth.get_routes()``
    returns -- it never calls ``get_well_known_routes()``. So the alias is
    computed but never actually served, and requests to it 404.

    ChatGPT's connector probes exactly this path right after completing the
    OAuth token exchange, so the flow visibly "works" (consent, callback,
    token 200 OK) but the connector still reports failure because of this
    trailing 404. Add the alias route ourselves, mirroring whatever handler
    ends up serving ``/.well-known/oauth-authorization-server`` (including
    FastMCP's CIMD-aware metadata handler).
    """

    from fastmcp.server.auth.oauth_proxy import proxy as fastmcp_proxy

    if getattr(fastmcp_proxy.OAuthProxy, "_mt4_openid_alias_patch", False):
        return

    original_get_routes = fastmcp_proxy.OAuthProxy.get_routes

    def get_routes(self, mcp_path: str | None = None):
        routes = original_get_routes(self, mcp_path)
        for route in routes:
            if getattr(route, "path", None) == "/.well-known/oauth-authorization-server":
                routes.append(
                    fastmcp_proxy.Route(
                        path="/.well-known/openid-configuration",
                        endpoint=route.endpoint,
                        methods=route.methods,
                    )
                )
                break
        return routes

    fastmcp_proxy.OAuthProxy.get_routes = get_routes
    fastmcp_proxy.OAuthProxy._mt4_openid_alias_patch = True



_patch_fastmcp_cimd_token_endpoint()

_patch_fastmcp_openid_configuration_alias()





def oauth_enabled() -> bool:

    return os.environ.get("MT4_OAUTH_ENABLED", "").lower() in ("1", "true", "yes")





def build_auth_provider() -> GoogleProvider | None:

    """Return a ChatGPT-compatible OAuth provider, or None when disabled."""

    if not oauth_enabled():

        return None



    base_url = os.environ.get("MT4_OAUTH_BASE_URL", "").rstrip("/")

    client_id = os.environ.get("MT4_GOOGLE_CLIENT_ID", "")

    client_secret = os.environ.get("MT4_GOOGLE_CLIENT_SECRET", "")



    if not base_url:

        raise RuntimeError(

            "MT4_OAUTH_ENABLED is set but MT4_OAUTH_BASE_URL is missing. "

            "Set it to your public HTTPS origin, e.g. https://abc123.ngrok-free.dev"

        )

    if not client_id or not client_secret:

        raise RuntimeError(

            "MT4_OAUTH_ENABLED is set but Google OAuth credentials are missing. "

            "Set MT4_GOOGLE_CLIENT_ID and MT4_GOOGLE_CLIENT_SECRET."

        )



    jwt_key = os.environ.get("MT4_OAUTH_JWT_KEY") or secrets.token_urlsafe(32)



    return GoogleProvider(

        client_id=client_id,

        client_secret=client_secret,

        base_url=base_url,

        resource_base_url=base_url,

        issuer_url=base_url,

        required_scopes=MT4_OAUTH_SCOPES,

        allowed_client_redirect_uris=ALLOWED_CLIENT_REDIRECT_PATTERNS,

        jwt_signing_key=jwt_key,

        forward_resource=True,

        enable_cimd=True,

    )


