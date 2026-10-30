// Browser-player origins only. UE streamer connections stay on loopback and
// are deliberately not required to send a browser Origin header.
export function parseAllowedOrigins(raw = "[]") {
  const values = JSON.parse(raw || "[]");
  if (!Array.isArray(values)) throw new Error("PS_ALLOWED_ORIGINS must be a JSON array");
  for (const value of values) {
    if (typeof value !== "string") throw new Error("Invalid allowed origin");
    const url = new URL(value);
    if (!["http:", "https:"].includes(url.protocol) || url.origin !== value || value.includes("*")) {
      throw new Error("Allowed origins must be exact scheme://host[:port] values");
    }
  }
  return new Set(values);
}

export function isAllowedOrigin(origin, allowedOrigins) {
  return allowedOrigins.size === 0 || allowedOrigins.has(origin);
}
