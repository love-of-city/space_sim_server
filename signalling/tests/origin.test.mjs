import test from "node:test";
import assert from "node:assert/strict";
import {parseAllowedOrigins, isAllowedOrigin} from "../origin.mjs";

test("configured player origin allowlist is exact and rejects missing/opaque origins", () => {
  const origins = parseAllowedOrigins('["https://sim.test","https://sim.test:8443"]');
  assert.equal(isAllowedOrigin("https://sim.test", origins), true);
  assert.equal(isAllowedOrigin("https://sim.test:8443", origins), true);
  for (const value of [undefined, "null", "https://sim.test.evil.test", "http://sim.test", "https://sim.test/"]) {
    assert.equal(isAllowedOrigin(value, origins), false);
  }
});
test("local-mode empty allowlist remains compatible", () => {
  assert.equal(isAllowedOrigin(undefined, parseAllowedOrigins()), true);
});
test("misconfigured origins fail closed at startup", () => {
  for (const raw of ['{}', '[null]', '["*"]', '["https://*.test"]', '["https://sim.test/"]', '["https://user@sim.test"]', '["ws://sim.test"]']) {
    assert.throws(() => parseAllowedOrigins(raw));
  }
});
