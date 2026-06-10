// ThingsBoard WebSocket API latency test
// Measures end-to-end: HTTP telemetry POST → WS push received in browser
// This is the real-time latency your dashboard experiences

import WebSocket from "ws";

const TB_LOCAL   = "http://localhost:9090";
const TB_WS      = "ws://localhost:9090";
const DEVICE_ID  = "bca27110-5f54-11f1-85bc-217f640bf259";
const TOKEN_HTTP = "xzJfJY1RAwAKARPV1NDR";   // device access token for HTTP ingest
const ITERATIONS = 50;

// Fresh JWT each run (expires in 2.5h)
async function getJwt() {
  const res  = await fetch(`${TB_LOCAL}/api/auth/login`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ username: "tenant@thingsboard.org", password: "tenant" }),
  });
  return (await res.json()).token;
}

const results = [];
const jwt     = await getJwt();

console.log(`TB WebSocket API latency test — ${ITERATIONS} iterations`);
console.log(`Device: ${DEVICE_ID}`);
console.log(`WS: ${TB_WS}/api/ws/plugins/telemetry\n`);

const wsTimeout = setTimeout(() => {
  console.error("WS connection timed out"); process.exit(1);
}, 10000);

const ws = new WebSocket(`${TB_WS}/api/ws/plugins/telemetry?token=${jwt}`);

ws.on("error",  (e) => { console.error("WS error:", e.message); process.exit(1); });

ws.on("open", () => {
  clearTimeout(wsTimeout);
  console.log("  WebSocket connected");

  // Subscribe to latest telemetry for our device
  ws.send(JSON.stringify({
    tsSubCmds: [{
      entityType:  "DEVICE",
      entityId:    DEVICE_ID,
      scope:       "LATEST_TELEMETRY",
      cmdId:       1,
    }],
    historyCmds:   [],
    attrSubCmds:   [],
  }));
  console.log("  Subscribed — starting test\n");
});

let   pendingResolve = null;
let   sendTime       = 0;
let   iteration      = 0;

ws.on("message", async (raw) => {
  const msg  = JSON.parse(raw);
  const data = msg?.data;

  // Skip subscription confirmation (no data key or empty)
  if (!data || !data.test_val) return;

  if (pendingResolve) {
    const latency = performance.now() - sendTime;
    results.push(latency);
    process.stdout.write(`\r  [${iteration}/${ITERATIONS}] ${latency.toFixed(1)} ms`);
    pendingResolve();
    pendingResolve = null;
  }
});

// Wait for subscription to settle, then run
await new Promise(r => setTimeout(r, 500));

for (let i = 1; i <= ITERATIONS; i++) {
  iteration = i;

  await new Promise((resolve) => {
    pendingResolve = resolve;
    sendTime = performance.now();

    // POST telemetry via device HTTP API — TB stores it and pushes to WS subscribers
    fetch(`${TB_LOCAL}/api/v1/${TOKEN_HTTP}/telemetry`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ test_val: i }),
    }).catch(e => console.error("\n  POST error:", e.message));

    // Failsafe: if WS push doesn't arrive in 5s, move on
    setTimeout(() => { if (pendingResolve) { pendingResolve = null; resolve(); } }, 5000);
  });

  // TB pushes WS updates at most once per ~1s — wait long enough to catch each push
  await new Promise(r => setTimeout(r, 1100));
}

ws.close();
printStats("TB WebSocket API (end-to-end)", results);

function printStats(label, data) {
  if (!data.length) { console.log("\nNo successful results"); return; }
  const sorted = [...data].sort((a, b) => a - b);
  const avg    = data.reduce((a, b) => a + b, 0) / data.length;
  const p50    = sorted[Math.floor(sorted.length * 0.50)];
  const p95    = sorted[Math.floor(sorted.length * 0.95)];
  const p99    = sorted[Math.floor(sorted.length * 0.99)];

  console.log(`\n\n=== ${label} Results (${data.length}/${ITERATIONS} successful) ===`);
  console.log(`  Min    : ${sorted[0].toFixed(1)} ms`);
  console.log(`  Avg    : ${avg.toFixed(1)} ms`);
  console.log(`  Median : ${p50.toFixed(1)} ms`);
  console.log(`  P95    : ${p95.toFixed(1)} ms`);
  console.log(`  P99    : ${p99.toFixed(1)} ms`);
  console.log(`  Max    : ${sorted[sorted.length - 1].toFixed(1)} ms`);
}
