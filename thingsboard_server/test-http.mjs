// HTTP telemetry latency test — ThingsBoard via Cloudflare tunnel

const TB_URL     = "https://midnight-pair-periodic-handy.trycloudflare.com";
const TOKEN      = "xzJfJY1RAwAKARPV1NDR";
const ITERATIONS = 50;
const ENDPOINT   = `${TB_URL}/api/v1/${TOKEN}/telemetry`;

const results = [];

console.log(`HTTP latency test — ${ITERATIONS} iterations`);
console.log(`Endpoint: ${ENDPOINT}\n`);

for (let i = 0; i < ITERATIONS; i++) {
  const payload = JSON.stringify({ ts: Date.now(), values: { test_val: i } });

  const t0 = performance.now();
  try {
    const res = await fetch(ENDPOINT, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: payload,
    });
    const latency = performance.now() - t0;

    if (res.ok) {
      results.push(latency);
      process.stdout.write(`\r  [${i + 1}/${ITERATIONS}] ${latency.toFixed(1)} ms`);
    } else {
      console.log(`\n  [${i + 1}] HTTP ${res.status}`);
    }
  } catch (e) {
    console.log(`\n  [${i + 1}] ERROR: ${e.message}`);
  }

  // 200ms gap between requests
  await new Promise(r => setTimeout(r, 200));
}

printStats("HTTP", results);

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
