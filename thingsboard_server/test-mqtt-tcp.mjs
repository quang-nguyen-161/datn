// MQTT TCP latency test — direct port 1883 (local only, not through tunnel)

import mqtt from "mqtt";

const TOKEN      = "xzJfJY1RAwAKARPV1NDR";
const ITERATIONS = 50;
const TOPIC      = "v1/devices/me/telemetry";
const results    = [];

console.log(`MQTT TCP latency test — ${ITERATIONS} iterations`);
console.log(`Broker: mqtt://localhost:1883\n`);

const timeout = setTimeout(() => {
  console.error("Connection timed out after 10s");
  process.exit(1);
}, 10000);

const client = mqtt.connect("mqtt://localhost:1883", {
  username:        TOKEN,
  password:        "",
  protocolVersion: 4,
  reconnectPeriod: 0,
});

client.on("error",  (e) => { console.error("Error:", e.message); process.exit(1); });

client.on("connect", async () => {
  clearTimeout(timeout);
  console.log("  Connected\n");

  for (let i = 0; i < ITERATIONS; i++) {
    const payload = JSON.stringify({ ts: Date.now(), values: { test_val: i } });

    await new Promise((resolve) => {
      const t0 = performance.now();
      client.publish(TOPIC, payload, { qos: 1 }, (err) => {
        const latency = performance.now() - t0;
        if (err) {
          console.log(`\n  [${i + 1}] ERROR: ${err.message}`);
        } else {
          results.push(latency);
          process.stdout.write(`\r  [${i + 1}/${ITERATIONS}] ${latency.toFixed(1)} ms`);
        }
        resolve();
      });
    });

    await new Promise(r => setTimeout(r, 200));
  }

  client.end();
  printStats("MQTT TCP (local)", results);
});

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
