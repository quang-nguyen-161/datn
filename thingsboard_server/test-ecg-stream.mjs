// ECG streaming test — simulates ESP32 firmware sending real-time ECG to TB
// Matches firmware behavior: ecgSampleFreq=250Hz, ecgPacketInterval=200ms (50 samples/batch)
// Sends via HTTP (the real path: ESP32 → Vercel → TB)
// Receives via TB WebSocket API (the real path: TB → browser dashboard)

import WebSocket from "ws";

// ── Config ──────────────────────────────────────────────────────────────────
const TB_LOCAL        = "http://localhost:9090";
const TB_WS           = "ws://localhost:9090";
const DEVICE_TOKEN    = "xzJfJY1RAwAKARPV1NDR";
const DEVICE_ID       = "bca27110-5f54-11f1-85bc-217f640bf259";
const ECG_SAMPLE_FREQ = 250;           // Hz — matches firmware default
const PACKET_INTERVAL = 200;          // ms  — matches ecgPacketInterval
const SAMPLES_PER_PKT = Math.round(ECG_SAMPLE_FREQ * PACKET_INTERVAL / 1000); // 50
const TEST_DURATION   = 15000;        // ms total test time
const TOTAL_PACKETS   = Math.round(TEST_DURATION / PACKET_INTERVAL);

// ── Synthetic ECG generator ──────────────────────────────────────────────────
// Simple PQRST-like waveform using sum of sinusoids
function generateEcgBatch(batchIndex) {
  const samples = [];
  for (let i = 0; i < SAMPLES_PER_PKT; i++) {
    const t = (batchIndex * SAMPLES_PER_PKT + i) / ECG_SAMPLE_FREQ;
    const hr = 75; // bpm
    const phase = (t * hr / 60) % 1;

    let v = 0;
    // P wave
    if (phase > 0.1 && phase < 0.3)  v += 0.15 * Math.sin(Math.PI * (phase - 0.1) / 0.2);
    // QRS complex
    if (phase > 0.4 && phase < 0.42) v -= 0.1 * Math.sin(Math.PI * (phase - 0.4) / 0.02);
    if (phase > 0.42 && phase < 0.46) v += 1.0 * Math.sin(Math.PI * (phase - 0.42) / 0.04);
    if (phase > 0.46 && phase < 0.5)  v -= 0.2 * Math.sin(Math.PI * (phase - 0.46) / 0.04);
    // T wave
    if (phase > 0.55 && phase < 0.75) v += 0.25 * Math.sin(Math.PI * (phase - 0.55) / 0.2);
    // Baseline noise
    v += (Math.random() - 0.5) * 0.02;

    samples.push(Math.round(2048 + v * 400)); // 12-bit ADC centered at 2048
  }
  return samples;
}

// ── Stats tracking ───────────────────────────────────────────────────────────
const sentPackets    = [];   // { index, sentAt }
const receivedBatches = [];  // { index, sentAt, receivedAt, latency }
let   missedPackets  = 0;

// ── WebSocket subscription ───────────────────────────────────────────────────
async function getJwt() {
  const res = await fetch(`${TB_LOCAL}/api/auth/login`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ username: "tenant@thingsboard.org", password: "tenant" }),
  });
  return (await res.json()).token;
}

const jwt = await getJwt();
const ws  = new WebSocket(`${TB_WS}/api/ws/plugins/telemetry?token=${jwt}`);

await new Promise((resolve, reject) => {
  ws.on("open", () => {
    ws.send(JSON.stringify({
      tsSubCmds: [{ entityType: "DEVICE", entityId: DEVICE_ID, scope: "LATEST_TELEMETRY", cmdId: 1 }],
      historyCmds: [], attrSubCmds: [],
    }));
    resolve();
  });
  ws.on("error", reject);
  setTimeout(() => reject(new Error("WS connect timeout")), 8000);
});

console.log(`ECG Stream Test`);
console.log(`  Sample freq  : ${ECG_SAMPLE_FREQ} Hz`);
console.log(`  Batch size   : ${SAMPLES_PER_PKT} samples`);
console.log(`  Packet rate  : 1 packet / ${PACKET_INTERVAL}ms`);
console.log(`  Duration     : ${TEST_DURATION / 1000}s  (${TOTAL_PACKETS} packets)`);
console.log(`  Total samples: ${TOTAL_PACKETS * SAMPLES_PER_PKT}`);
console.log(`\nStarting stream...\n`);

// Map sent packet index via a timestamp watermark embedded in the batch
const pendingSends = new Map(); // batchIndex → sentAt

ws.on("message", (raw) => {
  const msg  = JSON.parse(raw);
  const data = msg?.data;
  if (!data?.ecg_batch) return;

  const receivedAt = performance.now();
  // ecg_batch value is [[ts, jsonString]] — decode to find batchIndex
  const batchStr = Array.isArray(data.ecg_batch)
    ? data.ecg_batch[0][1]
    : data.ecg_batch;

  try {
    const batch = JSON.parse(batchStr);
    const marker = batch?.[0]; // first sample encodes batchIndex * 10000 + 9999
    if (marker && marker > 9999) {
      const batchIndex = Math.floor(marker / 10000);
      const sentEntry  = pendingSends.get(batchIndex);
      if (sentEntry) {
        const latency = receivedAt - sentEntry;
        receivedBatches.push({ batchIndex, latency });
        pendingSends.delete(batchIndex);
      }
    }
  } catch {}
});

// ── Send loop ─────────────────────────────────────────────────────────────────
const startTime = performance.now();
let   batchIndex = 0;

while (batchIndex < TOTAL_PACKETS) {
  const samples   = generateEcgBatch(batchIndex);
  // Embed batchIndex as a watermark in the first sample
  samples[0]      = batchIndex * 10000 + 9999;
  const payload   = JSON.stringify({ ecg_batch: JSON.stringify(samples) });
  const sentAt    = performance.now();

  pendingSends.set(batchIndex, sentAt);
  sentPackets.push({ batchIndex, sentAt });

  fetch(`${TB_LOCAL}/api/v1/${DEVICE_TOKEN}/telemetry`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: payload,
  }).catch(() => { pendingSends.delete(batchIndex); missedPackets++; });

  batchIndex++;
  const elapsed  = performance.now() - startTime;
  const nextSend = batchIndex * PACKET_INTERVAL;
  const waitMs   = nextSend - elapsed;
  if (waitMs > 0) await new Promise(r => setTimeout(r, waitMs));

  // Live progress
  if (batchIndex % 5 === 0) {
    const received = receivedBatches.length;
    const recent   = receivedBatches.slice(-5).map(r => r.latency.toFixed(0)).join(" ");
    process.stdout.write(`\r  Sent: ${batchIndex}/${TOTAL_PACKETS}  Recv: ${received}  Recent latency: [${recent}] ms   `);
  }
}

// Wait up to 5s for in-flight packets
console.log("\n\nWaiting for last packets...");
await new Promise(r => setTimeout(r, 5000));
ws.close();

// ── Report ────────────────────────────────────────────────────────────────────
const totalSent     = sentPackets.length;
const totalReceived = receivedBatches.length;
const dropRate      = ((totalSent - totalReceived) / totalSent * 100).toFixed(1);
const latencies     = receivedBatches.map(r => r.latency).sort((a, b) => a - b);
const avg           = latencies.reduce((a, b) => a + b, 0) / latencies.length;
const p50           = latencies[Math.floor(latencies.length * 0.50)];
const p95           = latencies[Math.floor(latencies.length * 0.95)];
const p99           = latencies[Math.floor(latencies.length * 0.99)];

console.log(`\n${"═".repeat(55)}`);
console.log(` ECG STREAMING RESULTS`);
console.log(`${"═".repeat(55)}`);
console.log(` Packets sent     : ${totalSent}`);
console.log(` Packets received : ${totalReceived}  (${dropRate}% dropped)`);
console.log(` Samples/sec sent : ${(totalSent * SAMPLES_PER_PKT / (TEST_DURATION / 1000)).toFixed(0)} samples/s`);
console.log(`─${"─".repeat(54)}`);
console.log(` WS push latency  (HTTP ingest → WS received)`);
console.log(`   Min   : ${latencies[0].toFixed(1)} ms`);
console.log(`   Avg   : ${avg.toFixed(1)} ms`);
console.log(`   Median: ${p50.toFixed(1)} ms`);
console.log(`   P95   : ${p95.toFixed(1)} ms`);
console.log(`   P99   : ${p99.toFixed(1)} ms`);
console.log(`   Max   : ${latencies[latencies.length - 1].toFixed(1)} ms`);
console.log(`─${"─".repeat(54)}`);
const waveformLag = avg + (PACKET_INTERVAL / 2);
console.log(` Est. dashboard waveform lag : ~${waveformLag.toFixed(0)} ms`);
console.log(` (pipeline latency + half a packet interval)`);
console.log(`${"═".repeat(55)}`);
