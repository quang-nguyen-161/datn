# VitalSync — Health Monitoring Dashboard

A real-time health monitoring dashboard built with **Next.js**, fetching live
sensor data from **ThingsBoard Cloud**. Designed for Vercel deployment.

---

## Features

- **8 vital metrics**: Heart Rate, SpO₂, Temperature, Blood Pressure (Systolic/Diastolic), Respiratory Rate, Glucose, Steps
- **Live auto-refresh** every 10 seconds
- **Trend charts** with 1H / 6H / 24H history windows
- **Alert detection**: cards glow red and show ALERT badge when values are critical
- **Patient info** pulled from ThingsBoard device attributes
- **Fully responsive** (mobile-friendly)

---

## ThingsBoard Setup

### 1. Create a Device in ThingsBoard Cloud

1. Log in to [thingsboard.cloud](https://thingsboard.cloud)
2. Go to **Devices** → **+** → **Add new device**
3. Name it (e.g. `PatientMonitor_01`)
4. Copy the **Device ID** from the URL after clicking the device

### 2. Telemetry Keys Expected

Your device should publish telemetry with these **exact keys**:

| Key                | Description                          | Example Value |
|--------------------|--------------------------------------|---------------|
| `ecgHeartRate`     | Heart rate derived from ECG signal   | `72`          |
| `ppgHeartRate`     | Heart rate derived from PPG signal   | `71`          |
| `spo2`             | Oxygen saturation (%)                | `98.5`        |
| `temperature`      | Body temperature (°C)                | `36.6`        |
| `systolic`         | Systolic BP (mmHg)                   | `118`         |
| `diastolic`        | Diastolic BP (mmHg)                  | `76`          |
| `respiratoryRate`  | Breaths per minute                   | `16`          |
| `glucose`          | Blood glucose (mg/dL)                | `95`          |
| `steps`            | Daily step count                     | `4320`        |

You can push test data via the ThingsBoard HTTP API:
```bash
curl -X POST https://thingsboard.cloud/api/v1/YOUR_DEVICE_TOKEN/telemetry \
  -H "Content-Type: application/json" \
  -d '{"ecgHeartRate":72,"ppgHeartRate":71,"spo2":98,"temperature":36.6}'
```

Find `YOUR_DEVICE_TOKEN` under **Device** → **Manage credentials**.

### 3. Patient Attributes (Optional)

Set these **Server-scope attributes** on your device for the patient info bar:

| Key           | Example Value     |
|---------------|-------------------|
| `patientName` | `Nguyen Van A`    |
| `patientId`   | `PT-2024-001`     |
| `ward`        | `Cardiology`      |
| `physician`   | `Dr. Tran Thi B`  |
| `age`         | `45`              |
| `gender`      | `Male`            |
| `bloodType`   | `O+`              |
| `weight`      | `68`              |

---

## Local Development

```bash
# 1. Install dependencies
npm install

# 2. Configure environment variables
cp .env.example .env.local
# Edit .env.local with your ThingsBoard credentials

# 3. Run dev server
npm run dev
# Open http://localhost:3000
```

---

## Deploy to Vercel

```bash
# Option A: Vercel CLI
npm i -g vercel
vercel login
vercel --prod

# Option B: GitHub integration
# Push to GitHub, then import project at vercel.com/new
```

**Add environment variables in Vercel:**
- `TB_BASE_URL` = `https://thingsboard.cloud`
- `TB_USERNAME` = your ThingsBoard email
- `TB_PASSWORD` = your ThingsBoard password
- `TB_DEVICE_ID` = your device UUID

---

## Project Structure

```
health-monitor/
├── pages/
│   ├── index.js               # Main dashboard UI
│   ├── _app.js                # Global app wrapper
│   └── api/
│       ├── telemetry/
│       │   ├── latest.js      # GET latest vital values
│       │   └── history.js     # GET time-series history
│       └── patient.js         # GET device attributes (patient info)
├── components/
│   ├── VitalCard.js           # Individual metric card
│   └── TrendChart.js          # Recharts area chart
├── lib/
│   └── thingsboard.js         # ThingsBoard API helper + token cache
├── styles/
│   └── globals.css            # Full design system CSS
└── .env.example               # Environment variable template
```

---

## Customization

**Add a new vital metric:**
1. Have your device publish a new telemetry key (e.g. `ecgSignal`)
2. Add it to the `keys` array in `pages/api/telemetry/latest.js`
3. Add a new entry to the `VITALS` array in `pages/index.js`

**Change refresh interval:**  
Edit the `setInterval` call in `pages/index.js` (default: 10000 ms)

**Change alert thresholds:**  
Edit `critMin` and `critMax` in the `VITALS` array in `pages/index.js`
