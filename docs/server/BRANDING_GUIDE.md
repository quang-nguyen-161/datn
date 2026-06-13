# VitalSync — Branding & Customization Guide

How to change the app name, logo, colors, and icons.

---

## 1. App Name

**Three places to change:**

### `pages/index.js`
```jsx
// Header brand name
<div className="brand-name">VITALSYNC</div>        // ← change this
<div className="brand-sub">HEALTH MONITORING SYSTEM</div>  // ← and this

// Browser tab title
<title>HealthMonitor — Live Dashboard</title>       // ← and this
```

### `pages/settings.js`
```jsx
<title>Settings — VitalSync</title>   // ← change
```

### `components/PrintModal.js`
```jsx
// Print report header
<div style={{ fontSize:22, fontWeight:700, color:"#00c8ff" }}>VITALSYNC</div>
<div>Health Monitoring System — Patient Report</div>

// Footer
<span>VitalSync Health Monitoring System</span>
```

---

## 2. Logo / Brand Icon

The logo in the header is an inline SVG ECG waveform:

```jsx
// pages/index.js — header brand icon
<svg width="22" height="22" viewBox="0 0 24 24" fill="none">
  <path
    d="M3 12h3l3-9 3 18 3-9h3"   // ← ECG waveform path
    stroke="#00c8ff"               // ← icon color
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
  />
</svg>
```

**To use an image logo instead:**
```jsx
// Replace the <svg> with:
<img src="/logo.png" width={32} height={32} alt="Logo" style={{ borderRadius: 6 }} />
// Place logo.png in the /public/ folder
```

**To use an emoji:**
```jsx
<span style={{ fontSize: 22 }}>🏥</span>
```

---

## 3. Favicon

Change the favicon in `pages/index.js`:
```jsx
// Current: heart emoji SVG favicon
<link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><text y='.9em' font-size='90'>♥</text></svg>" />

// Option A: Use a PNG file in /public/
<link rel="icon" href="/favicon.ico" />

// Option B: Different emoji
<link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><text y='.9em' font-size='90'>🏥</text></svg>" />
```

---

## 4. Brand Colors

Primary accent color is `#00c8ff` (cyan). To change it, find/replace in:

| File | What to change |
|---|---|
| `pages/index.js` | `stroke="#00c8ff"` in SVG icon, `.status-dot` color, device card active border |
| `styles/globals.css` | `--cyan` variable — controls vital card colors, progress bars |
| `components/TrendChart.js` | Fallback color `COLOR_MAP` keys |
| `components/OverviewGrid.js` | Selected border and background |

**Quick full rebrand** — add to `styles/globals.css`:
```css
:root {
  --brand-primary: #your-color;   /* replaces #00c8ff everywhere */
}
```
Then replace `#00c8ff` with `var(--brand-primary)` in the files above.

---

## 5. Vital Icons

In `pages/index.js` — `VITALS` array:
```js
const VITALS = [
  { key: "ppgHeartRate", icon: "❤️",  ... },  // ← change emoji
  { key: "ecgHeartRate", icon: "💓",  ... },
  { key: "spo2",         icon: "💧",  ... },
  { key: "temperature",  icon: "🌡",  ... },
];
```

Common alternatives:
| Vital | Options |
|---|---|
| PPG Heart Rate | `❤️` `♥` `🫀` |
| ECG Heart Rate | `💓` `♥` `🫀` |
| SpO₂ | `💧` `🩸` `O₂` |
| Temperature | `🌡` `🌡️` `°C` |

---

## 6. Device Icon

Device cards use the `📡` emoji. To change:

```jsx
// components/OverviewGrid.js — line with 📡
<span style={{ fontSize: 14 }}>📡</span>   // ← change emoji

// pages/index.js — device-card-top
<span className="device-icon">
  {hasAlert ? "⚠" : "📡"}   // ← change 📡
</span>
```

---

## 7. App-wide Font

In `styles/globals.css`, the app uses system fonts by default. To set a custom font:
```css
/* Add to top of globals.css */
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap');

:root {
  --font-sans: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
}

body {
  font-family: var(--font-sans);
}
```

Chart axis labels use `Share Tech Mono` — change in `components/TrendChart.js`:
```js
fontFamily: "Share Tech Mono, monospace"  // ← replace with your monospace font
```