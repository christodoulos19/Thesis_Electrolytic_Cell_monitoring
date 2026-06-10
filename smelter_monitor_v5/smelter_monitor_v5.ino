// ============================================================
// Aluminum Smelter Brick Monitor
// Hardware: ESP32 + MCP3424 ADC + 2x MAX31855 Thermocouples
// Purpose:  Monitor brick impedance + hot/cold face temperatures
// ============================================================
//
// GPIO 21 (SDA) => SDA του ADC και SDA των δύο Thermocouples
// GPIO 22 (SCL) => SCL του ADC και SCL των δύο Thermocouples
// 3.3V => VCC όλων (Προσοχή αν τα modules θέλουν 5V όπως είπαμε)
// GND => Κοινή γείωση για όλα
//
// Cold Sensor (I2C 2): SDA (του αισθητήρα) => Pin IO27, SCL (του αισθητήρα) => Pin IO26
// Hot Sensor & ADC (I2C 1):Παραμένουν κανονικά στα pins 21 και 22.
//
//
// LIBRARIES NEEDED:
//   - MCP342x by Steve Marple
//   - Adafruit MAX31855 library
// ============================================================

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <MCP342x.h>
#include <DFRobot_MAX31855.h>

// ── WiFi ────────────────────────────────────────────────────
const char* SSID     = "sensors";
const char* PASSWORD = "sensorslab";

// ── Hardware pins ───────────────────────────────────────────
#define DAC_PIN         25    // Impedance excitation
//I have deleted spi pins
// Χρησιμοποιούμε GPIO 21 (SDA) και GPIO 22 (SCL) για όλα.

//
#define I2C_ADDR_HOT    0x10 
#define I2C_ADDR_COLD   0x10

// ── Pins για το 2ο I2C Bus (Cold Face) ──────────────────────
#define SDA2 27  // Το IO27 στη φωτογραφία σου
#define SCL2 26  // Το IO26 στη φωτογραφία σου

// ── Measurement parameters ──────────────────────────────────
#define R_REF           10000000.0 // Reference resistor (10MΩ)
#define SAMPLE_INTERVAL 2000       // Sample every 2 seconds
#define HISTORY_SIZE    200        // Circular buffer
#define ADC_AVERAGES    16         // Software averaging
#define DAC_EXCITATION  255        // Max voltage for better SNR

// ── Objects ─────────────────────────────────────────────────
// Δημιουργία του δεύτερου I2C αντικειμένου
TwoWire I2C_BUS_2 = TwoWire(1);
MCP342x adc(0x68);
DFRobot_MAX31855 thermoHot(&Wire, 0x10);        // Bus 0 (Pins 21, 22)
DFRobot_MAX31855 thermoCold(&I2C_BUS_2, 0x10);  // Bus 1 (Pins 27, 26)
WebServer server(80);

// ── Data structures ─────────────────────────────────────────
struct Reading {
  unsigned long timestamp;
  float impedance;
  float tempHot;
  float tempCold;
  float vBrick;
  float vTotal;
};

Reading history[HISTORY_SIZE];
int historyIndex = 0;
int historyCount = 0;
int totalCount   = 0;

float currentZ        = 0;
float currentTempHot  = 0;
float currentTempCold = 0;
float baselineZ       = 0;
bool  hasBaseline     = false;
bool  alertActive     = false;

float minZ = 1e9, maxZ = 0, sumZ = 0;
float minTH = 1e9, maxTH = 0, sumTH = 0;
float minTC = 1e9, maxTC = 0, sumTC = 0;

unsigned long lastSample = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Aluminum Smelter Monitor ===");
 

  // Αρχικοποίηση I2C (Κοινό για ADC και Thermocouples)
  Wire.begin(21, 22);
  delay(100);


  // Αρχικοποίηση 2ου Bus (Cold Face)
  I2C_BUS_2.begin(SDA2, SCL2, 100000); 

  // Αρχικοποίηση Θερμοστοιχείων
  // Καλούμε το begin() χωρίς σύγκριση γιατί επιστρέφει void
  thermoHot.begin();
  Serial.println("Hot Face Sensor: begin() called.");

  thermoCold.begin();
  Serial.println("Cold Face Sensor: begin() called.");

  delay(200); // Μικρή αναμονή για να σταθεροποιηθούν οι αισθητήρες

  thermoHot.begin();
  if (isnan(thermoHot.readCelsius())) {
    Serial.println("Warning: Hot Face Sensor NOT responding!");
  } else {
    Serial.println("Hot Face Sensor is ONLINE.");
  }

  thermoCold.begin();
  if (isnan(thermoCold.readCelsius())) {
    Serial.println("Warning: Cold Face Sensor NOT responding!");
  } else {
    Serial.println("Cold Face Sensor is ONLINE.");
  }
  
  

  // Αρχικοποίηση ADC
  MCP342x::generalCallReset();
  delay(200);

  dacWrite(DAC_PIN, 0);
  

  // Σύνδεση στο WiFi
  WiFi.begin(SSID, PASSWORD);
  Serial.print("Connecting to WiFi");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    WiFi.softAP("SmelterMonitor", "monitor123");
    Serial.println("\nAccess Point Started");
  }

  // Ρύθμιση Web Server
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/csv", handleCSV);
  server.on("/setbaseline", handleSetBaseline);
  server.on("/reset", handleReset);
  server.begin();

  Serial.println("System ready!");

  pinMode(34, INPUT);
  analogReadResolution(12);           // 12-bit ανάλυση (0-4095)
  analogSetAttenuation(ADC_11db);     // Επιτρέπει μετρήσεις έως ~3.1V - 3.3V
}

// ── Read ADC channel with averaging ─────────────────────────
float readMV(MCP342x::Channel ch) {
  long sum = 0;
  int valid = 0;

  for (int i = 0; i < ADC_AVERAGES; i++) {
    long value = 0;
    MCP342x::Config status;
    uint8_t err = adc.convertAndRead(ch, MCP342x::oneShot,
                                      MCP342x::resolution18, MCP342x::gain8,
                                      1000000, value, status);
    if (err == 0) {
      sum += value;
      valid++;
    }
  }

  if (valid == 0) return 0;
  
  // At 18-bit, gain=1: LSB = 0.015625 mV
  // With gain=8, the LSB is the same but the value is pre-amplified
  // So we divide by gain to get actual voltage
  return ((sum / valid) * 0.015625) / 8.0;
}

// ── Measure impedance ───────────────────────────────────────
float measureImpedance(float &vb, float &vt) {
  dacWrite(DAC_PIN, DAC_EXCITATION);
  delay(200);  // Longer settle for high impedance

  vb = readMV(MCP342x::channel1);  // Voltage across brick
  vt = readMV(MCP342x::channel2);  // Total voltage

  dacWrite(DAC_PIN, 0);

  float vRef = vt - vb;
  if (abs(vRef) < 0.01) return 9999999.0;  // Open circuit
  if (abs(vb) < 0.001)  return 0.0;        // Short circuit
  
  return R_REF * (abs(vb) / abs(vRef));
}

// ── Take complete reading ───────────────────────────────────
void takeReading() {
  Reading r;
  r.timestamp = millis();

  // Impedance
  r.impedance = measureImpedance(r.vBrick, r.vTotal);

  // Νέος τρόπος διαβάσματος I2C
  r.tempHot  = thermoHot.readCelsius();
  r.tempCold = thermoCold.readCelsius();

  // Validate
  if (isnan(r.tempHot))  r.tempHot  = -999;
  if (isnan(r.tempCold)) r.tempCold = -999;

  // Store
  history[historyIndex] = r;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) historyCount++;
  totalCount++;

  // Update current values
  currentZ        = r.impedance;
  currentTempHot  = r.tempHot;
  currentTempCold = r.tempCold;

  // Statistics
  if (r.impedance < 9000000) {
    sumZ += r.impedance;
    if (r.impedance < minZ) minZ = r.impedance;
    if (r.impedance > maxZ) maxZ = r.impedance;
  }

  if (r.tempHot > -900) {
    sumTH += r.tempHot;
    if (r.tempHot < minTH) minTH = r.tempHot;
    if (r.tempHot > maxTH) maxTH = r.tempHot;
  }

  if (r.tempCold > -900) {
    sumTC += r.tempCold;
    if (r.tempCold < minTC) minTC = r.tempCold;
    if (r.tempCold > maxTC) maxTC = r.tempCold;
  }

  // Auto-baseline
  if (!hasBaseline && totalCount >= 5 && currentZ < 9000000) {
    baselineZ = sumZ / totalCount;
    hasBaseline = true;
    Serial.printf("Baseline impedance: %.1f Ω\n", baselineZ);
  }

  // Alert
  alertActive = hasBaseline && (currentZ > baselineZ * 2.0);

  // Serial output
  Serial.printf("[%lus] Z=%.1fΩ | T_hot=%.1f°C | T_cold=%.1f°C%s\n",
    millis() / 1000, currentZ, currentTempHot, currentTempCold,
    alertActive ? " ⚠ALERT" : "");
}

// ── Format impedance ────────────────────────────────────────
String fmtZ(float z) {
  if (z >= 1e6) return String(z/1e6, 2) + " MΩ";
  if (z >= 1e3) return String(z/1e3, 2) + " kΩ";
  return String(z, 1) + " Ω";
}

//========BATTERY PERSENTAGE=============
const int batteryPin = 34; // ειχε θεμα η 36

float getBatteryPercentage() {
  // Χρήση του IO36 (A0) σύμφωνα με το datasheet
  int rawValue = analogRead(34); 
  
  // Μετατροπή σε τάση: 
  // Το datasheet αναφέρει working voltage 3.3V
  // Ο διαιρέτης τάσης στην FireBeetle είναι συνήθως 1:1 (x2)
  float voltage = (rawValue / 4095.0) * 3.3 * 2.15;
  Serial.print("Raw ADC Value: "); Serial.println(rawValue);
  
  // Serial Debugging για να δούμε την πραγματική τιμή
  Serial.print("Battery Raw: "); Serial.print(rawValue);
  Serial.print(" | Voltage: "); Serial.println(voltage);

  

  // Τα όρια της Li-Po μπαταρίας: 4.2V (100%) έως 3.5V (0%)
  float percentage = (voltage - 3.5) / (4.2 - 3.5) * 100;

  if (percentage > 100) return 100.0;
  if (percentage < 0) return 0.0;
  return percentage;
}

// ── Web: Main page ──────────────────────────────────────────
void handleRoot() {
  String html = R"html(
<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Aluminum Smelter Monitor</title>
<style>

:root {
  --bg: #0d0d0f;
  --panel: #1a1a1e;
  --border: #2a2a30;
  --accent: #ff6b1a;
  --hot: #ff3d3d;
  --cold: #00d4ff;
  --warn: #ffa500;
  --good: #00ff88;
  --text: #e8e8ea;
  --muted: #888;
}

.batt-good { color: var(--good) !important; }
.batt-low { color: var(--hot) !important; animation: blink 1s infinite; }

* { margin:0; padding:0; box-sizing:border-box; }

body {
  background: var(--bg);
  color: var(--text);
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
  padding: 20px;
  min-height: 100vh;
}

.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 30px;
  padding-bottom: 20px;
  border-bottom: 2px solid var(--border);
}

.header-left h1 {
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
  font-size: 28px;
  font-weight: 700;
  letter-spacing: 1px;
  color: var(--accent);
  text-transform: uppercase;
}

.header-left .subtitle {
  font-size: 13px;
  color: var(--muted);
  letter-spacing: 3px;
  margin-top: 5px;
}

.status-badge {
  background: var(--good);
  color: var(--bg);
  padding: 8px 20px;
  border-radius: 20px;
  font-weight: 700;
  font-size: 14px;
  letter-spacing: 1px;
  animation: pulse 2s infinite;
}

.status-badge.alert {
  background: var(--hot);
  animation: blink 1s infinite;
}

@keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.7} }
@keyframes blink { 0%,50%{opacity:1} 51%,100%{opacity:0.3} }

.alert-banner {
  display: none;
  background: linear-gradient(90deg, rgba(255,61,61,0.2), rgba(255,61,61,0.05));
  border: 2px solid var(--hot);
  border-radius: 8px;
  padding: 15px 20px;
  margin-bottom: 20px;
  font-size: 16px;
  font-weight: 700;
  letter-spacing: 2px;
  color: var(--hot);
}
.alert-banner.visible { display: block; }

.grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 20px;
  margin-bottom: 30px;
}

.card {
  background: var(--panel);
  border: 1px solid var(--border);
  border-radius: 12px;
  padding: 25px;
  position: relative;
  overflow: hidden;
  transition: all 0.3s;
}

.card::before {
  content: '';
  position: absolute;
  top: 0; left: 0; right: 0;
  height: 3px;
  background: var(--accent);
}

.card.hot::before { background: var(--hot); }
.card.cold::before { background: var(--cold); }
.card.alert-card::before { 
  background: var(--hot);
  animation: slideRight 2s infinite;
}

@keyframes slideRight {
  0%,100% { transform: translateX(-100%); }
  50% { transform: translateX(100%); }
}

.card-label {
  font-size: 11px;
  letter-spacing: 3px;
  text-transform: uppercase;
  color: var(--muted);
  margin-bottom: 12px;
  font-weight: 600;
}

.card-value {
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
  font-size: 36px;
  font-weight: 700;
  line-height: 1;
  color: var(--accent);
}

.card-value.hot { color: var(--hot); }
.card-value.cold { color: var(--cold); }
.card-value.alert { color: var(--hot); }
.card-value.good { color: var(--good); }

.card-unit {
  font-size: 13px;
  color: var(--muted);
  margin-top: 8px;
  font-weight: 300;
}

.chart-section {
  background: var(--panel);
  border: 1px solid var(--border);
  border-radius: 12px;
  padding: 25px;
  margin-bottom: 20px;
}

.chart-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 20px;
}

.chart-title {
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
  font-size: 16px;
  font-weight: 700;
  letter-spacing: 1px;
  text-transform: uppercase;
  color: var(--accent);
}

canvas { width: 100% !important; display: block; }

.controls {
  display: flex;
  gap: 12px;
  flex-wrap: wrap;
  margin-bottom: 20px;
}

button, a.btn {
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
  font-size: 13px;
  font-weight: 600;
  letter-spacing: 1px;
  text-transform: uppercase;
  padding: 12px 24px;
  border: 2px solid var(--border);
  border-radius: 6px;
  background: var(--panel);
  color: var(--text);
  cursor: pointer;
  transition: all 0.3s;
  text-decoration: none;
  display: inline-block;
}

button:hover, a.btn:hover {
  border-color: var(--accent);
  color: var(--accent);
  box-shadow: 0 0 15px rgba(255,107,26,0.3);
}

.raw-data {
  background: var(--panel);
  border: 1px solid var(--border);
  border-radius: 12px;
  padding: 25px;
}

.raw-title {
  font-size: 11px;
  letter-spacing: 3px;
  text-transform: uppercase;
  color: var(--muted);
  margin-bottom: 15px;
  font-weight: 600;
}

.raw-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 12px;
}

.raw-item {
  display: flex;
  justify-content: space-between;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
  font-size: 13px;
  padding: 8px 0;
  border-bottom: 1px solid var(--border);
}

.raw-key { color: var(--muted); }
.raw-val { color: var(--text); font-weight: 700; }

.footer {
  text-align: center;
  color: var(--muted);
  font-size: 12px;
  margin-top: 30px;
  padding-top: 20px;
  border-top: 1px solid var(--border);
  letter-spacing: 1px;
}
</style>
</head>
<body>

<div class="header">
  <div class="header-left">
    <h1>Smelter Monitor</h1>
    <div class="subtitle">BRICK HEALTH SURVEILLANCE SYSTEM</div>
  </div>
  <div class="status-badge" id="statusBadge">OPERATIONAL</div>
</div>

<div class="alert-banner" id="alertBanner">
  ⚠️ CRITICAL ALERT — Impedance exceeds 2× baseline. Inspect refractory immediately.
</div>

<div class="grid">
  <div class="card" id="zCard">
    <div class="card-label">Brick Impedance</div>
    <div class="card-value" id="zVal">---</div>
    <div class="card-unit">Current measurement</div>
  </div>

  <div class="card">
  <div class="card-label">Battery Status</div>
  <div class="card-value" id="battVal">---</div>
  <div class="card-unit">Zon.Cell Li-Po 2500mAh</div>
</div>
  
  <div class="card hot">
    <div class="card-label">Hot Face Temperature</div>
    <div class="card-value hot" id="tHotVal">---</div>
    <div class="card-unit">Thermocouple 1</div>
  </div>
  
  <div class="card cold">
    <div class="card-label">Cold Face Temperature</div>
    <div class="card-value cold" id="tColdVal">---</div>
    <div class="card-unit">Thermocouple 2</div>
  </div>
  
  <div class="card">
    <div class="card-label">Baseline Impedance</div>
    <div class="card-value good" id="baseVal">---</div>
    <div class="card-unit">Reference (healthy brick)</div>
  </div>
  
  <div class="card">
    <div class="card-label">Impedance Change</div>
    <div class="card-value" id="ratioVal">---</div>
    <div class="card-unit">Ratio vs baseline</div>
  </div>
  
  <div class="card">
    <div class="card-label">ΔT (Gradient)</div>
    <div class="card-value" id="dtVal">---</div>
    <div class="card-unit">Hot - Cold</div>
  </div>
</div>

<div class="chart-section">
  <div class="chart-header">
    <span class="chart-title">Impedance History</span>
    <span style="color:var(--muted);font-size:12px" id="chartInfo">--- points</span>
  </div>
  <canvas id="chartZ" height="200"></canvas>
</div>

<div class="chart-section">
  <div class="chart-header">
    <span class="chart-title">Temperature History</span>
    <span style="color:var(--muted);font-size:12px">Thermal gradient monitoring</span>
  </div>
  <canvas id="chartT" height="200"></canvas>
</div>

<div class="controls">
  <button onclick="setBaseline()">Set Current as Baseline</button>
  <a href="/csv" class="btn">Download CSV</a>
  <button onclick="reset()">Reset Statistics</button>
</div>

<div class="raw-data">
  <div class="raw-title">Raw Sensor Readings</div>
  <div class="raw-grid">
    <div class="raw-item"><span class="raw-key">V_brick</span><span class="raw-val" id="rawVb">---</span></div>
    <div class="raw-item"><span class="raw-key">V_total</span><span class="raw-val" id="rawVt">---</span></div>
    <div class="raw-item"><span class="raw-key">V_ref</span><span class="raw-val" id="rawVr">---</span></div>
    <div class="raw-item"><span class="raw-key">Samples</span><span class="raw-val" id="rawCount">---</span></div>
  </div>
</div>


<div class="footer">
  UPTIME: <span id="uptime">00:00:00</span> | AUTO-REFRESH: 3s
</div>

<script>
let chartDataZ = [], chartDataTH = [], chartDataTC = [];

function fmtZ(z) {
  if (z >= 1e6) return (z/1e6).toFixed(2) + ' MΩ';
  if (z >= 1e3) return (z/1e3).toFixed(2) + ' kΩ';
  return z.toFixed(1) + ' Ω';
}

function fmtTime(s) {
  const h = Math.floor(s/3600);
  const m = Math.floor((s%3600)/60);
  const sec = s%60;
  return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(sec).padStart(2,'0')}`;
}

function update(d) {
  const badge = document.getElementById('statusBadge');
  const banner = document.getElementById('alertBanner');
  const zCard = document.getElementById('zCard');

  const battEl = document.getElementById('battVal');
  battEl.textContent = d.batt + '%';
  
  // Αλλαγή χρώματος και blink αν η μπαταρία είναι κάτω από 20%
  if (d.batt < 20) {
    battEl.className = 'card-value batt-low';
  } else {
    battEl.className = 'card-value batt-good';
  }

  if (d.alert) {
    badge.textContent = 'CRACK ALERT';
    badge.classList.add('alert');
    banner.classList.add('visible');
    zCard.classList.add('alert-card');
  } else {
    badge.textContent = 'OPERATIONAL';
    badge.classList.remove('alert');
    banner.classList.remove('visible');
    zCard.classList.remove('alert-card');
  }

  document.getElementById('zVal').textContent = fmtZ(d.z);
  document.getElementById('tHotVal').textContent = d.tHot > -900 ? d.tHot.toFixed(1) + ' °C' : 'ERROR';
  document.getElementById('tColdVal').textContent = d.tCold > -900 ? d.tCold.toFixed(1) + ' °C' : 'ERROR';
  document.getElementById('baseVal').textContent = d.hasBase ? fmtZ(d.base) : 'Calibrating...';
  document.getElementById('uptime').textContent = fmtTime(d.uptime);

  if (d.hasBase && d.base > 0) {
    const ratio = d.z / d.base;
    const el = document.getElementById('ratioVal');
    el.textContent = ratio.toFixed(2) + '×';
    el.className = 'card-value ' + (ratio > 2 ? 'alert' : ratio > 1.5 ? 'warn' : 'good');
  }

  if (d.tHot > -900 && d.tCold > -900) {
    const dt = d.tHot - d.tCold;
    document.getElementById('dtVal').textContent = dt.toFixed(1) + ' °C';
  }

  if (d.history && d.history.length > 0) {
    const last = d.history[d.history.length - 1];
    document.getElementById('rawVb').textContent = last.vb.toFixed(4) + ' mV';
    document.getElementById('rawVt').textContent = last.vt.toFixed(4) + ' mV';
    document.getElementById('rawVr').textContent = (last.vt - last.vb).toFixed(4) + ' mV';
    document.getElementById('rawCount').textContent = d.count;
  }

  chartDataZ = d.history.map(h => ({t: h.t, v: h.z}));
  chartDataTH = d.history.map(h => ({t: h.t, v: h.tH}));
  chartDataTC = d.history.map(h => ({t: h.t, v: h.tC}));
  
  document.getElementById('chartInfo').textContent = chartDataZ.length + ' points';
  
  drawChartZ();
  drawChartT();
}

function drawChartZ() {
  const canvas = document.getElementById('chartZ');
  const ctx = canvas.getContext('2d');
  const W = canvas.parentElement.clientWidth - 50;
  const H = 200;
  canvas.width = W;
  canvas.height = H;

  const pad = {top: 20, right: 20, bottom: 30, left: 70};
  const cW = W - pad.left - pad.right;
  const cH = H - pad.top - pad.bottom;

  ctx.fillStyle = '#1a1a1e';
  ctx.fillRect(0, 0, W, H);

  if (chartDataZ.length < 2) {
    ctx.fillStyle = '#888';
    ctx.font = '14px Arial';
    ctx.textAlign = 'center';
    ctx.fillText('Collecting data...', W/2, H/2);
    return;
  }

  const vals = chartDataZ.map(d => d.v).filter(v => v < 9e6);
  if (vals.length === 0) return;

  let minV = Math.min(...vals) * 0.9;
  let maxV = Math.max(...vals) * 1.1;
  if (maxV === minV) maxV = minV + 1;

  const xScale = cW / (chartDataZ.length - 1);
  const yScale = cH / (maxV - minV);

  // Grid
  ctx.strokeStyle = '#2a2a30';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = pad.top + (cH * i / 4);
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(pad.left + cW, y);
    ctx.stroke();
    const val = maxV - (maxV - minV) * i / 4;
    ctx.fillStyle = '#888';
    ctx.font = '11px Arial';
    ctx.textAlign = 'right';
    ctx.fillText(fmtZ(val), pad.left - 6, y + 4);
  }

  // Line
  ctx.strokeStyle = '#ff6b1a';
  ctx.lineWidth = 2;
  ctx.beginPath();
  let first = true;
  chartDataZ.forEach((d, i) => {
    if (d.v >= 9e6) return;
    const x = pad.left + i * xScale;
    const y = pad.top + cH - (d.v - minV) * yScale;
    if (first) { ctx.moveTo(x, y); first = false; }
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

function drawChartT() {
  const canvas = document.getElementById('chartT');
  const ctx = canvas.getContext('2d');
  const W = canvas.parentElement.clientWidth - 50;
  const H = 200;
  canvas.width = W;
  canvas.height = H;

  const pad = {top: 20, right: 20, bottom: 30, left: 70};
  const cW = W - pad.left - pad.right;
  const cH = H - pad.top - pad.bottom;

  ctx.fillStyle = '#1a1a1e';
  ctx.fillRect(0, 0, W, H);

  if (chartDataTH.length < 2) {
    ctx.fillStyle = '#888';
    ctx.font = '14px Arial';
    ctx.textAlign = 'center';
    ctx.fillText('Collecting data...', W/2, H/2);
    return;
  }

  const valsH = chartDataTH.map(d => d.v).filter(v => v > -900);
  const valsC = chartDataTC.map(d => d.v).filter(v => v > -900);
  const allVals = [...valsH, ...valsC];
  if (allVals.length === 0) return;

  let minV = Math.min(...allVals) * 0.95;
  let maxV = Math.max(...allVals) * 1.05;
  if (maxV === minV) maxV = minV + 1;

  const xScale = cW / (chartDataTH.length - 1);
  const yScale = cH / (maxV - minV);

  // Grid
  ctx.strokeStyle = '#2a2a30';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = pad.top + (cH * i / 4);
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(pad.left + cW, y);
    ctx.stroke();
    const val = maxV - (maxV - minV) * i / 4;
    ctx.fillStyle = '#888';
    ctx.font = '11px Arial';
    ctx.textAlign = 'right';
    ctx.fillText(val.toFixed(0) + '°C', pad.left - 6, y + 4);
  }

  // Hot line
  ctx.strokeStyle = '#ff3d3d';
  ctx.lineWidth = 2;
  ctx.beginPath();
  let first = true;
  chartDataTH.forEach((d, i) => {
    if (d.v <= -900) return;
    const x = pad.left + i * xScale;
    const y = pad.top + cH - (d.v - minV) * yScale;
    if (first) { ctx.moveTo(x, y); first = false; }
    else ctx.lineTo(x, y);
  });
  ctx.stroke();

  // Cold line
  ctx.strokeStyle = '#00d4ff';
  ctx.lineWidth = 2;
  ctx.beginPath();
  first = true;
  chartDataTC.forEach((d, i) => {
    if (d.v <= -900) return;
    const x = pad.left + i * xScale;
    const y = pad.top + cH - (d.v - minV) * yScale;
    if (first) { ctx.moveTo(x, y); first = false; }
    else ctx.lineTo(x, y);
  });
  ctx.stroke();

  // Legend
  ctx.fillStyle = '#ff3d3d';
  ctx.fillRect(pad.left, H - 20, 12, 12);
  ctx.fillStyle = '#e8e8ea';
  ctx.font = '12px Arial';
  ctx.textAlign = 'left';
  ctx.fillText('Hot Face', pad.left + 18, H - 10);

  ctx.fillStyle = '#00d4ff';
  ctx.fillRect(pad.left + 100, H - 20, 12, 12);
  ctx.fillText('Cold Face', pad.left + 118, H - 10);
}

function fetchData() {
  fetch('/data')
    .then(r => r.json())
    .then(d => update(d))
    .catch(e => console.error(e));
}

function setBaseline() {
  fetch('/setbaseline').then(() => fetchData());
}

function reset() {
  if (confirm('Reset all data?')) {
    fetch('/reset').then(() => fetchData());
  }
}

setInterval(fetchData, 3000);
fetchData();

window.addEventListener('resize', () => {
  if (chartDataZ.length > 0) { drawChartZ(); drawChartT(); }
});
</script>
</body>
</html>
)html";
  server.send(200, "text/html", html);
}

// ── Web: JSON data ──────────────────────────────────────────
void handleData() {
  String json = "{";
  json += "\"z\":" + String(currentZ, 1) + ",";
  json += "\"tHot\":" + String(currentTempHot, 2) + ",";
  json += "\"tCold\":" + String(currentTempCold, 2) + ",";
  json += "\"batt\":" + String(getBatteryPercentage(), 0) + ",";
  json += "\"base\":" + String(baselineZ, 1) + ",";
  json += "\"hasBase\":" + String(hasBaseline ? "true" : "false") + ",";
  json += "\"alert\":" + String(alertActive ? "true" : "false") + ",";
  json += "\"count\":" + String(totalCount) + ",";
  json += "\"uptime\":" + String(millis() / 1000) + ",";

  json += "\"history\":[";
  int start = (historyCount < HISTORY_SIZE) ? 0 : historyIndex;
  for (int i = 0; i < historyCount; i++) {
    int idx = (start + i) % HISTORY_SIZE;
    if (i > 0) json += ",";
    json += "{";
    json += "\"t\":" + String(history[idx].timestamp / 1000) + ",";
    json += "\"z\":" + String(history[idx].impedance, 1) + ",";
    json += "\"tH\":" + String(history[idx].tempHot, 2) + ",";
    json += "\"tC\":" + String(history[idx].tempCold, 2) + ",";
    json += "\"vb\":" + String(history[idx].vBrick, 4) + ",";
    json += "\"vt\":" + String(history[idx].vTotal, 4);
    json += "}";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

// ── Web: CSV download ───────────────────────────────────────
void handleCSV() {
  String csv = "Time(s),Impedance(Ohm),Temp_Hot(C),Temp_Cold(C),V_brick(mV),V_total(mV)\n";
  int start = (historyCount < HISTORY_SIZE) ? 0 : historyIndex;
  for (int i = 0; i < historyCount; i++) {
    int idx = (start + i) % HISTORY_SIZE;
    csv += String(history[idx].timestamp / 1000) + ",";
    csv += String(history[idx].impedance, 1) + ",";
    csv += String(history[idx].tempHot, 2) + ",";
    csv += String(history[idx].tempCold, 2) + ",";
    csv += String(history[idx].vBrick, 4) + ",";
    csv += String(history[idx].vTotal, 4) + "\n";
  }
  server.sendHeader("Content-Disposition", "attachment; filename=smelter_log.csv");
  server.send(200, "text/csv", csv);
}

// ── Web: Set baseline ───────────────────────────────────────
void handleSetBaseline() {
  if (totalCount > 0 && currentZ < 9000000) {
    baselineZ = currentZ;
    hasBaseline = true;
    Serial.printf("Baseline set: %.1f Ω\n", baselineZ);
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

// ── Web: Reset ──────────────────────────────────────────────
void handleReset() {
  historyIndex = 0;
  historyCount = 0;
  totalCount = 0;
  hasBaseline = false;
  baselineZ = 0;
  alertActive = false;
  minZ = 1e9; maxZ = 0; sumZ = 0;
  minTH = 1e9; maxTH = 0; sumTH = 0;
  minTC = 1e9; maxTC = 0; sumTC = 0;
  Serial.println("Reset complete");
  server.send(200, "application/json", "{\"ok\":true}");
}



// ── Loop ────────────────────────────────────────────────────
void loop() {
  server.handleClient();

  if (millis() - lastSample >= SAMPLE_INTERVAL) {
    lastSample = millis();
    takeReading();
  }
}
