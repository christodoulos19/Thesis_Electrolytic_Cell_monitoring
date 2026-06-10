#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MCP342x.h>

// WiFi credentials - CHANGE THESE!
const char* ssid = "sensors";
const char* password = "sensorslab";

// MCP3424 setup
MCP342x adc = MCP342x(0x68);

// Standard web server
WebServer server(80);

// Data storage
const int MAX_READINGS = 500;
float readings[MAX_READINGS];
unsigned long timestamps[MAX_READINGS];
int readingIndex = 0;
int totalReadings = 0;

// Timing
unsigned long lastReading = 0;
const unsigned long INTERVAL = 5000;  // 5 seconds

// Statistics
float currentVoltage = 0;
float minVoltage = 0;
float maxVoltage = 0;
float avgVoltage = 0;

const int batteryPin = 34;

float getBatteryPercentage() {
  long sumRaw = 0;
  for(int i=0; i<15; i++) { 
    sumRaw += analogRead(batteryPin); 
    delay(2); 
  }
  float rawValue = sumRaw / 15.0;
  
  // Υπολογισμός τάσης με τον "δοκιμασμένο" συντελεστή 2.15
  float voltage = (rawValue / 4095.0) * 3.3 * 2.2;
  
  // Ποσοστό: 4.2V = 100%, 3.5V = 0%
  float percentage = (voltage - 3.5) / (4.2 - 3.5) * 100.0;

  if (percentage > 100) return 100.0;
  if (percentage < 0)   return 0.0;
  return percentage;
}

void setup() {
  Serial.begin(115200);
  pinMode(batteryPin, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  delay(1000);
  
  Serial.println("Millivoltmeter Data Logger");
  Serial.println("==========================");
  
  // Initialize I2C and ADC
  Wire.begin(21, 22);
  MCP342x::generalCallReset();
  delay(100);
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\n\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("\nOpen this address in your browser:");
  Serial.print("http://");
  Serial.println(WiFi.localIP());
  Serial.println("\n==========================");
  
  // Setup HTTP routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/download", handleDownload);
  
  // Start server
  server.begin();
  Serial.println("Web server started!");
}

void loop() {
  // Handle web requests
  server.handleClient();
  
  // Take reading every 5 seconds
  if (millis() - lastReading >= INTERVAL) {
    takeReading();
    lastReading = millis();
  }
}

void takeReading() {
  long value = 0;
  MCP342x::Config status;
  
  // Προσπαθούμε να διαβάσουμε
  uint8_t err = adc.convertAndRead(MCP342x::channel3, MCP342x::oneShot,
                                 MCP342x::resolution18, MCP342x::gain1,
                                 1000000, value, status);
  
  if (err == 0) {
    float voltage_mV = value * 0.015625;
    
    // Αποθήκευση
    readings[readingIndex] = voltage_mV;
    timestamps[readingIndex] = millis() / 1000;
    readingIndex = (readingIndex + 1) % MAX_READINGS;
    if (totalReadings < MAX_READINGS) totalReadings++;
    
    currentVoltage = voltage_mV;
    calculateStats();
    
    Serial.print("SUCCESS! Reading: ");
    Serial.println(voltage_mV);
  } else {
    // ΕΔΩ ΕΙΝΑΙ ΤΟ ΚΛΕΙΔΙ: Θα μας πει γιατί αποτυγχάνει
    Serial.print("ADC ERROR CODE: ");
    Serial.print(err); 
    if (err == 4) Serial.println(" (Device not found - Check I2C Wiring/Address)");
    else if (err == 1) Serial.println(" (Data too long)");
    else Serial.println(" (Unknown Error)");
  }
}

void calculateStats() {
  if (totalReadings == 0) return;
  
  int count = min(totalReadings, MAX_READINGS);
  minVoltage = readings[0];
  maxVoltage = readings[0];
  float sum = 0;
  
  for (int i = 0; i < count; i++) {
    if (readings[i] < minVoltage) minVoltage = readings[i];
    if (readings[i] > maxVoltage) maxVoltage = readings[i];
    sum += readings[i];
  }
  avgVoltage = sum / count;
}

void handleRoot() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1'>
    <title>Millivoltmeter Monitor</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: 'Segoe UI', Arial, sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); padding: 20px; min-height: 100vh; }
        .container { background: white; padding: 30px; border-radius: 15px; max-width: 1200px; margin: auto; box-shadow: 0 10px 40px rgba(0,0,0,0.2); }
        h1 { color: #333; margin-bottom: 10px; font-size: 32px; }
        .status { display: inline-block; padding: 5px 15px; border-radius: 20px; font-size: 14px; font-weight: bold; margin-bottom: 20px; background: #4CAF50; color: white; }
        .current-reading { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 40px; border-radius: 10px; text-align: center; margin: 20px 0; box-shadow: 0 5px 20px rgba(102, 126, 234, 0.4); }
        .current-reading .label { font-size: 18px; opacity: 0.9; margin-bottom: 10px; }
        .current-reading .value { font-size: 64px; font-weight: bold; text-shadow: 2px 2px 4px rgba(0,0,0,0.2); }
        .stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 15px; margin: 20px 0; }
        .stat { background: #f8f9fa; padding: 15px; border-radius: 10px; border-left: 4px solid #667eea; transition: transform 0.2s; }
        .stat:hover { transform: translateY(-3px); box-shadow: 0 4px 10px rgba(0,0,0,0.05); }
        .stat .label { color: #666; font-size: 13px; margin-bottom: 5px; }
        .stat .value { color: #333; font-size: 22px; font-weight: bold; }
        .chart-container { margin: 20px 0; background: #f8f9fa; padding: 20px; border-radius: 10px; }
        canvas { width: 100% !important; height: auto !important; }
        .controls { display: flex; gap: 10px; margin: 20px 0; }
        .button { background: #667eea; color: white; padding: 12px 24px; text-decoration: none; border-radius: 8px; border: none; cursor: pointer; font-weight: 600; transition: 0.3s; box-shadow: 0 4px 6px rgba(102, 126, 234, 0.3); }
        .button:hover { background: #5568d3; transform: translateY(-2px); }
        .footer { margin-top: 30px; color: #666; font-size: 14px; text-align: center; border-top: 1px solid #eee; padding-top: 15px; }
        @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.7; } }
        .updating { animation: pulse 0.5s; }
    </style>
    </head>
    <body>
    <div class='container'>
        <h1>⚡ Millivoltmeter Logger</h1>
        <span class='status' id='status-indicator'>🟢 Live</span>
        <div class='current-reading'>
            <div class='label'>Current Reading</div>
            <div class='value' id='current'>---.--- mV</div>
        </div>
        <div class='stats'>
            <div class='stat'><div class='label'>Battery</div><div class='value' id='batt'>--- %</div></div>
            <div class='stat'><div class='label'>Total Readings</div><div class='value' id='total'>0</div></div>
            <div class='stat'><div class='label'>Average</div><div class='value' id='avg'>--- mV</div></div>
            <div class='stat'><div class='label'>Minimum</div><div class='value' id='min'>--- mV</div></div>
            <div class='stat'><div class='label'>Maximum</div><div class='value' id='max'>--- mV</div></div>
        </div>
        <div class='chart-container'><canvas id='chart' width='800' height='300'></canvas></div>
        <div class='controls'>
            <a href='/download' class='button'>📥 Download CSV</a>
            <button class='button' onclick='clearChart()'>🗑️ Clear Chart</button>
        </div>
        <div class='footer'>🔄 Auto-refresh every 2s | ⏱️ Uptime: <span id='uptime'>0</span>s</div>
    </div>
    <script>
        let chartData = [];
        const MAX_POINTS = 50;
        const canvas = document.getElementById('chart');
        const ctx = canvas.getContext('2d');

        function updateData() {
            fetch('/data')
                .then(res => res.json())
                .then(data => {
                    document.getElementById('current').innerText = data.current.toFixed(3) + ' mV';
                    document.getElementById('batt').innerText = data.batt + '%';
                    document.getElementById('total').innerText = data.total;
                    document.getElementById('avg').innerText = data.avg.toFixed(3) + ' mV';
                    document.getElementById('min').innerText = data.min.toFixed(3) + ' mV';
                    document.getElementById('max').innerText = data.max.toFixed(3) + ' mV';
                    document.getElementById('uptime').innerText = data.uptime;
                    
                    document.getElementById('batt').style.color = (data.batt < 20) ? '#ff3d3d' : '#2ecc71';
                    
                    document.getElementById('current').classList.add('updating');
                    setTimeout(() => document.getElementById('current').classList.remove('updating'), 500);

                    chartData.push(data.current);
                    if(chartData.length > MAX_POINTS) chartData.shift();
                    drawChart();
                    document.getElementById('status-indicator').style.background = '#4CAF50';
                })
                .catch(err => {
                    document.getElementById('status-indicator').style.background = '#ff3d3d';
                });
        }

        function drawChart() {
            const p = 40, w = canvas.width - 2*p, h = canvas.height - 2*p;
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            if(chartData.length < 2) return;
            
            const min = Math.min(...chartData), max = Math.max(...chartData), r = (max-min) || 1;

            // Σχεδίαση Πλέγματος (Grid) και Αξόνων
            ctx.strokeStyle = '#e0e0e0'; ctx.lineWidth = 1; ctx.textAlign = 'right'; ctx.fillStyle = '#666';
            for (let i = 0; i <= 5; i++) {
                const y = p + (h / 5) * i;
                ctx.beginPath(); ctx.moveTo(p, y); ctx.lineTo(p + w, y); ctx.stroke();
                const val = max - (r / 5) * i;
                ctx.fillText(val.toFixed(2), p - 10, y + 4);
            }

            // Σχεδίαση Γραμμής
            ctx.strokeStyle = '#667eea'; ctx.lineWidth = 3; ctx.beginPath();
            chartData.forEach((v, i) => {
                const x = p + (w/(MAX_POINTS-1))*i;
                const y = p + h - ((v-min)/r)*h;
                if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
            });
            ctx.stroke();

            // Σχεδίαση Σημείων (Points)
            ctx.fillStyle = '#667eea';
            chartData.forEach((v, i) => {
                const x = p + (w/(MAX_POINTS-1))*i;
                const y = p + h - ((v-min)/r)*h;
                ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI*2); ctx.fill();
            });
        }

        function clearChart() { if(confirm('Clear chart?')) { chartData = []; drawChart(); } }
        setInterval(updateData, 2000);
        updateData();
    </script>
    </body>
    </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleData() {
  String json = "{";
  json += "\"current\":" + String(currentVoltage, 3) + ",";
  json += "\"batt\":" + String(getBatteryPercentage(), 0) + ",";
  json += "\"min\":" + String(minVoltage, 3) + ",";
  json += "\"max\":" + String(maxVoltage, 3) + ",";
  json += "\"avg\":" + String(avgVoltage, 3) + ",";
  json += "\"total\":" + String(totalReadings) + ",";
  json += "\"uptime\":" + String(millis() / 1000);
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleDownload() {
  String csv = "Reading,Time_seconds,Voltage_mV\n";
  
  int count = min(totalReadings, MAX_READINGS);
  int startIdx = totalReadings > MAX_READINGS ? readingIndex : 0;
  
  for (int i = 0; i < count; i++) {
    int idx = (startIdx + i) % MAX_READINGS;
    
    csv += String(totalReadings - count + i + 1);
    csv += ",";
    csv += String(timestamps[idx]);
    csv += ",";
    csv += String(readings[idx], 3);
    csv += "\n";
  }
  
  server.send(200, "text/csv", csv);
}
