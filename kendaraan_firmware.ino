/*
  ============================================================
  FIRMWARE KENDARAAN OTOMATIS (Person-Following + Obstacle Avoidance)
  ============================================================
  Board   : ESP32-S3 (terpasang di kendaraan)
  Sensor  : 3x Ultrasonik HC-SR04 (kiri, tengah, kanan)
            3x Penerima Inframerah KY-022 (kiri, tengah, kanan)
  Aktuator: 2x Motor DC via Motor Driver (TB6612FNG / L298N)
  Fitur   : Dashboard real-time via WiFi (Server-Sent Events),
            bisa dibuka lewat browser HP/laptop di jaringan
            WiFi yang sama.

  --- LANGKAH SEBELUM UPLOAD ---
  1. Isi WIFI_SSID dan WIFI_PASSWORD di bawah sesuai WiFi kamu.
  2. Cek ulang alokasi pin di bagian "PENGATURAN PIN" - sesuaikan
     kalau wiring fisik kamu beda dari tabel di laporan.
  3. Upload ke ESP32-S3 kendaraan (BUKAN yang dipakai buat beacon).
  4. Buka Serial Monitor (115200 baud) - setelah konek WiFi,
     akan muncul alamat IP. Buka alamat itu di browser HP/laptop
     yang nyambung ke WiFi yang sama untuk lihat dashboard.

  --- CATATAN PENTING ---
  - Kode ini TIDAK memerlukan library tambahan (semua library
    yang dipakai sudah bawaan Arduino core ESP32).
  - Kalau "analogWrite()" gagal compile di board kamu (tergantung
    versi Arduino core ESP32 yang terinstall), ganti fungsi
    setMotor() di bawah untuk pakai ledcAttach()/ledcWrite()
    sebagai gantinya (cari contoh "ESP32 ledcWrite PWM").
  ============================================================
*/

#include <WiFi.h>
#include <WebServer.h>

// ============================================================
// 1. PENGATURAN WIFI (WAJIB DIISI)
// ============================================================
const char* WIFI_SSID     = "NAMA_WIFI_ANDA";
const char* WIFI_PASSWORD = "PASSWORD_WIFI_ANDA";

// ============================================================
// 2. PENGATURAN PIN
// ============================================================

// --- Sensor Ultrasonik HC-SR04 ---
#define TRIG_KIRI   4
#define ECHO_KIRI   5
#define TRIG_TENGAH 6
#define ECHO_TENGAH 7
#define TRIG_KANAN  15
#define ECHO_KANAN  16

// --- Penerima Inframerah KY-022 (output digital) ---
#define IR_KIRI   1
#define IR_TENGAH 2
#define IR_KANAN  42

// --- Motor Driver (contoh: TB6612FNG) ---
#define AIN1 17   // Arah motor kiri
#define AIN2 18
#define PWMA 8    // Kecepatan motor kiri
#define BIN1 9    // Arah motor kanan
#define BIN2 10
#define PWMB 11   // Kecepatan motor kanan
#define STBY 12   // Standby / enable driver

// ============================================================
// 3. PARAMETER NAVIGASI (bisa kamu tuning nanti)
// ============================================================
const float JARAK_AMAN_CM   = 20.0;  // batas dianggap "terhalang"
const float JARAK_IDEAL_CM  = 30.0;  // jarak minimum sebelum berhenti dekat orang
const unsigned long IR_TIMEOUT_MS       = 150;   // batas waktu sinyal IR dianggap "masih ada"
const unsigned long BATAS_WAKTU_MENCARI = 5000;  // 5 detik muter cari sinyal sebelum nyerah
const int PWM_NORMAL = 150;  // kecepatan normal (0-255)
const int PWM_PUTAR  = 120;  // kecepatan saat berputar/menghindar

const int JUMLAH_SAMPEL   = 3;      // jumlah sampel untuk moving average ultrasonik
const unsigned long INTERVAL_SIKLUS = 100; // siklus kontrol tiap 100 ms (10 Hz)

// ============================================================
// 4. TIPE DATA & VARIABEL GLOBAL
// ============================================================
enum ArahSektor { KIRI, TENGAH, KANAN, HILANG };
enum ModeSistem { MENGIKUTI, MENGHINDAR, MUNDUR, MENCARI, BERHENTI };

WebServer server(80);
WiFiClient sseClient;

// --- Waktu pulsa IR terakhir terdeteksi (diupdate lewat interrupt) ---
volatile unsigned long lastPulseKiri   = 0;
volatile unsigned long lastPulseTengah = 0;
volatile unsigned long lastPulseKanan  = 0;

// --- Buffer moving average ultrasonik ---
float bufferKiri[JUMLAH_SAMPEL]   = {0};
float bufferTengah[JUMLAH_SAMPEL] = {0};
float bufferKanan[JUMLAH_SAMPEL]  = {0};
int idxSampel = 0;

unsigned long lastSiklus = 0;
unsigned long waktuMulaiMencari = 0;
ArahSektor arahTerakhirDiketahui = TENGAH;

// ============================================================
// 5. INTERRUPT HANDLER PENERIMA INFRAMERAH
// ============================================================
// KY-022 mengeluarkan pulsa LOW setiap kali menangkap sinyal
// 38kHz dari beacon. Kita cukup catat "kapan terakhir kali
// pulsa itu muncul" - tidak perlu mendekode isi sinyalnya.
void IRAM_ATTR isrKiri()   { lastPulseKiri   = millis(); }
void IRAM_ATTR isrTengah() { lastPulseTengah = millis(); }
void IRAM_ATTR isrKanan()  { lastPulseKanan  = millis(); }

// ============================================================
// 6. FUNGSI SENSOR
// ============================================================
float bacaJarakCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Timeout 25ms setara dengan jarak maksimum ±400cm.
  long durasi = pulseIn(echoPin, HIGH, 25000);
  if (durasi == 0) return 999.0; // tidak ada pantulan terdeteksi
  return durasi * 0.0343 / 2.0;  // konversi ke cm
}

float rataRata(float* buf) {
  float total = 0;
  for (int i = 0; i < JUMLAH_SAMPEL; i++) total += buf[i];
  return total / JUMLAH_SAMPEL;
}

ArahSektor tentukanArahSektor() {
  unsigned long now = millis();
  bool kiriOK   = (now - lastPulseKiri)   < IR_TIMEOUT_MS;
  bool tengahOK = (now - lastPulseTengah) < IR_TIMEOUT_MS;
  bool kananOK  = (now - lastPulseKanan)  < IR_TIMEOUT_MS;

  if (!kiriOK && !tengahOK && !kananOK) return HILANG;

  // Kalau lebih dari satu sektor mendeteksi sinyal bersamaan,
  // pilih yang pulsanya paling baru (paling segar).
  unsigned long tKiri   = kiriOK   ? (now - lastPulseKiri)   : 999999UL;
  unsigned long tTengah = tengahOK ? (now - lastPulseTengah) : 999999UL;
  unsigned long tKanan  = kananOK  ? (now - lastPulseKanan)  : 999999UL;

  if (tTengah <= tKiri && tTengah <= tKanan) return TENGAH;
  if (tKiri <= tKanan) return KIRI;
  return KANAN;
}

// ============================================================
// 7. FUNGSI KONTROL MOTOR
// ============================================================
void setMotor(int in1, int in2, int pwmPin, int kecepatan) {
  // kecepatan: -255..255 (negatif = mundur)
  if (kecepatan >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    kecepatan = -kecepatan;
  }
  analogWrite(pwmPin, constrain(kecepatan, 0, 255));
}

void gerakMotor(int kiri, int kanan) {
  digitalWrite(STBY, HIGH);
  setMotor(AIN1, AIN2, PWMA, kiri);
  setMotor(BIN1, BIN2, PWMB, kanan);
}

void berhentiMotor() {
  digitalWrite(STBY, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// ============================================================
// 8. HALAMAN WEB DASHBOARD (HTML + JS, tersimpan di flash)
// ============================================================
const char HALAMAN_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Dashboard Kendaraan Otomatis</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<style>
  body { font-family: Arial, sans-serif; background:#111; color:#eee; margin:0; padding:16px; }
  h2 { margin-top:0; }
  .status-row { display:flex; gap:12px; margin-bottom:16px; flex-wrap:wrap; }
  .card { background:#1e1e1e; border-radius:8px; padding:10px 16px; min-width:110px; }
  .card b { display:block; font-size:1.3em; margin-top:4px; }
  #chart-container { background:#1e1e1e; border-radius:8px; padding:12px; }
  button { background:#2ecc71; border:none; padding:8px 14px; border-radius:6px;
           color:#000; font-weight:bold; cursor:pointer; margin-right:8px; margin-bottom:12px; }
  #statusKoneksi { font-size:0.85em; color:#888; margin-bottom:12px; }
</style>
</head>
<body>
<h2>Dashboard Kendaraan Otomatis (Real-time)</h2>
<div id="statusKoneksi">Menghubungkan ke kendaraan...</div>

<div class="status-row">
  <div class="card">Mode Sistem<b id="modeVal">-</b></div>
  <div class="card">Arah Sektor<b id="arahVal">-</b></div>
  <div class="card">Jarak Kiri (cm)<b id="ukVal">-</b></div>
  <div class="card">Jarak Tengah (cm)<b id="utVal">-</b></div>
  <div class="card">Jarak Kanan (cm)<b id="ukaVal">-</b></div>
  <div class="card">PWM Kiri / Kanan<b id="pwmVal">-</b></div>
</div>

<div>
  <button onclick="mulaiRekam()">Start Recording</button>
  <button onclick="unduhCSV()">Download CSV</button>
</div>

<div id="chart-container">
  <canvas id="grafik" height="90"></canvas>
</div>

<script>
const MAKS_TITIK = 50;
let labelWaktu = [], dataKiri = [], dataTengah = [], dataKanan = [];
let rekamAktif = false;
let dataRekaman = [];

const ctx = document.getElementById('grafik').getContext('2d');
const chart = new Chart(ctx, {
  type: 'line',
  data: {
    labels: labelWaktu,
    datasets: [
      { label: 'Kiri (cm)',   data: dataKiri,   borderColor: '#e74c3c', tension: 0.3 },
      { label: 'Tengah (cm)', data: dataTengah, borderColor: '#2ecc71', tension: 0.3 },
      { label: 'Kanan (cm)',  data: dataKanan,  borderColor: '#3498db', tension: 0.3 }
    ]
  },
  options: {
    animation: false,
    scales: { y: { title: { display: true, text: 'Jarak (cm)' }, suggestedMax: 100 } }
  }
});

function mulaiRekam() {
  rekamAktif = true;
  dataRekaman = [];
  alert('Rekaman dimulai. Klik "Download CSV" kapan saja untuk mengunduh data yang sudah terkumpul.');
}

function unduhCSV() {
  if (dataRekaman.length === 0) {
    alert('Belum ada data yang direkam. Klik "Start Recording" dulu.');
    return;
  }
  const header = Object.keys(dataRekaman[0]).join(',');
  const baris = dataRekaman.map(d => Object.values(d).join(','));
  const csv = header + '\n' + baris.join('\n');
  const blob = new Blob([csv], { type: 'text/csv' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = 'log_kendaraan.csv';
  a.click();
  URL.revokeObjectURL(url);
}

const sumber = new EventSource('/events');

sumber.onopen = function() {
  document.getElementById('statusKoneksi').innerText = 'Terhubung ke kendaraan.';
};

sumber.onerror = function() {
  document.getElementById('statusKoneksi').innerText = 'Koneksi terputus, mencoba menyambung ulang...';
};

sumber.onmessage = function(event) {
  const d = JSON.parse(event.data);

  document.getElementById('modeVal').innerText  = d.mode_sistem;
  document.getElementById('arahVal').innerText   = d.arah_sektor;
  document.getElementById('ukVal').innerText     = d.us_kiri;
  document.getElementById('utVal').innerText     = d.us_tengah;
  document.getElementById('ukaVal').innerText    = d.us_kanan;
  document.getElementById('pwmVal').innerText    = d.pwm_kiri + ' / ' + d.pwm_kanan;

  labelWaktu.push('');
  dataKiri.push(d.us_kiri);
  dataTengah.push(d.us_tengah);
  dataKanan.push(d.us_kanan);
  if (labelWaktu.length > MAKS_TITIK) {
    labelWaktu.shift(); dataKiri.shift(); dataTengah.shift(); dataKanan.shift();
  }
  chart.update();

  if (rekamAktif) dataRekaman.push(d);
};
</script>
</body>
</html>
)rawliteral";

// ============================================================
// 9. HANDLER WEB SERVER
// ============================================================
void handleRoot() {
  server.send(200, "text/html", HALAMAN_HTML);
}

void handleEvents() {
  WiFiClient client = server.client();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/event-stream");
  client.println("Cache-Control: no-cache");
  client.println("Connection: keep-alive");
  client.println();
  sseClient = client; // simpan koneksi ini untuk dikirimi data tiap siklus
}

const char* namaArah(ArahSektor a) {
  switch (a) {
    case KIRI:   return "KIRI";
    case TENGAH: return "TENGAH";
    case KANAN:  return "KANAN";
    default:     return "HILANG";
  }
}

const char* namaMode(ModeSistem m) {
  switch (m) {
    case MENGIKUTI:  return "MENGIKUTI";
    case MENGHINDAR: return "MENGHINDAR";
    case MUNDUR:     return "MUNDUR";
    case MENCARI:    return "MENCARI";
    default:         return "BERHENTI";
  }
}

void kirimDataSSE(unsigned long ts, float uk, float ut, float ukn,
                   ArahSektor arah, ModeSistem mode, int pwmK, int pwmKa) {
  if (!sseClient || !sseClient.connected()) return;

  String json = "{";
  json += "\"timestamp\":" + String(ts) + ",";
  json += "\"us_kiri\":"    + String(uk, 1) + ",";
  json += "\"us_tengah\":"  + String(ut, 1) + ",";
  json += "\"us_kanan\":"   + String(ukn, 1) + ",";
  json += "\"arah_sektor\":\"" + String(namaArah(arah)) + "\",";
  json += "\"mode_sistem\":\"" + String(namaMode(mode)) + "\",";
  json += "\"pwm_kiri\":"  + String(pwmK) + ",";
  json += "\"pwm_kanan\":" + String(pwmKa);
  json += "}";

  sseClient.print("data: ");
  sseClient.print(json);
  sseClient.print("\n\n");
}

// ============================================================
// 10. SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  // --- Ultrasonik ---
  pinMode(TRIG_KIRI, OUTPUT);   pinMode(ECHO_KIRI, INPUT);
  pinMode(TRIG_TENGAH, OUTPUT); pinMode(ECHO_TENGAH, INPUT);
  pinMode(TRIG_KANAN, OUTPUT);  pinMode(ECHO_KANAN, INPUT);

  // --- Penerima Inframerah ---
  pinMode(IR_KIRI, INPUT);
  pinMode(IR_TENGAH, INPUT);
  pinMode(IR_KANAN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR_KIRI),   isrKiri,   FALLING);
  attachInterrupt(digitalPinToInterrupt(IR_TENGAH), isrTengah, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR_KANAN),  isrKanan,  FALLING);

  // --- Motor Driver ---
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  berhentiMotor();

  // --- WiFi ---
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Menghubungkan ke WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Terhubung! Buka dashboard di browser lewat alamat: http://");
  Serial.println(WiFi.localIP());

  // --- Web Server ---
  server.on("/", handleRoot);
  server.on("/events", handleEvents);
  server.begin();

  Serial.println("Sistem siap. Memulai navigasi...");
}

// ============================================================
// 11. LOOP UTAMA
// ============================================================
void loop() {
  server.handleClient();

  unsigned long now = millis();
  if (now - lastSiklus < INTERVAL_SIKLUS) return;
  lastSiklus = now;

  // --- 1. Baca ultrasonik + update moving average ---
  bufferKiri[idxSampel]   = bacaJarakCM(TRIG_KIRI, ECHO_KIRI);
  bufferTengah[idxSampel] = bacaJarakCM(TRIG_TENGAH, ECHO_TENGAH);
  bufferKanan[idxSampel]  = bacaJarakCM(TRIG_KANAN, ECHO_KANAN);
  idxSampel = (idxSampel + 1) % JUMLAH_SAMPEL;

  float usKiri   = rataRata(bufferKiri);
  float usTengah = rataRata(bufferTengah);
  float usKanan  = rataRata(bufferKanan);

  // --- 2. Tentukan arah sektor dari larik inframerah ---
  ArahSektor arah = tentukanArahSektor();

  // --- 3. Logika pengambilan keputusan (lihat Bagian 4.1 laporan) ---
  ModeSistem mode;
  int pwmKiri = 0, pwmKanan = 0;

  if (arah == HILANG) {
    // Beacon tidak terdeteksi sama sekali -> mode MENCARI
    mode = MENCARI;
    if (waktuMulaiMencari == 0) waktuMulaiMencari = now;

    if (now - waktuMulaiMencari > BATAS_WAKTU_MENCARI) {
      mode = BERHENTI;
      pwmKiri = 0; pwmKanan = 0;
    } else {
      // Berputar pelan ke arah terakhir diketahui sambil terus memindai
      if (arahTerakhirDiketahui == KIRI) {
        pwmKiri = -PWM_PUTAR; pwmKanan = PWM_PUTAR;
      } else if (arahTerakhirDiketahui == KANAN) {
        pwmKiri = PWM_PUTAR; pwmKanan = -PWM_PUTAR;
      } else {
        pwmKiri = PWM_PUTAR; pwmKanan = -PWM_PUTAR; // default: putar ke kanan
      }
    }
  } else {
    waktuMulaiMencari = 0;
    arahTerakhirDiketahui = arah;

    float usTarget = (arah == KIRI) ? usKiri : (arah == KANAN) ? usKanan : usTengah;
    bool halanganTengah  = (usTengah < JARAK_AMAN_CM);
    bool semuaTerhalang  = (usKiri < JARAK_AMAN_CM) && (usTengah < JARAK_AMAN_CM) && (usKanan < JARAK_AMAN_CM);

    if (semuaTerhalang) {
      // Dead-end - mundur pelan
      mode = MUNDUR;
      pwmKiri = -PWM_NORMAL; pwmKanan = -PWM_NORMAL;
    } else if (halanganTengah) {
      // Ada halangan tepat di depan -> menghindar ke sisi yang lebih lapang
      mode = MENGHINDAR;
      if (usKiri > usKanan) { pwmKiri = -PWM_PUTAR; pwmKanan = PWM_PUTAR; }
      else                  { pwmKiri = PWM_PUTAR;  pwmKanan = -PWM_PUTAR; }
    } else if (usTarget < JARAK_IDEAL_CM) {
      // Sudah cukup dekat dengan target -> berhenti
      mode = MENGIKUTI;
      pwmKiri = 0; pwmKanan = 0;
    } else {
      // Jalur aman, lanjut mengikuti sesuai arah sektor
      mode = MENGIKUTI;
      if (arah == TENGAH) {
        pwmKiri = PWM_NORMAL; pwmKanan = PWM_NORMAL;
      } else if (arah == KIRI) {
        pwmKiri = PWM_NORMAL * 0.5; pwmKanan = PWM_NORMAL;
      } else {
        pwmKiri = PWM_NORMAL; pwmKanan = PWM_NORMAL * 0.5;
      }
    }
  }

  gerakMotor(pwmKiri, pwmKanan);

  // --- 4. Kirim data ke dashboard (kalau ada browser yang terhubung) ---
  kirimDataSSE(now, usKiri, usTengah, usKanan, arah, mode, pwmKiri, pwmKanan);
}
