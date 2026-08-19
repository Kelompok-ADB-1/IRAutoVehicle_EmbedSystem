/*
  ============================================================
  FIRMWARE KENDARAAN OTOMATIS (Person-Following + Obstacle Avoidance)
  ============================================================
  Board   : ESP32-S3 (terpasang di kendaraan)
  Sensor  : 3x Ultrasonik HC-SR04 (kiri, tengah, kanan)
            3x Penerima Inframerah KY-022 (kiri, tengah, kanan)
  Aktuator: 2x Motor DC via Motor Driver L298N
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
  - File ini butuh 1 file tambahan di folder sketch yang sama:
    "dashboard_html.h" - JANGAN dihapus/dipindah, karena isi
    dashboard web (HTML/JS) sengaja dipisah ke situ supaya
    proses auto-prototype Arduino IDE tidak salah baca kode
    JavaScript di dalamnya sebagai kode C++ (lihat catatan di
    dashboard_html.h untuk detail kenapa).
  - Kalau "analogWrite()" gagal compile di board kamu (tergantung
    versi Arduino core ESP32 yang terinstall), ganti fungsi
    setMotor() di bawah untuk pakai ledcAttach()/ledcWrite()
    sebagai gantinya (cari contoh "ESP32 ledcWrite PWM").
  ============================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include "dashboard_html.h"   // isi halaman dashboard (HTML+JS) - lihat file terpisah

// ============================================================
// 1. PENGATURAN WIFI (WAJIB DIISI)
// ============================================================
const char* WIFI_SSID     = "RG-AP720-L";
const char* WIFI_PASSWORD = "Ap1234321";

// --- Nama mDNS (supaya dashboard SELALU bisa dibuka lewat nama
//     ini, tanpa perlu tau/tebak gateway atau IP dari router.
//     Berguna pas kendaraan jalan pakai baterai tanpa USB,
//     karena Serial Monitor jadi nggak bisa dibuka buat lihat
//     IP yang dikasih router). ---
// Buka di browser: http://kendaraan.local
const char* MDNS_HOSTNAME = "kendaraan";

// ============================================================
// 2. PENGATURAN PIN
// ============================================================

// --- Sensor Ultrasonik HC-SR04 ---
#define TRIG_KIRI   7
#define ECHO_KIRI   15
#define TRIG_TENGAH 16
#define ECHO_TENGAH 17
#define TRIG_KANAN  18
#define ECHO_KANAN  8

// --- Penerima Inframerah KY-022 (output digital) ---
#define IR_KIRI   4
#define IR_TENGAH 5
#define IR_KANAN  6

// --- Motor Driver L298N ---
#define IN1 46   // Arah motor kiri
#define IN2 9
#define ENA 12    // Kecepatan (PWM) motor kiri
#define IN3 10    // Arah motor kanan
#define IN4 11
#define ENB 13   // Kecepatan (PWM) motor kanan
// Catatan: L298N tidak punya pin STBY seperti TB6612FNG.
// Modul L298N biasanya punya jumper "ENA"/"ENB" bawaan - pastikan
// jumper itu DILEPAS kalau kamu mau kontrol kecepatan lewat PWM
// dari ESP32-S3 (kalau jumper terpasang, motor selalu full speed).

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
bool wifiTersambung = false; // status WiFi - dicek juga di loop() supaya server.handleClient() cuma dipanggil kalau WiFi memang nyambung

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
  setMotor(IN1, IN2, ENA, kiri);
  setMotor(IN3, IN4, ENB, kanan);
}

void berhentiMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ============================================================
// 8. HANDLER WEB SERVER
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
// 8b. LOGGING KE FLASH INTERNAL (LittleFS) - TIDAK BUTUH WIFI
// ============================================================
// Ini jalur penyimpanan data CADANGAN yang selalu aktif, terlepas
// dari WiFi/dashboard nyambung atau tidak. Cocok dipakai pas
// kendaraan jalan pakai baterai tanpa terhubung ke PC/WiFi.
//
// CATATAN PENTING SEBELUM UPLOAD:
// Fitur ini butuh partition scheme yang menyediakan ruang
// LittleFS/SPIFFS. Di Arduino IDE: Tools > Partition Scheme >
// pilih salah satu yang ada kata "spiffs" di namanya (misal
// "Default 4MB with spiffs"). Kalau partition scheme yang
// kepilih nggak punya spiffs, LittleFS.begin() akan gagal terus
// (akan muncul pesan error di Serial Monitor).
const char* LOG_PATH = "/log.csv";
bool littleFsSiap = false;

void tulisHeaderLogJikaBelumAda() {
  if (!LittleFS.exists(LOG_PATH)) {
    File f = LittleFS.open(LOG_PATH, "w");
    if (f) {
      f.println("timestamp,us_kiri,us_tengah,us_kanan,arah_sektor,mode_sistem,pwm_kiri,pwm_kanan");
      f.close();
    }
  }
}

void catatBarisLog(unsigned long ts, float uk, float ut, float ukn,
                    ArahSektor arah, ModeSistem mode, int pwmK, int pwmKa) {
  if (!littleFsSiap) return;

  File f = LittleFS.open(LOG_PATH, "a");
  if (!f) return;

  f.print(ts);           f.print(',');
  f.print(uk, 1);        f.print(',');
  f.print(ut, 1);        f.print(',');
  f.print(ukn, 1);       f.print(',');
  f.print(namaArah(arah)); f.print(',');
  f.print(namaMode(mode)); f.print(',');
  f.print(pwmK);          f.print(',');
  f.println(pwmKa);
  f.close(); // close() otomatis flush ke flash - lebih lambat tapi lebih aman
             // kalau baterai tiba-tiba habis di tengah pengujian
}

// Ketik "dump" di Serial Monitor untuk menampilkan seluruh isi
// log (tinggal copy-paste ke file .csv di laptop kamu), atau
// ketik "clear" untuk menghapus log dan mulai sesi baru.
void cekPerintahSerial() {
  if (!Serial.available()) return;

  String perintah = Serial.readStringUntil('\n');
  perintah.trim();

  if (perintah == "dump") {
    File f = LittleFS.open(LOG_PATH, "r");
    if (!f) {
      Serial.println("Belum ada file log.");
      return;
    }
    Serial.println("===== MULAI LOG (copy semua baris di bawah) =====");
    while (f.available()) Serial.write(f.read());
    Serial.println("===== SELESAI LOG =====");
    f.close();
  } else if (perintah == "clear") {
    LittleFS.remove(LOG_PATH);
    tulisHeaderLogJikaBelumAda();
    Serial.println("Log dihapus, mulai sesi baru.");
  }
}

// ============================================================
// 9. SETUP
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

  // --- Motor Driver L298N ---
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  berhentiMotor();

  // --- LittleFS: penyimpanan data cadangan, TIDAK BUTUH WIFI ---
  littleFsSiap = LittleFS.begin(true); // true = auto-format kalau gagal mount
  if (littleFsSiap) {
    tulisHeaderLogJikaBelumAda();
    Serial.println("LittleFS siap. Data sensor akan dicatat ke flash internal.");
    Serial.println("Ketik 'dump' di Serial Monitor untuk lihat log, 'clear' untuk hapus.");
  } else {
    Serial.println("PERINGATAN: LittleFS gagal di-mount. Cek Partition Scheme di Tools > Partition Scheme (pilih yang ada 'spiffs'-nya). Logging ke flash TIDAK aktif.");
  }

  // --- WiFi (OPSIONAL - robot tetap jalan walau ini gagal) ---
  // Dikasih batas waktu 8 detik supaya kalau WiFi gagal connect
  // (misal karena masalah daya), robot TETAP lanjut bergerak dan
  // TETAP mencatat data ke LittleFS - bukan diam menunggu selamanya.
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Menghubungkan ke WiFi (maks 8 detik)");
  unsigned long waktuMulaiWifi = millis();
  while (millis() - waktuMulaiWifi < 8000) {
    if (WiFi.status() == WL_CONNECTED) { wifiTersambung = true; break; }
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (wifiTersambung) {
    Serial.print("Terhubung! IP dari router: ");
    Serial.println(WiFi.localIP());

    // --- mDNS: supaya dashboard bisa dibuka lewat nama tetap,
    //     tanpa perlu tau IP dari router setiap kali. ---
    if (MDNS.begin(MDNS_HOSTNAME)) {
      Serial.print("mDNS aktif! Buka dashboard di: http://");
      Serial.print(MDNS_HOSTNAME);
      Serial.println(".local");
      MDNS.addService("http", "tcp", 80);
    } else {
      Serial.println("PERINGATAN: mDNS gagal diaktifkan. Pakai IP dari router di atas sebagai gantinya.");
    }

    // --- Web Server ---
    server.on("/", handleRoot);
    server.on("/events", handleEvents);
    server.begin();
  } else {
    Serial.println("WiFi tidak tersambung - lanjut TANPA dashboard.");
    Serial.println("Data sensor tetap dicatat ke LittleFS (kalau berhasil di-mount di atas).");
  }

  Serial.println("Sistem siap. Memulai navigasi...");
}

// ============================================================
// 10. LOOP UTAMA
// ============================================================
void loop() {
  if (wifiTersambung) server.handleClient();
  cekPerintahSerial(); // cek kalau ada perintah "dump" / "clear" dari Serial Monitor

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
  catatBarisLog(now, usKiri, usTengah, usKanan, arah, mode, pwmKiri, pwmKanan);
}
