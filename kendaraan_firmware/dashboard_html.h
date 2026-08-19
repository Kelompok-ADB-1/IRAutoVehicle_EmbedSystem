/*
  ============================================================
  DASHBOARD HTML + JS (untuk kendaraan_firmware.ino)
  ============================================================
  File ini SENGAJA dipisah dari kendaraan_firmware.ino.

  Alasannya: Arduino IDE otomatis nyisir isi file .ino buat
  bikin function prototype sebelum compile beneran (proses ini
  namanya ctags). Proses scan itu punya bug lama: kalau ada teks
  di dalam string mentah (raw string literal) yang polanya mirip
  fungsi C++ - misalnya kode JavaScript kita "function nama() {"
  - scanner-nya bisa salah paham dan mengira itu kode C++
  beneran, bukan teks di dalam string.

  Ini yang bikin muncul error kayak:
    "'function' does not name a type; did you mean 'union'?"

  Solusinya: taruh HTML/JS ini di file .h terpisah. Proses
  auto-prototype Arduino IDE cuma nyisir file .ino, JADI file
  .h ini otomatis aman dari bug itu.

  JANGAN diubah jadi .ino, dan jangan digabung balik ke file
  utama - biarkan tetap di sini.
  ============================================================
*/

#pragma once

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
  .chart-box { background:#1e1e1e; border-radius:8px; padding:12px; margin-bottom:14px; }
  .chart-box h4 { margin:0 0 8px 0; font-weight:normal; color:#aaa; }
  button { background:#2ecc71; border:none; padding:8px 14px; border-radius:6px;
           color:#000; font-weight:bold; cursor:pointer; margin-right:8px; margin-bottom:12px; }
  button#btnRekam.aktif { background:#e74c3c; color:#fff; }
  #statusKoneksi { font-size:0.85em; color:#888; margin-bottom:12px; }
  #recIndicator { font-size:0.9em; margin-left:4px; }
  #recIndicator.aktif { color:#e74c3c; animation: kedip 1s infinite; }
  #recIndicator.mati { color:#666; }
  @keyframes kedip { 0%,100%{opacity:1;} 50%{opacity:0.25;} }
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
  <button id="btnRekam" onclick="toggleRekam()">Start Recording</button>
  <button onclick="unduhCSV()">Download CSV</button>
  <span id="recIndicator" class="mati">Tidak merekam</span>
</div>

<div class="chart-box">
  <h4>Jarak Ultrasonik (cm)</h4>
  <canvas id="grafikUS" height="80"></canvas>
</div>

<div class="chart-box">
  <h4>Arah Sektor Inframerah</h4>
  <canvas id="grafikIR" height="60"></canvas>
</div>

<div class="chart-box">
  <h4>PWM Motor Kiri / Kanan</h4>
  <canvas id="grafikPWM" height="70"></canvas>
</div>

<script>
// Batas jumlah titik yang disimpan di tiap grafik, biar memori
// browser & ESP32 tetap ringan (data lama otomatis dibuang).
const MAKS_TITIK = 50;

let labelWaktu = [];
let dataKiri = [], dataTengah = [], dataKanan = [];
let dataIR = [];
let dataPwmKiri = [], dataPwmKanan = [];

let rekamAktif = false;
let dataRekaman = [];

// Encode arah sektor jadi angka supaya bisa digambar sebagai grafik
// step: KIRI=-1, TENGAH=0, KANAN=1, HILANG=null (celah kosong di grafik)
function encodeArah(arah) {
  if (arah === 'KIRI') return -1;
  if (arah === 'TENGAH') return 0;
  if (arah === 'KANAN') return 1;
  return null; // HILANG -> celah, bukan digambar 0
}

const optDasar = { animation: false, elements: { point: { radius: 0 } } };

const chartUS = new Chart(document.getElementById('grafikUS').getContext('2d'), {
  type: 'line',
  data: { labels: labelWaktu, datasets: [
    { label: 'Kiri',   data: dataKiri,   borderColor: '#e74c3c', tension: 0.3 },
    { label: 'Tengah', data: dataTengah, borderColor: '#2ecc71', tension: 0.3 },
    { label: 'Kanan',  data: dataKanan,  borderColor: '#3498db', tension: 0.3 }
  ]},
  options: { ...optDasar, scales: { y: { suggestedMax: 100, title: { display:true, text:'cm' } } } }
});

const chartIR = new Chart(document.getElementById('grafikIR').getContext('2d'), {
  type: 'line',
  data: { labels: labelWaktu, datasets: [
    { label: 'Arah', data: dataIR, borderColor: '#f39c12', stepped: true, spanGaps: false }
  ]},
  options: { ...optDasar, scales: { y: {
    min: -1.5, max: 1.5, ticks: {
      stepSize: 1,
      callback: v => ({ '-1':'KIRI', '0':'TENGAH', '1':'KANAN' }[v] ?? '')
    }
  }}}
});

const chartPWM = new Chart(document.getElementById('grafikPWM').getContext('2d'), {
  type: 'line',
  data: { labels: labelWaktu, datasets: [
    { label: 'PWM Kiri',  data: dataPwmKiri,  borderColor: '#9b59b6', tension: 0.3 },
    { label: 'PWM Kanan', data: dataPwmKanan, borderColor: '#1abc9c', tension: 0.3 }
  ]},
  options: { ...optDasar, scales: { y: { suggestedMin: -255, suggestedMax: 255 } } }
});

function toggleRekam() {
  rekamAktif = !rekamAktif;
  const tombol = document.getElementById('btnRekam');
  const indikator = document.getElementById('recIndicator');

  if (rekamAktif) {
    dataRekaman = [];
    tombol.innerText = 'Stop Recording';
    tombol.classList.add('aktif');
    indikator.innerText = 'Merekam...';
    indikator.className = 'aktif';
  } else {
    tombol.innerText = 'Start Recording';
    tombol.classList.remove('aktif');
    indikator.innerText = dataRekaman.length + ' data siap diunduh';
    indikator.className = 'mati';
  }
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

  document.getElementById('modeVal').innerText = d.mode_sistem;
  document.getElementById('arahVal').innerText  = d.arah_sektor;
  document.getElementById('ukVal').innerText    = d.us_kiri;
  document.getElementById('utVal').innerText    = d.us_tengah;
  document.getElementById('ukaVal').innerText   = d.us_kanan;
  document.getElementById('pwmVal').innerText   = d.pwm_kiri + ' / ' + d.pwm_kanan;

  labelWaktu.push('');
  dataKiri.push(d.us_kiri);
  dataTengah.push(d.us_tengah);
  dataKanan.push(d.us_kanan);
  dataIR.push(encodeArah(d.arah_sektor));
  dataPwmKiri.push(d.pwm_kiri);
  dataPwmKanan.push(d.pwm_kanan);

  if (labelWaktu.length > MAKS_TITIK) {
    labelWaktu.shift();
    dataKiri.shift(); dataTengah.shift(); dataKanan.shift();
    dataIR.shift();
    dataPwmKiri.shift(); dataPwmKanan.shift();
  }

  chartUS.update();
  chartIR.update();
  chartPWM.update();

  if (rekamAktif) {
    dataRekaman.push(d);
    document.getElementById('recIndicator').innerText = 'Merekam... (' + dataRekaman.length + ' data)';
  }
};
</script>
</body>
</html>
)rawliteral";
