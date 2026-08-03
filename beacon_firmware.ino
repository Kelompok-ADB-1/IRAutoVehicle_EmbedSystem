/*
  ============================================================
  FIRMWARE BEACON INFRAMERAH
  ============================================================
  Board   : ESP32-S3 (unit spare, dipakai oleh PENGGUNA, bukan
            dipasang di kendaraan)
  Fungsi  : Memancarkan sinyal inframerah termodulasi 38kHz
            secara berkala lewat LED IR, sehingga bisa
            dideteksi oleh larik penerima KY-022 yang terpasang
            di kendaraan (lihat firmware kendaraan_firmware.ino).

  Library yang perlu diinstall dulu (WAJIB):
    1. Buka Arduino IDE
    2. Menu: Sketch > Include Library > Manage Libraries...
    3. Cari "IRremote" (pembuat: shirriff / Armin Joachimsmeyer)
    4. Install versi terbaru (versi 4.x)

  Catatan buat pemula:
    - Kode ini SENGAJA tidak mendekode/membaca apa pun.
      Robot cuma perlu tahu "ada sinyal", bukan "isi sinyalnya
      apa". Jadi kode ini beneran cuma "teriak" sinyal terus-
      menerus.
    - Kalau nanti ada error compile terkait nama fungsi
      (misal "sendNEC"), kemungkinan besar versi IRremote yang
      terinstall beda dari yang dipakai saat kode ini ditulis.
      Buka contoh bawaan library (File > Examples > IRremote >
      SimpleSender) dan sesuaikan baris pengiriman sinyalnya.
  ============================================================
*/

#include <IRremote.hpp>

// ------------------------------------------------------------
// PENGATURAN PIN & WAKTU
// ------------------------------------------------------------

// Pilih pin GPIO yang aman untuk LED IR.
// Hindari GPIO 19/20 (dipakai untuk USB pada banyak board ESP32-S3).
// Kalau board kamu beda, cek pinout board tersebut.
const int IR_LED_PIN = 4;

// Beacon mengirim sinyal setiap 50 ms (20x per detik).
// Ini cukup sering supaya larik penerima di kendaraan tidak
// pernah "kehilangan jejak" terlalu lama, tapi tidak terlalu
// sering sehingga boros baterai.
const unsigned long KIRIM_INTERVAL_MS = 50;

unsigned long waktuKirimTerakhir = 0;

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  IrSender.begin(IR_LED_PIN);

  Serial.println("============================================");
  Serial.println(" Beacon Inframerah - SIAP");
  Serial.print(" LED IR terpasang di GPIO: ");
  Serial.println(IR_LED_PIN);
  Serial.println(" Mengirim sinyal setiap 50 ms...");
  Serial.println("============================================");
}

// ------------------------------------------------------------
// LOOP UTAMA
// ------------------------------------------------------------
void loop() {
  unsigned long sekarang = millis();

  if (sekarang - waktuKirimTerakhir >= KIRIM_INTERVAL_MS) {
    waktuKirimTerakhir = sekarang;

    // Kirim kode NEC apa saja secara konsisten.
    // Alamat (0xFF00) dan perintah (0x01) di bawah ini BEBAS,
    // tidak ada artinya secara khusus - yang penting dikirim
    // terus-menerus dengan pola yang sama supaya larik
    // penerima di kendaraan bisa mendeteksi "ada beacon di
    // sekitar sini".
    IrSender.sendNEC(0xFF00, 0x01, 0);

    // Baris ini opsional, cuma buat kamu memantau lewat
    // Serial Monitor kalau lagi dites di meja.
    // Serial.println("Sinyal IR terkirim");
  }
}
