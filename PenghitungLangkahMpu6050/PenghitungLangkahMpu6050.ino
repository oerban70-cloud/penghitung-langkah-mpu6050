// Penghitung Langkah — ESP32 + MPU6050 + LCD I2C 16x2 + Buzzer + Tombol Reset
//
// Alat kecil penghitung langkah kaki (pedometer): modul MPU6050 merasakan
// guncangan setiap langkah, ESP32 mendeteksinya dari puncak besarnya
// percepatan, lalu menampilkan jumlah langkah dan jarak tempuh di LCD
// 16x2. Begitu tembus target 1000 langkah, buzzer berbunyi sebagai tanda
// target tercapai. Tombol reset menghitung ulang dari nol.
//
// Cara kerja deteksi langkah (peak detection):
//   Setiap langkah kaki menghentak tanah, besarnya percepatan total
//   (|a| = akar(ax^2 + ay^2 + az^2)) melonjak di atas 1 g lalu turun
//   lagi. Satu langkah dihitung saat lonjakan melewati ambang atas
//   (0,25 g di atas 1 g) DAN sudah lewat jeda minimum sejak langkah
//   sebelumnya (250 ms, supaya satu hentakan tidak terhitung dua kali).
//   Detektor baru siap lagi setelah percepatan turun di bawah ambang
//   bawah (0,15 g). Jarak tempuh = langkah x panjang langkah rata-rata
//   orang dewasa (0,70 m per langkah).
//
// Angka datasheet MPU6050 (InvenSense, terverifikasi 26 Nov 2026):
//   Sensor            : akselerometer 3 sumbu + giroskop 3 sumbu (6-axis IMU)
//   Alamat I2C        : 0x68 (pin AD0 ke GND)
//   Akselerometer     : rentang default +-2 g, 16384 LSB per g (16-bit ADC)
//   Register baca     : 0x3B (ACCEL_XOUT_H) sampai 0x40, 6 byte sekaligus
//   PWR_MGMT_1 (0x6B) : tulis 0 untuk membangunkan chip dari sleep
//   WHO_AM_I (0x75)   : harus terbaca 0x68 bila modul tersambung benar
//
// Wiring:
//   MPU6050 VCC -> ESP32 3V3
//   MPU6050 GND -> ESP32 GND
//   MPU6050 SDA -> ESP32 GPIO21 (SDA I2C)
//   MPU6050 SCL -> ESP32 GPIO22 (SCL I2C)
//   LCD 16x2 I2C (0x27): VCC -> ESP32 VIN 5V, GND -> GND,
//     SDA -> GPIO21, SCL -> GPIO22 (satu bus I2C dengan MPU6050)
//   Buzzer + -> ESP32 GPIO25 (aktif HIGH)
//   Buzzer - -> ESP32 GND
//   Tombol reset -> ESP32 GPIO26 (INPUT_PULLUP, tekan = ke GND)
//   Daya: kabel USB 5 V ke port USB ESP32
//
// Library yang dibutuhkan (Arduino IDE / arduino-cli):
//   LiquidCrystal_I2C 1.1.2
//   (MPU6050 TIDAK butuh library eksternal — dibaca langsung lewat Wire.h)
//
// Compile:
//   arduino-cli compile --fqbn esp32:esp32:esp32 PenghitungLangkahMpu6050
//
// Skema + repo: https://github.com/oerban70-cloud/penghitung-langkah-mpu6050

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Konfigurasi — semua angka yang bisa diubah ada di sini, bukan di tengah kode
// ---------------------------------------------------------------------------

// Nyalakan buzzer peringatan target atau tidak (false = tanpa buzzer)
#define USE_BUZZER true

// Pin bus I2C (pin default ESP32 DevKit)
const uint8_t PIN_I2C_SDA = 21;
const uint8_t PIN_I2C_SCL = 22;

// Pin buzzer aktif (HIGH = bunyi)
const uint8_t PIN_BUZZER = 25;

// Pin tombol reset (INPUT_PULLUP: tekan = LOW ke GND)
const uint8_t PIN_TOMBOL_RESET = 26;

// Alamat I2C
const uint8_t ADDR_MPU6050 = 0x68;  // pin AD0 modul ke GND
const uint8_t ADDR_LCD     = 0x27;  // alamat I2C LCD 16x2

// Ukuran LCD
const uint8_t LEBAR_LCD  = 16;  // jumlah kolom
const uint8_t TINGGI_LCD = 2;   // jumlah baris

// --- Register MPU6050 (datasheet) ---
const uint8_t REG_PWR_MGMT_1   = 0x6B;  // tulis 0 = bangun dari sleep
const uint8_t REG_WHO_AM_I     = 0x75;  // harus terbaca 0x68
const uint8_t NILAI_WHO_AM_I   = 0x68;  // nilai yang diharapkan
const uint8_t REG_ACCEL_XOUT_H = 0x3B;  // awal blok baca 6 byte
const uint8_t JUMLAH_BYTE_BACA = 6;     // accel X, Y, Z (2 byte tiap sumbu)

// --- Skala sensor (datasheet, rentang default +-2 g) ---
// Akselerometer +-2 g: 16384 LSB per g.
const float LSB_PER_G_MPU6050 = 16384.0;

// --- Deteksi langkah (peak detection pada besar percepatan) ---
// Percepatan diam = 1 g (gravitasi). Lonjakan di atas ambang ini = langkah.
// Ambang atas 0,25 g: cukup tinggi supaya goyangan tangan tidak terhitung,
// cukup rendah supaya langkah pelan tetap terhitung.
const float AMBANG_LANGKAH_G = 0.25;
// Ambang bawah 0,15 g: detektor baru siap lagi setelah guncangan mereda.
const float AMBANG_SIAP_G = 0.15;
// Jeda minimum antar langkah: 250 ms = paling cepat 4 langkah per detik.
// Langkah jalan normal sekitar 2 langkah per detik, jadi aman.
const unsigned long JEDA_MIN_ANTAR_LANGKAH_MS = 250;

// --- Target dan jarak ---
// Target harian yang memberi bunyi: 1000 langkah.
const uint32_t TARGET_LANGKAH = 1000;
// Panjang satu langkah rata-rata orang dewasa (meter).
const float PANJANG_LANGKAH_M = 0.70;

// Seberapa sering sensor dibaca dan LCD diperbarui (milidetik)
const unsigned long INTERVAL_BACA_MS = 50;   // 20 kali per detik
const unsigned long INTERVAL_LCD_MS  = 200;  // 5 kali per detik

// Debounce tombol reset (milidetik)
const unsigned long DEBOUNCE_TOMBOL_MS = 50;

// Durasi bunyi buzzer tiap peringatan target (milidetik)
const unsigned long DURASI_BUNYI_MS = 200;
const uint8_t JUMLAH_BUNYI_TARGET = 3;

// ---------------------------------------------------------------------------
// Objek global
// ---------------------------------------------------------------------------

LiquidCrystal_I2C lcd(ADDR_LCD, LEBAR_LCD, TINGGI_LCD);

// Jumlah langkah yang sudah terhitung
uint32_t jumlahLangkah = 0;

// Status detektor: true = siap mendeteksi lonjakan berikutnya
bool detektorSiap = true;

// Waktu langkah terakhir yang terhitung (milidetik)
unsigned long waktuLangkahTerakhirMs = 0;

// Penanda target sudah tercapai (supaya buzzer tidak bunyi terus)
bool targetTercapai = false;

// Waktu baca / LCD terakhir (milidetik)
unsigned long waktuBacaTerakhirMs = 0;
unsigned long waktuLcdTerakhirMs = 0;

// Status tombol reset (debounce)
bool tombolTerakhirDitekan = false;
unsigned long waktuTombolBerubahMs = 0;

// ---------------------------------------------------------------------------
// Tulis satu byte ke register MPU6050
// ---------------------------------------------------------------------------
void tulisRegisterMpu6050(uint8_t alamatRegister, uint8_t nilai) {
  Wire.beginTransmission(ADDR_MPU6050);
  Wire.write(alamatRegister);
  Wire.write(nilai);
  Wire.endTransmission();
}

// ---------------------------------------------------------------------------
// Baca satu byte dari register MPU6050.
// Mengembalikan -1 bila MPU6050 tidak menjawab.
// ---------------------------------------------------------------------------
int bacaRegisterMpu6050(uint8_t alamatRegister) {
  Wire.beginTransmission(ADDR_MPU6050);
  Wire.write(alamatRegister);
  if (Wire.endTransmission(false) != 0) {
    return -1;  // MPU6050 tidak menjawab
  }
  Wire.requestFrom(ADDR_MPU6050, (uint8_t)1);
  if (Wire.available() < 1) {
    return -1;  // data tidak datang
  }
  return Wire.read();
}

// ---------------------------------------------------------------------------
// Baca akselerometer dari MPU6050 (6 byte dari register 0x3B).
// Mengisi ax, ay, az dalam satuan g.
// Mengembalikan false bila MPU6050 tidak menjawab.
// ---------------------------------------------------------------------------
bool bacaAkselerometer(float &ax, float &ay, float &az) {
  Wire.beginTransmission(ADDR_MPU6050);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;  // MPU6050 tidak menjawab
  }
  Wire.requestFrom(ADDR_MPU6050, JUMLAH_BYTE_BACA);
  if (Wire.available() < JUMLAH_BYTE_BACA) {
    return false;  // data tidak lengkap
  }
  // Urutan byte: accel X (2 byte), accel Y (2 byte), accel Z (2 byte).
  int16_t mentahAx = (Wire.read() << 8) | Wire.read();
  int16_t mentahAy = (Wire.read() << 8) | Wire.read();
  int16_t mentahAz = (Wire.read() << 8) | Wire.read();

  ax = mentahAx / LSB_PER_G_MPU6050;
  ay = mentahAy / LSB_PER_G_MPU6050;
  az = mentahAz / LSB_PER_G_MPU6050;
  return true;
}

// ---------------------------------------------------------------------------
// Besar percepatan total (dalam g). Diam = 1 g (gravitasi bumi).
// ---------------------------------------------------------------------------
float besarPercepatan(float ax, float ay, float az) {
  return sqrt(ax * ax + ay * ay + az * az);
}

// ---------------------------------------------------------------------------
// Deteksi satu langkah dari besar percepatan.
// Mengembalikan true tepat saat satu langkah baru terhitung.
// ---------------------------------------------------------------------------
bool deteksiLangkah(float besarG, unsigned long sekarangMs) {
  // Lonjakan di atas ambang + jeda minimum sudah lewat = satu langkah.
  if (detektorSiap && (besarG - 1.0) > AMBANG_LANGKAH_G &&
      (sekarangMs - waktuLangkahTerakhirMs) >= JEDA_MIN_ANTAR_LANGKAH_MS) {
    detektorSiap = false;  // kunci sampai guncangan mereda
    waktuLangkahTerakhirMs = sekarangMs;
    return true;
  }
  // Guncangan sudah mereda di bawah ambang bawah = siap deteksi lagi.
  if (!detektorSiap && (besarG - 1.0) < AMBANG_SIAP_G) {
    detektorSiap = true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Jarak tempuh (meter) dari jumlah langkah.
// ---------------------------------------------------------------------------
float jarakTempuhMeter(uint32_t langkah) {
  return langkah * PANJANG_LANGKAH_M;
}

// ---------------------------------------------------------------------------
// Tampilkan jumlah langkah dan jarak di LCD.
// Baris 1: "Langkah: 1234" — Baris 2: "Jarak: 864 m"
// ---------------------------------------------------------------------------
void tampilkanLcd() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Langkah: ");
  lcd.print(jumlahLangkah);

  lcd.setCursor(0, 1);
  lcd.print("Jarak: ");
  lcd.print(jarakTempuhMeter(jumlahLangkah), 0);
  lcd.print(" m");
  if (targetTercapai) {
    lcd.print(" TARGET!");
  }
}

// ---------------------------------------------------------------------------
// Bunyikan buzzer beberapa kali (tanda target tercapai).
// ---------------------------------------------------------------------------
void bunyikanBuzzerTarget() {
  if (!USE_BUZZER) {
    return;
  }
  for (uint8_t i = 0; i < JUMLAH_BUNYI_TARGET; i++) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(DURASI_BUNYI_MS);
    digitalWrite(PIN_BUZZER, LOW);
    delay(DURASI_BUNYI_MS);
  }
}

// ---------------------------------------------------------------------------
// Cek tombol reset (debounce 50 ms). Tekan = hitung ulang dari nol.
// ---------------------------------------------------------------------------
void cekTombolReset(unsigned long sekarangMs) {
  bool ditekan = (digitalRead(PIN_TOMBOL_RESET) == LOW);
  if (ditekan != tombolTerakhirDitekan) {
    tombolTerakhirDitekan = ditekan;
    waktuTombolBerubahMs = sekarangMs;
  }
  // Tombol stabil ditekan selama debounce = reset sekali.
  if (ditekan && (sekarangMs - waktuTombolBerubahMs) >= DEBOUNCE_TOMBOL_MS &&
      jumlahLangkah > 0) {
    jumlahLangkah = 0;
    targetTercapai = false;
    detektorSiap = true;
    tampilkanLcd();
    Serial.println("Tombol reset ditekan: hitungan diulang dari nol.");
  }
}

// ---------------------------------------------------------------------------
// Tampilkan pesan error di LCD + Serial bila MPU6050 tidak ditemukan.
// ---------------------------------------------------------------------------
void tampilkanErrorMpu6050() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MPU6050 TIDAK");
  lcd.setCursor(0, 1);
  lcd.print("TERDETEKSI!");
  Serial.println("MPU6050 tidak terdeteksi di alamat 0x68.");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_TOMBOL_RESET, INPUT_PULLUP);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // Nyalakan LCD.
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Penghitung");
  lcd.setCursor(0, 1);
  lcd.print("Langkah...");

  // Bangunkan MPU6050 dari sleep (register PWR_MGMT_1 = 0).
  tulisRegisterMpu6050(REG_PWR_MGMT_1, 0);
  delay(100);

  // Periksa identitas chip (WHO_AM_I harus 0x68).
  int siapa = bacaRegisterMpu6050(REG_WHO_AM_I);
  if (siapa != NILAI_WHO_AM_I) {
    tampilkanErrorMpu6050();
    while (true) {
      delay(1000);  // MPU6050 tidak ada: tidak ada gunanya lanjut
    }
  }

  tampilkanLcd();
  waktuBacaTerakhirMs = millis();
  waktuLcdTerakhirMs = millis();
  Serial.println("Penghitung langkah siap. Jalan dan hitung langkahmu!");
}

void loop() {
  unsigned long sekarangMs = millis();

  cekTombolReset(sekarangMs);

  if (sekarangMs - waktuBacaTerakhirMs < INTERVAL_BACA_MS) {
    return;  // belum waktunya baca lagi
  }
  waktuBacaTerakhirMs = sekarangMs;

  float ax, ay, az;
  if (!bacaAkselerometer(ax, ay, az)) {
    tampilkanErrorMpu6050();
    return;
  }

  if (deteksiLangkah(besarPercepatan(ax, ay, az), sekarangMs)) {
    jumlahLangkah++;
    if (jumlahLangkah == TARGET_LANGKAH && !targetTercapai) {
      targetTercapai = true;
      bunyikanBuzzerTarget();
      Serial.println("Target 1000 langkah tercapai!");
    }
    tampilkanLcd();
    Serial.print("Langkah: ");
    Serial.print(jumlahLangkah);
    Serial.print("  Jarak: ");
    Serial.print(jarakTempuhMeter(jumlahLangkah), 0);
    Serial.println(" m");
    return;
  }

  // Perbarui LCD berkala (supaya jarak selalu tampil segar).
  if (sekarangMs - waktuLcdTerakhirMs >= INTERVAL_LCD_MS) {
    waktuLcdTerakhirMs = sekarangMs;
    tampilkanLcd();
  }
}
