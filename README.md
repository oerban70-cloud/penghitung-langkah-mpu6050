# Penghitung Langkah — ESP32 + MPU6050 + LCD I2C

Alat kecil penghitung langkah kaki yang kamu bawa saat jalan pagi. Setiap
kakimu menghentak tanah, modul MPU6050 merasakan guncangannya; ESP32
menghitung satu langkah dari tiap puncak guncangan, lalu menampilkan jumlah
langkah dan jarak tempuh di LCD 16x2. Begitu tembus 1000 langkah, buzzer
berbunyi sekali — tanda target pagimu tercapai. Tekan tombol reset untuk
menghitung ulang dari nol.

Video penjelasan (60 detik):
link menyusul setelah upload

## Bagaimana satu langkah ketahuan?

Saat berjalan, kakimu menghentak tanah dan badanmu naik-turun sedikit di
tiap langkah. Hentakan itu membuat besarnya percepatan total yang dibaca
MPU6050 melonjak di atas 1 g (gravitasi) lalu turun lagi. ESP32 membaca
akselerometer 20 kali per detik dan menghitung satu langkah tepat saat
lonjakan melewati ambang 0,25 g — dengan dua pengaman supaya hitungannya
tidak meleset:

1. **Jeda minimum 250 ms** antar langkah — satu hentakan yang sama tidak
   bisa terhitung dua kali. Langkah jalan normal sekitar 2 langkah per
   detik, jadi jeda ini masih longgar.
2. **Ambang bawah 0,15 g** — detektor baru siap lagi setelah guncangan
   mereda di bawah ambang ini, jadi getaran kecil tangan tidak ikut
   terhitung.

Jarak tempuh dihitung dari jumlah langkah dikali panjang langkah rata-rata
orang dewasa, 0,70 m per langkah. Modul dibaca langsung lewat I2C mentah
(Wire.h) — tanpa library MPU6050 — dan identitas chip diperiksa lewat
register WHO_AM_I (harus terbaca 0x68) saat dinyalakan.

## Komponen

| Bagian | Keterangan |
|---|---|
| ESP32 DevKit | Otak alat |
| MPU6050 | Modul IMU 6 sumbu, alamat I2C 0x68 |
| LCD 16x2 I2C (0x27) | Tampil jumlah langkah + jarak tempuh |
| Buzzer | Bunyi saat tembus target 1000 langkah |
| Tombol | Reset hitungan ke nol (INPUT_PULLUP) |

## Wiring

| Dari | Ke |
|---|---|
| MPU6050 SDA | ESP32 GPIO21 |
| MPU6050 SCL | ESP32 GPIO22 |
| MPU6050 VCC | ESP32 3V3 |
| LCD SDA / SCL | Bus I2C yang sama (GPIO21 / GPIO22) |
| LCD VCC | ESP32 VIN 5V |
| Buzzer + | ESP32 GPIO25 (aktif HIGH) |
| Buzzer - | GND |
| Tombol | ESP32 GPIO26 → GND |
| Semua GND | Digabung |

Skema lengkap: [schematic.png](schematic.png)

## Cara kerja

1. MPU6050 dibangunkan dari sleep, lalu identitas chip diperiksa
   (WHO_AM_I = 0x68). Kalau tidak ketemu, LCD menampilkan
   "MPU6050 TIDAK TERDETEKSI!".
2. Akselerometer dibaca tiap 50 ms; besar percepatan total dihitung
   dari ketiga sumbunya.
3. Lonjakan di atas ambang 0,25 g + jeda 250 ms sudah lewat = satu
   langkah baru. LCD langsung diperbarui: baris 1 jumlah langkah,
   baris 2 jarak tempuh dalam meter.
4. Jumlah langkah mencapai 1000 = buzzer berbunyi 3 kali dan LCD
   menampilkan "TARGET!" di baris 2.
5. Tekan tombol reset = hitungan kembali ke nol, target direset.

## Konfigurasi (di awal sketch)

- `USE_BUZZER` (true) — bunyikan buzzer saat target tercapai.
  Set false untuk tanpa buzzer.
- `TARGET_LANGKAH` (1000) — target langkah yang memberi bunyi.
- `PANJANG_LANGKAH_M` (0,70) — panjang satu langkahmu (meter).
  Ukur langkahmu sendiri lalu ganti angka ini supaya jaraknya akurat.
- `AMBANG_LANGKAH_G` (0,25) — ambang lonjakan yang dihitung sebagai
  langkah. Naikkan bila goyangan tangan ikut terhitung, turunkan bila
  langkah pelan tidak terhitung.
- `JEDA_MIN_ANTAR_LANGKAH_MS` (250) — jeda minimum antar langkah.
- `INTERVAL_BACA_MS` (50) — seberapa sering sensor dibaca.

## Tips pemakaian

- Pasang modul di saku celana atau ikat di pinggang supaya guncangan
  langkah terbaca jelas. Dipegang di tangan juga bisa, tapi goyangan
  tangan lebih berisik dan bisa menambah hitungan.
- Kalibrasi panjang langkah: jalan 10 langkah biasa, ukur jaraknya,
  bagi 10 — lalu isi ke `PANJANG_LANGKAH_M`.

## Library yang dibutuhkan

| Library | Versi | Keterangan |
|---|---|---|
| LiquidCrystal_I2C | 1.1.2 | LCD 16x2 I2C |
| (tidak ada) | — | MPU6050 dibaca langsung lewat Wire.h |

## Angka compile terverifikasi

- Sketch: 317.641 byte (24%) dari maksimum 1.310.720 byte
- RAM global: 24.640 byte (7%) dari maksimum 327.680 byte
- Board: ESP32 DevKit, ESP32 core 4.0.0, arduino-cli 1.6.0
- Terverifikasi compile 27 Nov 2026 (tanpa error)
