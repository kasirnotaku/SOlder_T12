# Modifikasi — Soldering Station T12 OLED

Firmware: `Soldering station T12  OLED .ino` (basis v1.7).

## 1. Perbaikan compile
- Include mati `Adafruit_SH1106.h`, `Adafruit_SSD1305.h`, `splash.h` dihapus
  (display memakai `U8glib`, bukan Adafruit).
- `InputNameScreen()` (void) tidak lagi `return value;`.
- Wajib install via Library Manager: `U8glib` (olikraus) + `PID` (Brett Beauregard).

## 2. Factory Reset + EEPROM wipe
- Menu `Setup > Factory Reset` → konfirmasi `Are you sure?` → seluruh EEPROM
  ditulis `0xFF` (kondisi erased pabrik) pakai `EEPROM.update` (hemat write cycle),
  lalu tulis ulang identifier + default → pesan `Reset Done` → auto-reboot via watchdog.
- `wdt_disable()` di awal `setup()` agar tidak boot-loop setelah watchdog reset.

## 3. Layar utama: thermometer + strip OFF (kedua mode)
- **Thermometer tanpa frame** di kedua mode: bohlam di bawah (outline +
  merkuri) + **6 kotak bertingkat** di atasnya yang menyala dari bawah ke
  atas mengikuti `heatPct`, tanpa tumpang tindih. Via fungsi bersama
  `drawThermo()` (primitif U8glib, tanpa bitmap).
- Big Numbers: angka `fub42n` 42px **rata kanan** (x=128), y18–60.
- More Infos: nama tip digeser `0→12` agar tidak tertutup thermometer;
  Vin dan posisi suhu tetap.
- Saat heater **OFF**: angka diganti **tiga strip** (`drawBox`, tanpa font
  sehingga aman) — besar di Big Numbers, menyesuaikan di More Infos.

## 4. Kalibrasi (bug bawaan → 1-point)
Bug bawaan yang diperbaiki:
- Masuk kalibrasi saat sleep/off mengunci `Setpoint` → stuck `Please wait...`
  selamanya. Sekarang kalibrasi selalu clear `inSleep/inOff/inBoost`.
- `SENSORCheck()` bisa membuka `ChangeTipScreen` nested di tengah kalibrasi.
  Sekarang ditahan via flag `inCalibMode` (yang tadinya tidak dipakai).
- Rotary step 1 terlalu seret → step 5.

Metode akhir (**satu-satunya**, 3-point bawaan dihapus):
- `Tip Settings > Calibrate Tip`: panaskan ke `SetTemp` yang sedang dipakai
  (misal set 400 dulu di layar utama), tunggu stabil otomatis (±4°C/6 detik,
  `beep beep` = READY), tampil `T:` target, `C:` terbaca, `U:` ukur, `A:` ADC.
- Tempel termometer eksternal, putar sampai `U:` = suhu ujung real,
  tekan pendek = hitung `offset = ukur − CurrentTemp`, geser `Cal[0..2]`,
  clamp 20–500, cek monotonik, konfirmasi `Store?`. Tahan 1 detik = batal.
  Timeout safety 5 menit, heater off saat keluar.

## 5. Batas suhu 450°C
- `TEMP_MAX` 400 → 450 (semua batas putaran ikut otomatis).
- Ambang tip-terlepas jadi konstanta `TEMP_TIP_ERR 600` (7 titik),
  karena 450 + Boost 100 = 550 menyentuh ambang lama 500 (false `ERROR`).
  Tip tercabut asli terbaca ~1000+, jadi 600 tetap aman.

## 6. Mode 3 channel (CH1/CH2/CH3)
- `PresetTemp[3]` default `300/350/400`, `ActiveChannel`, tersimpan di EEPROM
  slot tetap `900–906` (di luar area data tip yang panjangnya variabel).
  Validasi range saat baca → byte lama/`0xFF` otomatis jadi default,
  jadi **kalibrasi & setting lama tidak hilang**.
- Layar utama tampil `CH1:`/`CH2:`/`CH3:` + setpoint.
- `DefaultTemp` dihapus total (variabel, menu, byte EEPROM 2–3 jadi legacy)
  karena digantikan preset; `Temp Settings` tinggal Sleep/Boost.
- Boot **selalu di CH1** (`ActiveChannel = 0` paksa di `setup()`; hanya RAM,
  EEPROM tidak ditulis). Nilai CH1 mengikuti simpanan (default 300 atau hasil
  edit + tahan-3-detik terakhir). Inisialisasi `c0/d0` yang hilang
  diperbaiki (klik pertama tidak lagi diabaikan).

## 7. Peta tombol (final)
| Aksi | Fungsi |
|---|---|
| Putar | Ubah suhu channel aktif (RAM) |
| Klik | Boost on/off |
| Double-click (< 350 ms) | Pindah CH1→CH2→CH3 (bip 1/2/3×) |
| Tahan 0,5–3 dtk | Setup Menu |
| Tahan 3 dtk | **Simpan channel** → beep 2 detik (heater off sesaat, tik tiap detik) |

Putaran tanpa tahan tidak menulis EEPROM; ganti channel & masuk/keluar
Setup ikut menyimpan nilai tertunda sebagai jaring pengaman.

## 8. Pembersihan kode mati (hemat flash)
Dihapus: include Adafruit, `CalibrationScreen()` 3-point, menu `Preset`
+ `PresetScreen()`, array `PresetItems/CHItems/CHxTempItems`, global
`Step` & `ChipTemp` (tak terbaca), komentar/kode mati. Fungsi yang tersisa
semuanya terpakai dan terverifikasi braces seimbang.

## 11. Kompresi flash (storage 95%)
- **Hapus semua `print(float)`**: setpoint, Vin/Vcc/Tmp kini dicetak via
  aritmetika integer (mis. `Vin/1000` + `"."` + `(Vin%1000)/100`) — melepas
  linker formatter float (`dtostrf`) yang mahal ±2KB. Perilaku & tampilan sama.
- **`snprintf` → `utoa`** untuk buffer angka kanan (avr-libc, jauh lebih kecil).
- **Baris mati dicabut**: `TipIsPresent=true` per-loop + variabel/update
  `lastbutton` di QuickCalib (tip-detect sudah ditahan via `inCalibMode`).
- Aritmetika float tetap dipakai di PID & sensor (kecil, dibutuhkan).

## 9. Layout EEPROM
- `0–1`: identifier `0xE76C` · `2–3`: legacy (ex DefaultTemp) ·
  `4–5`: SleepTemp · `6`: BoostTemp · `7`: time2sleep ·
  `8`: time2off · `9`: timeOfBoost · `10`: MainScrType · `11`: PIDenable ·
  `12`: beepEnable · `13`: CurrentTip · `14`: NumberOfTips ·
  `15+`: data tip (6 nama + 8 kalibrasi per tip) ·
  `900–905`: CH1–CH3 · `906`: ActiveChannel.

## 10. Data hardware tercatat
Vin 24,5 V, Vcc 5,0 V (supply 24 V 3 A+, divider & referensi ADC benar).
