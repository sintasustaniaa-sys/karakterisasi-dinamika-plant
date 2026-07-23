# Karakterisasi Dinamika Plant Hidroponik NFT dan Kontrol ANFIS

Repository ini berisi source code yang digunakan pada penelitian sistem hidroponik NFT tanaman cabai rawit berbasis ESP32 dan Artificial Neuro-Fuzzy Inference System (ANFIS).

## Isi Repository

| File | Deskripsi |
|------|-----------|
| `karakterisasi_dinamika_plant.ino` | Program karakterisasi dinamika plant hidroponik NFT |
| `ambil_data_training_anfis.ino` | Program pengambilan data untuk training ANFIS |
| `kontrol_anfis_tipe1.ino` | Program kontrol menggunakan ANFIS Tipe-1 |
| `ANFIS_T1_params_FIKSSS.h` | Parameter hasil training ANFIS Tipe-1 |
| `kontrol_it2_anfis.ino` | Program kontrol menggunakan Interval Type-2 ANFIS |
| `IT2_ANFIS_params_FIKSSS.h` | Parameter hasil training Interval Type-2 ANFIS |
| `training_anfis.py` | Program training ANFIS menggunakan PyTorch dan Nested Cross Validation |

## Hardware

- ESP32
- ADS1115
- Sensor pH DFRobot
- Sensor EC DFRobot
- Sensor DO DFRobot
- Sensor Suhu DS18B20
- Relay Module
- Pompa Nutrisi
- Pompa pH Up
- Pompa pH Down
- Aerator
- Kipas DC

## Software dan Library

### ESP32
- WiFi
- PubSubClient
- Adafruit ADS1X15
- OneWire
- DallasTemperature
- DFRobot_PH_ADS
- DFRobot_EC_ADS
- DFRobot_DO_ADS

### Python
- PyTorch
- NumPy
- Pandas

## Fitur

- Karakterisasi dinamika plant
- Pengambilan data training ANFIS
- Training ANFIS Tipe-1
- Training Interval Type-2 ANFIS
- Kontrol suhu
- Kontrol pH
- Kontrol Electrical Conductivity (EC)
- Kontrol Dissolved Oxygen (DO)
- Pengiriman data melalui MQTT dalam format JSON

## Catatan

Credential WiFi pada source code telah diganti dengan placeholder sebelum dipublikasikan.
