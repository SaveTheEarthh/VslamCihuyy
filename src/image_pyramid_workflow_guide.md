# Panduan Alur Kerja Sistem Frontend VSLAM (Python & Verilog)

Panduan ini menjelaskan cara menjalankan seluruh *pipeline* Frontend VSLAM, mulai dari ekstraksi gambar (Python) hingga simulasi *hardware* (Pyramid, FAST, Gaussian, NMS) di Verilog.

## Langkah 1: Menghasilkan Data Referensi (Golden Data) dengan Python
Skrip Python digunakan untuk memproses gambar asli dan menghasilkan file `.hex` yang akan menjadi input (piksel) dan output referensi (hasil yang diharapkan) untuk memverifikasi *hardware*.

1. Buka terminal dan masuk ke direktori Python:
   `cd "pyCodes/"`
2. Hasilkan *golden data* untuk Image Pyramid:
   `python3 orb_pyramid_golden.py`
3. Hasilkan *golden data* untuk FAST, Gaussian, dan NMS:
   `python3 orb_frontend_golden.py`
4. Pastikan file referensi telah berhasil dibuat di folder `pyCodes/imageTest/` (seperti `level0_pixels.hex`, `level0_gauss.hex`, `level0_kp.hex`, dll).

## Langkah 2: Kompilasi Sistem Hardware di Verilog
Modul Verilog akan memproses file input dari Python dan menghasilkan outputnya sendiri.

1. Buka terminal dan masuk ke direktori Verilog:
   `cd "../Verilog/"`
2. Kompilasi seluruh *pipeline* Frontend dengan Icarus Verilog:
   `iverilog -o frontend_sim.vvp TBFrontend.v orb_input_fifo.v orb_pyr_scaler.v orb_image_pyramid.v orb_window7.v orb_fast_core.v orb_nms3x3.v orb_gauss7.v orb_frontend_level.v orb_frontend_top.v`

## Langkah 3: Menjalankan Simulasi
Simulasi (Testbench) secara otomatis akan membandingkan hasil keluaran Verilog dengan *golden data* dari Python.

1. Jalankan simulasi:
   `vvp frontend_sim.vvp`
2. Abaikan peringatan `WARNING: Not enough words in the file`. Ini normal karena alokasi memori tes (*keypoint depth*) dibuat lebih besar dari jumlah *keypoint* aktual gambar.
3. Tunggu hingga proses selesai. Jika sistem berjalan dengan sempurna, terminal akan menampilkan pesan:
   `*** PASS: all levels match the golden model ***`
