# Penjelasan Modul Verilog FPGA VSLAM

Dokumen ini menjelaskan fungsi dari setiap modul perangkat keras (Verilog) dalam *pipeline* VSLAM.

## Modul Image Pyramid
* **`orb_input_fifo.v`**: *Buffer* (antrean) sementara untuk menerima aliran piksel gambar input menggunakan protokol AXI-Stream. Memastikan kelancaran aliran data sebelum diproses.
* **`orb_pyr_scaler.v`**: Mesin *downscaler* (pengecil resolusi) yang mengubah ukuran gambar ke level berikutnya menggunakan interpolasi bilinear sederhana (dengan operasi *shift-add*, tanpa *multiplier*).
* **`orb_image_pyramid.v`**: Menyatukan beberapa `orb_pyr_scaler` secara berjenjang (*cascaded*). Menerima gambar asli (Level 0) dan menghasilkan aliran piksel untuk Level 1, Level 2, dan Level 3 secara *real-time*.

## Modul Ekstraksi Fitur (Frontend)
* **`orb_window7.v`**: *Line buffer* yang menahan aliran piksel secara serial untuk membangun jendela 7x7 utuh. Outputnya adalah array 392-bit yang dikirim ke FAST dan Gaussian.
* **`orb_fast_core.v`**: Mengevaluasi lingkaran Bresenham (radius 3) di dalam jendela 7x7 untuk mendeteksi sudut (corner). Menghasilkan validasi dan skor sudut untuk setiap piksel.
* **`orb_gauss7.v`**: Menerapkan filter *blur* Gaussian (kernel 7x7) pada jendela piksel yang sama. Dirancang bebas-DSP (*multiplier-free*), hanya menggunakan operasi *shift* (geser bit) dan *add* (tambah) dengan latensi 2 siklus.
* **`orb_nms3x3.v`**: Modul *Non-Maximum Suppression*. Menyimpan skor FAST ke dalam *line buffer* mandiri untuk membuat jendela 3x3, lalu menyeleksi *keypoint* yang skornya merupakan nilai tertinggi lokal (maksimum) di antara 8 tetangganya.
* **`orb_frontend_level.v`**: *Wrapper* (pembungkus) level tunggal. Modul ini menyatukan `orb_window7`, `orb_fast_core`, `orb_gauss7`, dan `orb_nms3x3` sehingga proses blur dan deteksi sudut berjalan serentak (*paralel*) pada satu level skala.

## Top-Level & Testbench
* **`orb_frontend_top.v`**: Modul puncak arsitektur. Modul ini menginstansiasi satu `orb_image_pyramid` dan empat `orb_frontend_level` (untuk L0, L1, L2, L3). Seluruh sistem beroperasi bersamaan dalam satu detak (*pixel clock*).
* **`TBFrontend.v`**: *Testbench* otomatis (*self-checking*). Membaca `level0_pixels.hex` sebagai input, memasukkannya ke *hardware*, lalu menangkap output `gauss`, `fast_score`, dan `kp` dari keempat level. Hasilnya dibandingkan secara bit-demi-bit (pada setiap siklus jam) dengan `.hex` referensi dari Python.
