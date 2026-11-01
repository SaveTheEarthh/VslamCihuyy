"""
rbrief_only.py : model Python (golden model) untuk modul rBRIEF

Tugas modul rBRIEF:
    Untuk SATU keypoint, hasilkan descriptor 256-bit. Caranya: ambil 256 pasang
    titik (A, B) di sekitar keypoint, lalu bandingkan kecerahan A dan B. Kalau A
    lebih terang dari B, bit descriptor-nya 1, selain itu 0. Sebelum dibandingkan,
    pola titik diputar mengikuti arah (sudut) keypoint.

Input:
    window   : tabel 37x37, tiap isi = kecerahan piksel 6-bit (0..63).
               Sudah melewati Gaussian dan kuantisasi (dikerjakan modul lain).
    cos_val  : cos sudut keypoint, bilangan bulat bertanda, dikali 128 (-128..+128)
    sin_val  : sin sudut keypoint, bilangan bulat bertanda, dikali 128 (-128..+128)
               (cos_val dan sin_val dikirim oleh modul orientasi)

Output:
    descriptor 256-bit (bilangan bulat Python), bit ke-i berasal dari pasangan ke-i.

Tujuan file ini:
    1. Jadi acuan yang benar untuk membandingkan hasil RTL SystemVerilog.
    2. Membuat file vektor tes (.hex) yang dibaca testbench.

Cara pakai:
    1. Atur NAMA_GAMBAR di bagian PENGATURAN (di bawah import):
         NAMA_GAMBAR = None              -> window tes berisi angka acak
         NAMA_GAMBAR = "tes_gambar.jpg"  -> window tes dipotong dari gambar itu
                                            (butuh: pip install numpy opencv-python)
    2. Jalankan file ini (tombol Run, atau: python rbrief_only.py)
    Hasilnya folder vectors/ (di samping file ini) berisi pattern_rom.hex,
    windows.hex, params.hex, dan expected.hex
"""
import math
import os
import random

# =============================================================================
# PENGATURAN (satu-satunya yang perlu kamu ubah)
# =============================================================================
# Sumber window untuk vektor tes:
#   NAMA_GAMBAR = None              -> tanpa gambar, window berisi angka acak
#   NAMA_GAMBAR = "tes_gambar.jpg"  -> window dipotong dari gambar ini
# Cukup nama file kalau gambarnya satu folder dengan file .py ini. Kalau gambarnya
# di tempat lain, tulis path lengkap, misalnya r"C:\folder\tes_gambar.jpg".
NAMA_GAMBAR = "C:\\Users\\kenny\\OneDrive\\Documents\\C\\VSLAM\\tes_gambar.jpg"

# Folder tempat file ini berada. Folder vectors/ dan gambar dicari/dibuat di sini,
# jadi hasilnya selalu di samping file .py, apa pun folder kerja VS Code.
FOLDER_SKRIP = os.path.dirname(os.path.abspath(__file__))

# =============================================================================
# 1. KONSTANTA
# =============================================================================
HALF = 18               # jarak dari pusat window ke tepinya (piksel).
                        # Koordinat relatif titik ada di rentang -18..+18.
WIN = 2 * HALF + 1      # lebar window = 18 kiri + 1 pusat + 18 kanan = 37
N_BITS = 256            # jumlah pasangan titik = jumlah perbandingan = panjang descriptor
SCALE_BITS = 7          # cos dan sin dikali 2^7 = 128 agar menjadi bilangan bulat.
                        # Contoh: cos = 0,866 disimpan sebagai 111 (0,866 x 128).


# =============================================================================
# 2. POLA TITIK (di hardware disimpan sebagai ROM)
# =============================================================================
def make_pattern(seed=42):
    """Buat 256 pasangan titik (A, B). Tiap titik berbentuk (x, y) relatif
    terhadap pusat keypoint, dengan x dan y antara -13 dan +13.

    Return: list berisi 256 elemen, masing-masing ((x1, y1), (x2, y2)).
        (x1, y1) = titik A, (x2, y2) = titik B.

    Catatan: ini pola acak sementara untuk keperluan verifikasi hardware.
    Untuk SLAM sungguhan, ganti dengan tabel bit_pattern_31_ dari orb.cpp (OpenCV).
    """
    rng = random.Random(seed)       # seed tetap -> pola selalu sama tiap dijalankan
    pattern = []
    for _ in range(N_BITS):
        titik_a = (rng.randint(-13, 13), rng.randint(-13, 13))
        titik_b = (rng.randint(-13, 13), rng.randint(-13, 13))
        pattern.append((titik_a, titik_b))
    return pattern


# =============================================================================
# 3. INTI rBRIEF
# =============================================================================
def rotate(x, y, c, s):
    """Putar satu titik (x, y) sebesar sudut yang diwakili c (cos) dan s (sin).

    c dan s sudah dikali 128, jadi hasil perkalian juga 128 kali lebih besar
    dari nilai sebenarnya. Geser kanan sebanyak SCALE_BITS (>> 7) berarti dibagi
    128, yang mengembalikan hasil ke satuan piksel.

    Return: (xr, yr) = koordinat titik setelah diputar (bilangan bulat).
    """
    # Penambah pembulatan. Geser kanan selalu membulatkan ke bawah. Dengan
    # menambah 0,5 sebelum dibagi, hasilnya menjadi pembulatan ke bilangan bulat
    # terdekat. Karena kita bekerja dalam skala 128, 0,5 ditulis sebagai 64.
    round_offset = 1 << (SCALE_BITS - 1)        # = 64

    # Rumus rotasi 2D:  x' = x*cos - y*sin     y' = x*sin + y*cos
    # Python menggeser bilangan negatif secara aritmetik (>>), sama dengan >>>
    # pada variabel signed di SystemVerilog.
    xr = (x * c - y * s + round_offset) >> SCALE_BITS
    yr = (x * s + y * c + round_offset) >> SCALE_BITS
    return xr, yr


def rbrief(window, pattern, cos_val, sin_val):
    """Hitung descriptor 256-bit untuk satu keypoint.

    window  : tabel 37x37, diakses sebagai window[baris][kolom].
              Baris 0 = paling atas (y = -18), kolom 0 = paling kiri (x = -18).
    pattern : 256 pasangan titik dari make_pattern().
    cos_val, sin_val : arah keypoint (lihat keterangan di atas file).

    Return: bilangan bulat 256-bit. Bit ke-i bernilai 1 kalau titik A pasangan
            ke-i lebih terang dari titik B, selain itu 0.
    """
    desc = 0    # descriptor mulai dari 256 bit nol

    # Ulangi untuk setiap pasangan titik. i = nomor pasangan = nomor bit.
    for i, ((x1, y1), (x2, y2)) in enumerate(pattern):

        # Langkah 1: putar titik A dan titik B. cos_val dan sin_val sama untuk
        # seluruh 256 pasangan, karena sudutnya milik keypoint, bukan milik pasangan.
        xa, ya = rotate(x1, y1, cos_val, sin_val)
        xb, yb = rotate(x2, y2, cos_val, sin_val)

        # Langkah 2: geser titik pusat dari (0,0) ke (18,18). Koordinat relatif
        # bisa negatif (-18..+18), sedangkan indeks tabel harus 0..36.
        # c = kolom (dari x), r = baris (dari y).
        kolom_a, baris_a = xa + HALF, ya + HALF
        kolom_b, baris_b = xb + HALF, yb + HALF

        # Langkah 3: pengaman. Pastikan keempat indeks ada di dalam window (0..36).
        # Secara matematis tidak akan pernah gagal (lihat check_range). Tanpa ini,
        # indeks negatif di Python diam-diam membaca dari belakang tabel.
        assert 0 <= kolom_a < WIN and 0 <= baris_a < WIN, f"titik A di luar window (pasangan {i})"
        assert 0 <= kolom_b < WIN and 0 <= baris_b < WIN, f"titik B di luar window (pasangan {i})"

        # Langkah 4: bandingkan kecerahan. Urutan indeks: [baris][kolom].
        # Kalau A lebih terang dari B, nyalakan bit ke-i. Kalau sama terang
        # atau A lebih gelap, bit tetap 0.
        if window[baris_a][kolom_a] > window[baris_b][kolom_b]:
            desc |= 1 << i      # 1 << i = angka yang hanya bit ke-i bernilai 1

    return desc


# =============================================================================
# 4. BANTUAN UNTUK TES (bukan bagian modul rBRIEF)
#    Bagian ini hanya membuat data dummy untuk menguji RTL. Pada sistem asli,
#    window datang dari modul Gaussian, dan cos/sin dari modul orientasi.
# =============================================================================
def sample_trig_pairs():
    """Buat semua pasangan (cos, sin) yang mungkin dikirim modul orientasi.

    Lingkaran dibagi 64 sektor. Cukup dibuat 16 sektor untuk kuadran pertama
    (0..90 derajat), lalu kuadran lain didapat dengan membalik tanda.
    Total: 16 sektor x 4 kombinasi tanda = 64 pasangan.
    """
    pairs = []
    for k in range(16):                                 # k = nomor sektor
        # Sudut tengah sektor ke-k. Tiap sektor selebar 90/16 = 5,625 derajat.
        sudut = math.radians((k + 0.5) * 90.0 / 16)
        # Dikali 128 lalu dibulatkan menjadi bilangan bulat.
        c = round(math.cos(sudut) * 128)
        s = round(math.sin(sudut) * 128)
        # Variasi tanda mewakili keempat kuadran.
        for tanda_c in (1, -1):
            for tanda_s in (1, -1):
                pairs.append((tanda_c * c, tanda_s * s))
    return pairs


def check_range():
    """Buktikan bahwa titik hasil rotasi tidak pernah keluar dari window.

    Coba semua titik di kotak 27x27 (-13..+13) dengan semua 64 pasangan sudut,
    lalu catat koordinat hasil rotasi yang paling jauh dari pusat.
    Harus <= 18 (batas window 37x37).
    """
    terjauh = 0
    for c, s in sample_trig_pairs():
        for x in range(-13, 14):
            for y in range(-13, 14):
                xr, yr = rotate(x, y, c, s)
                terjauh = max(terjauh, abs(xr), abs(yr))
    assert terjauh <= HALF, "ada titik yang keluar dari window"
    return terjauh


def s5(v):
    """Ubah bilangan bulat (boleh negatif) menjadi 5 bit two's complement.
    Contoh: -5 -> 0b11011 (27). Format ini dipakai hardware untuk bilangan negatif."""
    return v & 0x1F


def s9(v):
    """Sama seperti s5, tetapi 9 bit (untuk cos dan sin yang rentangnya -128..+128)."""
    return v & 0x1FF


def dump_pattern_rom(pattern, path):
    """Tulis isi ROM pola ke file hex (dibaca $readmemh di SystemVerilog).

    Satu baris = satu pasangan titik. Empat koordinat (x1, y1, x2, y2) masing-masing
    5 bit digabung menjadi 20 bit, ditulis sebagai 5 digit hex.
    """
    with open(path, "w") as f:
        for (x1, y1), (x2, y2) in pattern:
            word = (s5(x1) << 15) | (s5(y1) << 10) | (s5(x2) << 5) | s5(y2)
            f.write(f"{word:05X}\n")


def load_image_quantized(path):
    """Baca gambar lalu siapkan seperti yang dikerjakan modul Gaussian.

    Langkah: ubah ke grayscale (0..255) -> haluskan dengan Gaussian binomial 7x7
    (kernel dibagi 4096, paper Fig. 9) -> kuantisasi ke 6 bit (buang 2 bit bawah,
    paper Section 5.2).

    Return: array numpy 2D berisi piksel 0..63, seukuran gambar asli.
    Butuh numpy dan opencv-python. Pembulatan Gaussian di sini dibuang pecahannya
    (>> 12); samakan dengan modul Gaussian temanmu kalau berbeda.
    """
    import numpy as np
    import cv2

    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f"gambar tidak bisa dibaca: {path}")
    if min(img.shape) < WIN:
        raise SystemExit(f"gambar terlalu kecil, minimal {WIN}x{WIN} piksel")

    # Kernel 7x7 = perkalian luar dari baris [1 6 15 20 15 6 1]; jumlahnya 4096.
    k1d = np.array([1, 6, 15, 20, 15, 6, 1], dtype=np.int32)
    k2d = np.outer(k1d, k1d)

    # Konvolusi: untuk tiap piksel, jumlahkan 49 tetangganya dikali bobot kernel.
    # Tepi gambar diperluas dengan mengulang piksel terluar (mode "edge").
    tinggi, lebar = img.shape
    padded = np.pad(img.astype(np.int32), 3, mode="edge")
    acc = np.zeros((tinggi, lebar), dtype=np.int32)
    for dy in range(7):
        for dx in range(7):
            acc += k2d[dy, dx] * padded[dy:dy + tinggi, dx:dx + lebar]

    halus = acc >> 12       # bagi 4096 (kembali ke rentang 0..255)
    return halus >> 2       # 8 bit -> 6 bit (0..63)


def generate_vectors(pattern, n, outdir, seed=1, image_q=None):
    """Buat n vektor tes dan tulis ke tiga file di folder outdir.

    Satu vektor tes = satu keypoint, berisi:
        - window 37x37                 -> windows.hex  (1369 baris per tes)
        - sepasang (cos, sin) acak     -> params.hex   (1 baris per tes)
        - descriptor yang benar        -> expected.hex (1 baris per tes)

    image_q = None      : window diisi angka acak 0..63.
    image_q = array 2D  : window dipotong dari gambar (hasil load_image_quantized)
                          di posisi keypoint acak.
    """
    os.makedirs(outdir, exist_ok=True)
    rng = random.Random(seed)
    pairs = sample_trig_pairs()

    with open(f"{outdir}/windows.hex", "w") as fw, \
         open(f"{outdir}/params.hex", "w") as fp, \
         open(f"{outdir}/expected.hex", "w") as fe:

        for _ in range(n):
            # a) Window. Dua sumber:
            if image_q is None:
                # Window dummy: 37x37 piksel acak 0..63.
                window = [[rng.randint(0, 63) for _ in range(WIN)] for _ in range(WIN)]
            else:
                # Window dari gambar: pilih titik pusat (keypoint) acak yang
                # jaraknya minimal 18 piksel dari tepi, lalu potong 37x37 di sekitarnya.
                tinggi, lebar = image_q.shape
                kx = rng.randint(HALF, lebar - HALF - 1)
                ky = rng.randint(HALF, tinggi - HALF - 1)
                window = image_q[ky - HALF: ky + HALF + 1,
                                 kx - HALF: kx + HALF + 1].tolist()

            # b) Sudut dummy: pilih satu pasangan (cos, sin) secara acak.
            c, s = rng.choice(pairs)

            # c) Jawaban yang benar, dihitung oleh model rBRIEF di atas.
            desc = rbrief(window, pattern, c, s)

            # d) Tulis ke file.
            # windows.hex: satu piksel per baris, urut baris demi baris (row-major).
            for row in window:
                for px in row:
                    fw.write(f"{px:02X}\n")
            # params.hex: sin di 9 bit atas, cos di 9 bit bawah -> {sin[8:0], cos[8:0]}.
            fp.write(f"{(s9(s) << 9) | s9(c):05X}\n")
            # expected.hex: descriptor 256-bit = 64 digit hex. Bit ke-i descriptor
            # adalah bit ke-i dari angka ini.
            fe.write(f"{desc:064X}\n")


# =============================================================================
# 5. PROGRAM UTAMA (jalan saat file dieksekusi langsung)
# =============================================================================
if __name__ == "__main__":
    # a) Cek bahwa titik hasil rotasi tidak pernah keluar window (harus 18).
    print("koordinat terjauh setelah rotasi:", check_range())

    # b) Kalau NAMA_GAMBAR diisi, siapkan gambarnya (Gaussian + 6 bit).
    #    Kalau None, gambar tidak dipakai dan window tes berisi angka acak.
    gambar = None
    if NAMA_GAMBAR is not None:
        # os.path.join(FOLDER_SKRIP, ...) mencari di folder file ini; kalau
        # NAMA_GAMBAR sudah berupa path lengkap, path itu yang dipakai.
        path_gambar = os.path.join(FOLDER_SKRIP, NAMA_GAMBAR)
        gambar = load_image_quantized(path_gambar)
        print(f"gambar dipakai: {path_gambar} (ukuran {gambar.shape[1]}x{gambar.shape[0]})")

    # c) Buat pola titik, lalu simpan sebagai isi ROM.
    pattern = make_pattern()
    folder_vektor = os.path.join(FOLDER_SKRIP, "vectors")
    os.makedirs(folder_vektor, exist_ok=True)
    dump_pattern_rom(pattern, os.path.join(folder_vektor, "pattern_rom.hex"))

    # d) Buat 100 vektor tes (input + jawaban yang benar).
    generate_vectors(pattern, 100, folder_vektor, image_q=gambar)
    sumber = "potongan gambar" if gambar is not None else "window acak"
    print(f"100 vektor tes dari {sumber} ditulis ke {folder_vektor}")

    # e) Tampilkan descriptor dari tes ke-0 (baca kembali dari expected.hex).
    with open(os.path.join(folder_vektor, "expected.hex")) as f:
        desc0 = int(f.readline(), 16)
    biner = f"{desc0:0256b}"
    print()
    print("Descriptor tes ke-0")
    print("  hex             :", f"{desc0:064X}")
    print("  jumlah bit '1'  :", bin(desc0).count("1"), "dari", N_BITS)
    print("  biner (bit ke-255 di kiri, bit ke-0 di kanan):")
    for i in range(0, N_BITS, 64):
        print("   ", biner[i:i + 64])
