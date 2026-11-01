"""
trig_decoder_golden.py : jawaban kunci untuk modul trig_decoder.sv

Untuk semua 64 kombinasi (kuadran 0..3) x (sektor 0..15), hitung cos dan sin
langsung dari sudutnya dengan math.cos dan math.sin (BUKAN dari tabel), lalu
tulis ke vectors/trig_decoder.hex. Karena cara hitungnya berbeda dari modul
SystemVerilog (yang memakai tabel dan pembalik tanda), hasilnya bisa dipakai
untuk mengecek modul itu.

Arti kuadran dan sektor mengikuti modul orientasi (patch_visual.html):
    a = (sektor + 0,5) * 5,625 derajat, diukur dari sumbu horizontal.
    kuadran 0 -> sudut sebenarnya 180 - a
    kuadran 1 -> sudut sebenarnya       a
    kuadran 2 -> sudut sebenarnya 360 - a
    kuadran 3 -> sudut sebenarnya 180 + a
    (sudut pada bingkai gambar: 0 = kanan, 90 = bawah)

Cara pakai: jalankan file ini (tombol Run juga bisa).
"""
import math
import os

FOLDER_SKRIP = os.path.dirname(os.path.abspath(__file__))

# Contoh dari patch_visual.html: (kuadran, sektor, sudut yang ditampilkan modul
# orientasi, dalam derajat). Dipakai untuk memastikan terjemahan di sini sama
# dengan yang dimaksud modul orientasi.
CONTOH_TEMAN = [
    (1, 0, 2.81), (1, 8, 47.81), (0, 11, 115.31), (3, 4, 205.31),
    (2, 12, 289.69), (2, 5, 329.06), (0, 15, 92.81),
]


def sudut_sebenarnya(quadrant, sector):
    """Sudut arah keypoint dalam derajat (0 = kanan, 90 = bawah)."""
    a = (sector + 0.5) * 90.0 / 16          # tiap sektor selebar 90/16 = 5,625
    return [180 - a, a, 360 - a, 180 + a][quadrant]


def decode_trig(quadrant, sector):
    """Return (cos, sin), masing-masing dikali 128 dan dibulatkan."""
    sudut = math.radians(sudut_sebenarnya(quadrant, sector))
    return round(math.cos(sudut) * 128), round(math.sin(sudut) * 128)


def s9(v):
    """Bilangan bulat (boleh negatif) -> 9 bit two's complement."""
    return v & 0x1FF


if __name__ == "__main__":
    # a) Cocokkan dengan contoh dari modul orientasi temanmu.
    for q, k, sudut_tampil in CONTOH_TEMAN:
        c, s = decode_trig(q, k)
        sudut_balik = math.degrees(math.atan2(s, c)) % 360   # dari cos/sin kembali ke sudut
        selisih = abs(sudut_balik - sudut_tampil)
        assert selisih < 1.0, f"kuadran {q} sektor {k}: {sudut_balik:.2f} vs {sudut_tampil}"
    print(f"{len(CONTOH_TEMAN)} contoh dari modul orientasi cocok (selisih < 1 derajat)")

    # b) Tulis semua 64 kombinasi sebagai vektor tes.
    folder = os.path.join(FOLDER_SKRIP, "vectors")
    os.makedirs(folder, exist_ok=True)
    hasil = []
    with open(os.path.join(folder, "trig_decoder.hex"), "w") as f:
        for q in range(4):
            for k in range(16):
                c, s = decode_trig(q, k)
                hasil.append((c, s))
                # Satu baris 24 bit = {quadrant[1:0], sector[3:0], cos[8:0], sin[8:0]}
                f.write(f"{(q << 22) | (k << 18) | (s9(c) << 9) | s9(s):06X}\n")
    print(f"64 kombinasi ditulis ke {folder}")

    # c) 64 pasangan (cos, sin) ini harus sama dengan himpunan pasangan yang dipakai
    #    untuk menguji rbrief (sample_trig_pairs di rbrief_only.py).
    try:
        from rbrief_only import sample_trig_pairs
        print("sama dengan pasangan yang dipakai di tes rbrief:",
              set(hasil) == set(sample_trig_pairs()))
    except ImportError:
        print("(rbrief_only.py tidak ada di folder ini, cek himpunan dilewati)")
