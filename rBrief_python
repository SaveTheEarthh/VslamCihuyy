import cv2
import math
import random
import numpy as np

# --- KONSTANTA ---
PATCH_SIZE = 37
PATTERN_SIZE = 256

def load_orb_pattern_dataset():
    """
    Mensimulasikan dataset 256 pasang titik koordinat rBRIEF.
    Titik-titik ini dibatasi pada rentang -13 sampai 13 (area 27x27).
    """
    pattern_dataset = []
    
    # Kita kunci (seed) angka acaknya agar pola titik selalu sama 
    # setiap kali program dijalankan (mensimulasikan memori ROM di FPGA).
    random.seed(42) 
    
    for _ in range(PATTERN_SIZE):
        x1, y1 = random.randint(-13, 13), random.randint(-13, 13)
        x2, y2 = random.randint(-13, 13), random.randint(-13, 13)
        
        # Simpan sebagai tuple bersarang: ((x1, y1), (x2, y2))
        pattern_dataset.append(((x1, y1), (x2, y2)))
        
    return pattern_dataset

def compute_rbrief(image_patch, pattern_dataset, theta):
    """
    Fungsi utama untuk mengekstrak deskriptor rBRIEF 256-bit (32 Byte).
    """
    # Siapkan wadah 32 list yang isinya angka 0 semua
    descriptor = [0] * 32
    
    # Hitung cos dan sin dari theta satu kali saja 
    # (Di FPGA, ini simulasi mengambil nilai dari tabel LUT)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    for i in range(PATTERN_SIZE):
        # Ambil alamat Titik A dan Titik B dari dataset
        (x1, y1), (x2, y2) = pattern_dataset[i]

        # 1. ROTATOR: Putar alamat titik koordinat
        xr1 = round(x1 * cos_t - y1 * sin_t)
        yr1 = round(x1 * sin_t + y1 * cos_t)
        xr2 = round(x2 * cos_t - y2 * sin_t)
        yr2 = round(x2 * sin_t + y2 * cos_t)

        # Geser titik pusat koordinat dari (0,0) ke indeks positif array (digeser +18)
        cx1 = xr1 + (PATCH_SIZE // 2)
        cy1 = yr1 + (PATCH_SIZE // 2)
        cx2 = xr2 + (PATCH_SIZE // 2)
        cy2 = yr2 + (PATCH_SIZE // 2)

        # 2. GENERATOR: Cek batas aman agar tidak "Index out of range"
        if (0 <= cx1 < PATCH_SIZE and 0 <= cy1 < PATCH_SIZE and
            0 <= cx2 < PATCH_SIZE and 0 <= cy2 < PATCH_SIZE):
            
            # Bandingkan! Jika Titik A lebih terang dari Titik B, simpan nilai 1
            if image_patch[cy1][cx1] > image_patch[cy2][cx2]:
                
                # Masukkan bit '1' ke dalam byte yang tepat (i // 8)
                # dan geser posisinya (shift) ke urutan yang tepat (i % 8)
                descriptor[i // 8] |= (1 << (i % 8))

    return descriptor

# ==========================================
# BAGIAN UTAMA (MAIN PROGRAM)
# ==========================================
if __name__ == "__main__":
    print("=== PROTOTIPE EKSTRAKTOR rBRIEF ===")
    
    # 1. PERSIAPAN DATA GAMBAR (Simulasi data dari modul FAST)
    nama_file = r'c:\Users\kenny\OneDrive\Documents\C\VSLAM\tes_gambar.jpg'
    img_full = cv2.imread(nama_file, cv2.IMREAD_GRAYSCALE)
    
    patch_sungguhan_list = []
    titik_x, titik_y = 150, 200 # Pura-puranya ini titik yang ditemukan modul FAST

    # Cek apakah file gambar sungguhan ada di folder
    if img_full is not None:
        print(f"\n[INFO] Menggunakan gambar asli '{nama_file}' (Ukuran: {img_full.shape})")
        
        # Cek keamanan area potong (pastikan titik tidak terlalu mepet pinggir)
        if (titik_y >= 18 and titik_y + 19 <= img_full.shape[0] and
            titik_x >= 18 and titik_x + 19 <= img_full.shape[1]):
            
            # Potong (Crop) area 37x37 di sekeliling titik pusat
            patch_numpy = img_full[titik_y-18 : titik_y+19, titik_x-18 : titik_x+19]
            # Konversi array Numpy dari OpenCV ke List standar Python
            patch_sungguhan_list = patch_numpy.tolist()
        else:
            print("[PERINGATAN] Titik terlalu dekat dengan pinggir gambar!")
            exit()
    else:
        # JIKA GAMBAR TIDAK ADA: Otomatis bikin gambar buatan (Dummy)
        print(f"\n[INFO] File '{nama_file}' tidak ditemukan.")
        print("[INFO] Beralih menggunakan gambar Dummy buatan otomatis...")
        titik_x, titik_y = 0, 0 # Anggap titik pusatnya lokal
        patch_sungguhan_list = [[(x + y) * 2 for x in range(PATCH_SIZE)] for y in range(PATCH_SIZE)]

    # 2. PERSIAPAN DATA rBRIEF & ORIENTASI
    # Muat 256 pasang alamat koordinat
    orb_dataset = load_orb_pattern_dataset()
    print(f"[INFO] Berhasil memuat {len(orb_dataset)} pasang titik koordinat (Pattern).")

    # Simulasi mendapat informasi sudut 30 derajat dari Modul Orientasi
    sudut_derajat = 30.0
    theta_radian = math.radians(sudut_derajat)
    print(f"[INFO] Kemiringan (Theta) titik yang diproses: {sudut_derajat} derajat.")

    # 3. PROSES INTI (Modul rBRIEF Beraksi)
    print(f"\nMengekstrak deskriptor 256-bit pada koordinat ({titik_x}, {titik_y})...")
    final_descriptor = compute_rbrief(patch_sungguhan_list, orb_dataset, theta_radian)

    # 4. TAMPILKAN HASIL
    print("\n--- HASIL DESKRIPTOR rBRIEF (32 Byte / 256-bit) ---")
    
    # Format list angka desimal menjadi string Heksadesimal 2 digit berderet (Contoh: '1A 5F C3...')
    hex_result = " ".join([f"{byte:02X}" for byte in final_descriptor])
    
    # Cetak hasil per 8 byte (biar terbentuk kotak 4x8 yang rapi dibaca)
    hex_list = hex_result.split(" ")
    for i in range(0, 32, 8):
        print(" ".join(hex_list[i:i+8]))
        
    print("---------------------------------------------------")
