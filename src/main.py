import cv2
import numpy as np

# Import modul subsistem 
from fast import FastBriefSubsystem
from gaussian import GaussianOrientationSubsystem

def run_hardware_simulation(image_path):
    # 1. BACA GAMBAR INPUT (Simulasi AXI-Stream)
    img_8bit = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img_8bit is None:
        raise ValueError(f"Gambar tidak ditemukan di path: {image_path}")
        
    print(f"Gambar dimuat: {img_8bit.shape[1]}x{img_8bit.shape[0]} piksel.")
    
    # 2. KUANTISASI HARDWARE (8-bit ke 6-bit)
    img_6bit = img_8bit >> 2 
    
    # Inisialisasi Subsistem
    fast_module = FastBriefSubsystem()
    gauss_module = GaussianOrientationSubsystem()

    # Ambil sampel uji di tengah gambar
    x_sample, y_sample = int(img_6bit.shape[1]/2), int(img_6bit.shape[0]/2)

    # 3. EKSEKUSI SUBSISTEM 1: Gaussian & Orientasi
    sector_id = gauss_module.hw_gaussian_to_orientation(img_6bit, x_sample, y_sample)
    
    # 4. EKSEKUSI SUBSISTEM 2: FAST & BRIEF
    brief_descriptor = fast_module.hw_fast_to_brief(x_sample, y_sample)
    
    # 5. PACKING KE AXI DMA (HEX OUTPUT 320-BIT)
    hex_x       = f"{x_sample:04X}"       # 16-bit
    hex_y       = f"{y_sample:04X}"       # 16-bit
    hex_scale   = f"{0:02X}"              # 8-bit
    hex_sector  = f"{sector_id:02X}"      # 8-bit (valid 6-bit)
    hex_pad     = f"{0:02X}"              # 8-bit
    
    # Konversi array byte BRIEF (32 byte) ke string HEX
    hex_brief = "".join([f"{b:02X}" for b in brief_descriptor]) # 256-bit
    
    packet_320bit_hex = hex_x + hex_y + hex_scale + hex_sector + hex_pad + hex_brief
    
    print("\n=== OUTPUT AXI DMA ===")
    print(f"Koordinat      : ({x_sample}, {y_sample})")
    print(f"Sector Sudut   : {sector_id} (dari 64 sektor)")
    print(f"Data 320-bit   : 0x{packet_320bit_hex}")

if __name__ == "__main__":
    # Buat gambar dummy untuk pengujian tanpa sampel foto
    dummy_img = np.random.randint(0, 255, (200, 200), dtype=np.uint8)
    cv2.imwrite('dummy_test.jpg', dummy_img)
    
    # penyesuaian nama file simulasi
    run_hardware_simulation('picture_test.jpg')