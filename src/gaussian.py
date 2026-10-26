# gaussian_subsystem.py
import numpy as np
import math

class GaussianOrientationSubsystem:
    def __init__(self):
        # Inisialisasi Kernel Binomial Gaussian 7x7
        self.gauss_kernel = np.array([
            [ 1,  6, 15, 20, 15,  6,  1],
            [ 6, 36, 90,120, 90, 36,  6],
            [15, 90,225,300,225, 90, 15],
            [20,120,300,400,300,120, 20],
            [15, 90,225,300,225, 90, 15],
            [ 6, 36, 90,120, 90, 36,  6],
            [ 1,  6, 15, 20, 15,  6,  1]
        ], dtype=np.int32)

    def hw_gaussian_to_orientation(self, img_6bit, x, y):
        """
        Mensimulasikan Filter Gaussian 7x7 (DSP-Free) dan Orientation (64-Sektor).
        Input:
            img_6bit : Matriks citra 6-bit (numpy array)
            x, y     : Koordinat titik sudut (int)
        Output:
            sector_6bit : ID Sektor sudut (0-63)
        """
        # Ekstrak patch 7x7 dari gambar 6-bit dan ubah ke tipe data signed (int32)
        # Casting ke int32 ini mencegah OverflowError saat dikali dengan koordinat negatif
        patch = img_6bit[y-3 : y+4, x-3 : x+4].astype(np.int32)
        
        if patch.shape != (7, 7):
            return 0 # Abaikan jika posisi terlalu di tepi gambar
            
        # DSP-Free Gaussian Smoothing (Shift >> 12 menggantikan bagi 4096)
        sum_val = np.sum(patch * self.gauss_kernel)
        smoothed_pixel = sum_val >> 12 # Disimpan di pipeline hardware
        
        # Simulasi Perhitungan Orientasi (Centroid Moment m10 dan m01)
        m10, m01 = 0, 0
        for dy in range(-3, 4):
            for dx in range(-3, 4):
                val = patch[dy+3, dx+3]
                m10 += dx * val
                m01 += dy * val
                
        # Konversi ke 64 Sektor (Hardware asli menggunakan Constant-Tan dan Comparator)
        angle_rad = math.atan2(m01, m10)
        if angle_rad < 0:
            angle_rad += 2 * math.pi
            
        # Membagi 360 derajat (2*PI) menjadi 64 Sektor terkuantisasi (0-63)
        sector_6bit = int((angle_rad / (2 * math.pi)) * 64) % 64
        
        return sector_6bit
