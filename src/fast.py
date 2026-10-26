# fast_subsystem.py
import random

class FastBriefSubsystem:
    def __init__(self):
        # Inisialisasi parameter 
        pass

    def hw_fast_to_brief(self, x, y):
        """
        Mensimulasikan modul FAST Corner Detector dan BRIEF Generator.
        Input:
            x, y : Koordinat piksel (int)
        Output:
            brief_256bit : Array of integer (32 elemen x 8-bit) merepresentasikan 256-bit
        """
        # Pada hardware asli, koordinat ini divalidasi oleh subsistem FAST-9 dan NMS
        # Setelah dinyatakan sebagai 'corner', generator BRIEF membandingkan 256 pasang piksel
        
        # Simulasi Menghasilkan 256-bit (32 byte) nilai acak
        brief_256bit = [random.randint(0, 255) for _ in range(32)]
        
        return brief_256bit