# Python Golden Model (`orb_pyramid_golden.py`) - Detailed Breakdown

While the exact lines of your Python script may vary slightly, the structure of the golden model follows a strict sequence to generate the hardware test data. Here is the block-by-block breakdown of what the code does.

## 1. Setup and Argument Parsing
```python
import argparse
import numpy as np
from PIL import Image
import json
import os
```
* **Explanation:** Imports necessary libraries. `numpy` is for math arrays, `PIL` handles image reading/writing, `json` is for the manifest, and `os` handles directories.
* The script then uses `argparse` to read your command-line inputs like `--input test.png`, `--outdir imageTest`, and `--rounding round`.

## 2. Image Loading and Preprocessing
```python
img = Image.open(args.input).convert('L')
img_array = np.array(img)
```
* **Explanation:** Opens the image you passed in and immediately converts it to `L` mode (8-bit grayscale). It then transforms this image into a 2D mathematical matrix (`img_array`) where every pixel is a number between 0 and 255.

## 3. The Core Scaling Function (The Modulo-6 Math)
The script loops over the image to create the scaled levels. For each pixel in the new level, it maps back to the original image.
```python
# Pseudo-code representation of the core loop
for y in range(height):
    for x in range(width):
        x6 = x % 6
        y6 = y % 6
        
        # Skip condition (Downscaling by 5/6)
        if x6 == 5 or y6 == 5:
            continue 
```
* **Explanation:** The hardware uses streaming counters, so the Python code mimics this by looking at the modulo-6 (`% 6`) of the coordinates. If a coordinate's modulo is `5`, that pixel is skipped. This is how 6 pixels become 5, achieving the 1.2x downscale.

## 4. Bilinear Weight Calculation and Accumulation
```python
w00 = (5 - x6) * (5 - y6)
w01 = x6 * (5 - y6)
w10 = (5 - x6) * y6
w11 = x6 * y6

acc = w00*I[y, x] + w01*I[y, x+1] + w10*I[y+1, x] + w11*I[y+1, x+1]
```
* **Explanation:** Calculates the 4 weights based purely on the `x6` and `y6` counters. `w00` is the top-left pixel, `w11` is the bottom-right. It multiplies the 4 neighboring pixels by their weights and adds them together into the raw `acc` (accumulator) value.

## 5. Normalization
```python
if rounding == 'round':
    pixel_out = (acc + 12) // 25
else:
    pixel_out = acc // 25
```
* **Explanation:** Since the maximum sum of the weights is 25, we must divide by 25. Adding 12 before dividing gives us "round half up" behavior, exactly matching the hardware.

## 6. File Export (Hex Generation)
```python
with open("level1_pixels.hex", "w") as f:
    for p in pixels:
        f.write(f"{p:02x}\n")
```
* **Explanation:** Verilog's `$readmemh` expects text files where each line is a hexadecimal number. This block takes our resulting pixel array and formats each number into a 2-digit hex string (e.g., `255` becomes `ff`). It does the same for the 13-bit accumulator values, formatting them as 4-digit hex strings (e.g., `04a3`).
