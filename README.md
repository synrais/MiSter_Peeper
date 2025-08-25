# MiSTer_Peeper

# mister_peeper

A lightweight C utility for **MiSTer FPGA** that reads scaler registers and reports live video output status, FPS, static detection, and approximate dominant color of the video stream.  
It uses `/dev/mem` to directly map the scaler buffer and sample pixels with very low CPU overhead (<1%).  

---

## ✨ Features
- Reports current scaler resolution and output resolution.  
- Displays current FPS using the scaler’s frame counter.  
- Detects if the image is static (no pixel changes over time).  
- Reads the current game/core name from `/tmp/SAM_Game.txt`, `/tmp/ROM`, or `/tmp/NAME`.  
- Samples pixels to estimate the **dominant RGB value** and maps it to a human-readable color.  
- Efficient design: uses coarse sampling and only prints updates once per second.  

---

## 📦 Building

On your Linux host:  

```bash
gcc -O2 mister_peeper.c -o mister_peeper
```

Then copy it to your MiSTer:  

```bash
scp mister_peeper root@192.168.1.xxx:/media/fat/
```

---

## 🚀 Usage

Run it directly from the MiSTer console (SSH or UART):  

```bash
./mister_peeper
```

It will output a single continuously refreshing line like:

```
Output=1 | StaticTime=0 | RGB=#FF0000 -> Red | FPS= 60.00 | Resolution= 320x240 -> 1920x1080 | Game=Jaguar Test Suite
```

### Fields explained:
- **Output=1** → Scaler is active  
- **StaticTime** → Counter of consecutive frames with no pixel change  
- **RGB** → Dominant color sampled from video stream (`hex → nearest named color`)  
- **FPS** → Measured frames per second  
- **Resolution** → Input framebuffer size → output resolution  
- **Game** → Core/game name (from MiSTer temp files)  

---

## ⚠️ Notes

- Channel ordering (RGB vs BGR) can differ per core. The current version is tuned to match 240p Test Suite and Jaguar, but other cores may need adjustment.  
- Static detection is a heuristic (sampling every 16th pixel) and may not catch very small changes.  
- Requires root (uses `/dev/mem`).  

---

## 📜 License
MIT License — do whatever you like, but no warranty.  
