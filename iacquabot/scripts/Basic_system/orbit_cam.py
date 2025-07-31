import os
import time
import math
import subprocess

R = 8        # Radio de la órbita
Z = 3        # Altura
steps = 720  # Pasos para una órbita completa
delay = 0.03 # Pausa entre pasos (s)

for i in range(steps):
    angle = (2 * math.pi * i) / steps  # Ángulo de 0 a 2π
    x = R * math.cos(angle)
    y = R * math.sin(angle)
    
    # Mueve la cámara
    cmd = f'gz camera --camera_name user_camera --pose "{x} {y} {Z} 0 0 {angle}"'
    subprocess.run(cmd, shell=True)
    
    # Captura frame opcional
    # frame_name = f"frame_{i:03d}.png"
    # subprocess.run(f'gz camera --camera_name user_camera --save_frame {frame_name}', shell=True)
    
    time.sleep(delay)
