import re
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# --- KONFIGURACJA PORTU ---
SERIAL_PORT = 'COM3'  # Taki sam jak w logach PlatformIO
BAUD_RATE = 115200

# Nawiazanie polaczenia z ESP32
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)

# Wyrazenie regularne wyciagajace kwaternion Q:[w, x, y, z] oraz AccNorm z Twojego printf
pattern = re.compile(
    r"Q:\[\s*([-+]?\d*\.\d+|\d+),\s*([-+]?\d*\.\d+|\d+),\s*([-+]?\d*\.\d+|\d+),\s*([-+]?\d*\.\d+|\d+)\]\s*\|\s*AccNorm:([-+]?\d*\.\d+|\d+)g"
)

# Aktualny stan kwaternionu [w, x, y, z]
current_q = np.array([1.0, 0.0, 0.0, 0.0])
current_acc = 1.0

def quat_to_rot_matrix(q):
    """Konwertuje kwaternion [w, x, y, z] na macierz obrotu 3x3."""
    w, x, y, z = q
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm > 0:
        w, x, y, z = w/norm, x/norm, y/norm, z/norm

    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])

# Wymiary prostopadloscianu (przypominajacego plytke MPU6050)
dx, dy, dz = 1.2, 0.8, 0.25
base_vertices = np.array([
    [-dx, -dy, -dz], [ dx, -dy, -dz], [ dx,  dy, -dz], [-dx,  dy, -dz],
    [-dx, -dy,  dz], [ dx, -dy,  dz], [ dx,  dy,  dz], [-dx,  dy,  dz]
])

# Definicja 6 scian prostopadloscianu (indeksy wierzcholkow)
faces_idx = [
    [0, 1, 2, 3], # Dol
    [4, 5, 6, 7], # Gora (wyróżnimy ją kolorem, żeby widzieć góra/dół)
    [0, 1, 5, 4], # Przod
    [2, 3, 7, 6], # Tyl
    [1, 2, 6, 5], # Prawa
    [4, 7, 3, 0]  # Lewa
]
face_colors = ['#4a5568', '#3182ce', '#e53e3e', '#718096', '#a0aec0', '#a0aec0']

# --- KONFIGURACJA WYKRESU 3D ---
fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection='3d')

def update(frame):
    global current_q, current_acc

    # Odczytujemy wszystkie zalegle linie z bufora, zeby nie bylo opoznien (lagow)
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            match = pattern.search(line)
            if match:
                w, x, y, z, acc = map(float, match.groups())
                current_q = np.array([w, x, y, z])
                current_acc = acc
        except Exception:
            pass

    R = quat_to_rot_matrix(current_q)
    # Obrot wierzcholkow prostopadloscianu
    rotated_verts = base_vertices @ R.T

    ax.cla()
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    ax.set_xlabel('X (Swiat)')
    ax.set_ylabel('Y (Swiat)')
    ax.set_zlabel('Z (Swiat)')
    ax.set_title(f"Orientacja MPU6050 | AccNorm: {current_acc:.2f}g\n"
                 f"Niebieska gora | Czerwony przod (+X)")

    # Rysowanie scian prostopadloscianu
    poly3d = [[rotated_verts[idx] for idx in face] for face in faces_idx]
    collection = Poly3DCollection(poly3d, facecolors=face_colors, linewidths=1, edgecolors='black', alpha=0.85)
    ax.add_collection3d(collection)

    # Rysowanie lokalnego ukladu wspolrzednych czujnika (X=Czerwony, Y=Zielony, Z=Niebieski)
    axis_len = 1.6
    origin = np.array([0, 0, 0])
    x_axis = R @ np.array([axis_len, 0, 0])
    y_axis = R @ np.array([0, axis_len, 0])
    z_axis = R @ np.array([0, 0, axis_len])

    ax.quiver(*origin, *x_axis, color='red',   linewidth=2.5, label='Oś X')
    ax.quiver(*origin, *y_axis, color='green', linewidth=2.5, label='Oś Y')
    ax.quiver(*origin, *z_axis, color='blue',  linewidth=2.5, label='Oś Z')
    ax.legend(loc='upper left')

ani = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()

ser.close()