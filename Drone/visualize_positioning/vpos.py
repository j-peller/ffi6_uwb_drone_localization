import socket
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Verbindung zum C++ Server
HOST = '192.168.2.145'
PORT = 6969

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

# Anchors und Ursprung bestimmen
anchors_raw = [(-0.25, 0.25, 0), (0.25, 0.25, 0), (0.25, -0.25, 0), (-0.25, -0.25, 0)]
anchor_min_x = min(a[0] for a in anchors_raw)
anchor_min_y = min(a[1] for a in anchors_raw)
anchor_min_z = min(a[2] for a in anchors_raw)

# Anchors auf Ursprung umrechnen
anchors = [(
    a[0] - anchor_min_x,
    a[1] - anchor_min_y,
    a[2] - anchor_min_z
) for a in anchors_raw]

# Plot Setup
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_zlim(0, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Live Drone Position (fixed scale)")

# Anchors anzeigen
ax.scatter(*zip(*anchors), c='red', label='Anchors')
drone, = ax.plot([0], [0], [0], 'bo', label='Drone')
label = ax.text(0, 0, 0.05, "Drone")
text_display = plt.figtext(0.15, 0.02, "Position: (x=..., y=..., z=...)", fontsize=10, ha='left')

plt.ion()
plt.legend()
plt.show()

while True:
    data = s.recv(1024)
    if not data:
        continue
    try:
        x, y, z = map(float, data.decode().strip().split(','))
        if not all(map(math.isfinite, (x, y, z))):
            print(f"⚠️ Ungültige Daten: {x}, {y}, {z}")
            continue

        # Ursprung korrigieren
        x -= anchor_min_x
        y -= anchor_min_y
        z -= anchor_min_z

        # Drohne aktualisieren
        drone.set_data([x], [y])
        drone.set_3d_properties([z])
        label.set_position((x, y))
        label.set_3d_properties(z + 0.05, zdir='z')
        text_display.set_text(f"Position: x={x:.2f}, y={y:.2f}, z={z:.2f}")

        plt.pause(0.01)
    except Exception as e:
        print("❌ Parse-Fehler:", e)