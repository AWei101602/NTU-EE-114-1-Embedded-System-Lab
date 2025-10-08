import socket
import matplotlib.pyplot as plt
from collections import deque

# ===== TCP Server 設定 =====
HOST = '0.0.0.0'
PORT = 8002

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)
print(f"Server listening on {HOST}:{PORT}")

conn, addr = server.accept()
print(f"Connected by {addr}")

# ===== 設定即時繪圖 =====
plt.ion()
N = 100
x_vals = deque([0]*N, maxlen=N)
y_vals = deque([0]*N, maxlen=N)
z_vals = deque([0]*N, maxlen=N)
t = list(range(N))

fig, ax = plt.subplots()
line_x, = ax.plot(t, x_vals, 'r-', label='X')
line_y, = ax.plot(t, y_vals, 'g-', label='Y')
line_z, = ax.plot(t, z_vals, 'b-', label='Z')
ax.legend()
ax.set_ylim(-2000, 2000)
ax.set_xlabel("Samples")
ax.set_ylabel("Acceleration (mg)")
ax.set_title("STM32 Accelerometer Data (TCP)")

# ===== 主接收 + 繪圖迴圈 =====
while True:
    data = conn.recv(1024)
    if not data:
        print("Connection closed by STM32.")
        break

    try:
        msg = data.decode(errors='ignore').strip()
        # 偵測格式 X:xx Y:yy Z:zz
        if msg.startswith("X:"):
            parts = msg.replace("X:", "").replace("Y:", "").replace("Z:", "").split()
            if len(parts) >= 3:
                x, y, z = map(int, parts[:3])
                x_vals.append(x)
                y_vals.append(y)
                z_vals.append(z)

                line_x.set_ydata(x_vals)
                line_y.set_ydata(y_vals)
                line_z.set_ydata(z_vals)
                plt.draw()
                plt.pause(0.01)
        else:
            print("Other:", msg)

    except Exception as e:
        print("Parse error:", e, "| raw:", data)

conn.close()
server.close()
