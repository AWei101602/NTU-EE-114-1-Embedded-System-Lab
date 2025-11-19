from bluepy.btle import Peripheral, UUID, Descriptor
from bluepy.btle import Scanner, DefaultDelegate
from bluepy import btle
import struct
import threading

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print("Discovered device", dev.addr)
        elif isNewData:
            print("Received new data from", dev.addr)

    def handleNotification(self, chandle, data):
        """Handle BLE notification data from STM32"""
        length = len(data)
        print(f"\n🔔 Notification ({length} bytes): {data}")

        try:
            if length == 6:
                # X, Y, Z 各 16 bits
                accel_x, accel_y, accel_z = struct.unpack('<hhh', data)
                print(f"Acceleration X={accel_x}, Y={accel_y}, Z={accel_z}")

            elif length == 8:
                # 有些裝置會多傳 2 bytes 狀態或時間戳
                accel_x, accel_y, accel_z = struct.unpack('<hhh', data[:6])
                extra = struct.unpack('<H', data[6:8])[0]
                print(f"Acceleration X={accel_x}, Y={accel_y}, Z={accel_z}, Extra={extra}")

            else:
                print(f"⚠️ Unexpected accel data length ({length}): {data}")

        except struct.error as e:
            print(f"❌ struct.error while decoding ({length} bytes): {e}")

def input_thread(dev, freq_char, freq_read_char):
    """Thread function to capture user input and change the sampling frequency."""
    while True:
        try:
            in_freq = input("Enter new sampling frequency (>100ms): ")
            if in_freq.isdigit():
                new_freq = struct.pack('<H', int(in_freq) // 100)
                freq_handle = freq_char.getHandle()
                dev.writeCharacteristic(freq_handle, new_freq, withResponse=False)
                print(f"✅ Set sampling interval to {in_freq} ms.")
            else:
                print("Invalid input. Please enter a valid integer.")
        except KeyboardInterrupt:
            print("🛑 Exiting input thread.")
            break
        except Exception as e:
            print("⚠️ Error writing new frequency:", e)

# ===========================
#  BLE SCAN AND SELECTION
# ===========================
print("🔍 Scanning for 5 seconds...")
scanner = Scanner().withDelegate(ScanDelegate())
devices = scanner.scan(5.0)

if not devices:
    print("❌ No BLE devices found. Try again.")
    exit()

print("\n=== Available Devices ===")
addr = []
for i, dev in enumerate(devices):
    print(f"[{i}] {dev.addr} ({dev.addrType}), RSSI={dev.rssi} dB")
    addr.append(dev.addr)
    for (adtype, desc, value) in dev.getScanData():
        print(f"   {desc}: {value}")
print("==========================")

# 使用者選擇要連線的裝置
while True:
    try:
        num = int(input("Select device number to connect: "))
        if 0 <= num < len(addr):
            break
        else:
            print("Invalid number, please choose again.")
    except ValueError:
        print("Please enter a valid integer.")

target_addr = addr[num]
print(f"\n🔗 Connecting to {target_addr} ...")
dev = Peripheral(target_addr, 'random')
dev.setDelegate(ScanDelegate())

# ===========================
#  SERVICE & CHARACTERISTICS
# ===========================
print("\n📡 Discovering services...")
services = list(dev.services)
for i, svc in enumerate(services):
    print(f"Service [{i}] → {svc.uuid}")
    for j, ch in enumerate(svc.getCharacteristics()):
        print(f"  Char [{j}] {ch.uuid} | Props: {ch.propertiesToString()} | Handle: {ch.getHandle()}")

try:
    # 根據你的 STM32 GATT 結構調整索引
    svc_accel_index = 2
    svc_freq_index = 3
    accel_Service = dev.getServiceByUUID(services[svc_accel_index].uuid)
    freq_Service = dev.getServiceByUUID(services[svc_freq_index].uuid)

    accel_char = list(accel_Service.getCharacteristics())[1]
    freq_read_char = list(accel_Service.getCharacteristics())[0]
    freq_char = list(freq_Service.getCharacteristics())[0]

    # 啟動輸入執行緒
    input_thread_obj = threading.Thread(target=input_thread, args=(dev, freq_char, freq_read_char))
    input_thread_obj.daemon = True
    input_thread_obj.start()

    # 啟用加速度資料通知
    accel_handle = accel_char.getHandle()
    dev.writeCharacteristic(accel_handle + 1, struct.pack('<bb', 0x01, 0x00), withResponse=True)
    print("✅ Enabled notifications for acceleration data.")

    # 顯示目前取樣頻率
    freq_value_hex = freq_read_char.read()
   # print(f"📖 Raw frequency data ({len(freq_value_hex)} bytes): {freq_value_hex}")

    if len(freq_value_hex) == 2:
        print(f"Current sampling interval: {struct.unpack('<H', freq_value_hex)[0] * 100} ms")
    elif len(freq_value_hex) == 4:
        print(f"Current sampling interval: {struct.unpack('<I', freq_value_hex)[0]} ms")
    elif len(freq_value_hex) == 8:
        vals = struct.unpack('<HHHH', freq_value_hex)
        print(f"8-byte freq data unpacked (4x16bit): {vals}")
    else:
        print(f"⚠️ Unexpected data length ({len(freq_value_hex)} bytes): {freq_value_hex}")

    # 主循環等待通知
    while True:
        if dev.waitForNotifications(10.0):
            continue
        print("Waiting for notification...")

finally:
    dev.disconnect()
    print("🔌 Disconnected.")
