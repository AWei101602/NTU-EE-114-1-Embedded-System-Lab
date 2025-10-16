#!/usr/bin/env python3
from bluepy.btle import Peripheral, UUID, Scanner, DefaultDelegate, BTLEException

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print("Discovered device:", dev.addr)
        elif isNewData:
            print("Received new data from:", dev.addr)

class MyDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleNotification(self, cHandle, data):
        try:
            text = data.decode(errors="ignore")
            print(f"\nNotification from handle {cHandle}: {text}")
        except:
            print(f"\nNotification from handle {cHandle}: {data}")

print("Scanning for 10 seconds ...")
scanner = Scanner().withDelegate(ScanDelegate())
devices = list(scanner.scan(10.0))

addr_list = []
print("\n=== Scan Results ===")
for i, dev in enumerate(devices):
    name = None
    for (adtype, desc, value) in dev.getScanData():
        if desc in ["Complete Local Name", "Short Local Name"]:
            name = value
            break
    if not name:
        name = "<Unknown>"
    print(f"[{i}] {name:20s}  {dev.addr}  ({dev.addrType}), RSSI={dev.rssi} dB")
    addr_list.append((dev.addr, dev.addrType, name))

num = int(input("\nEnter device number to connect: "))
addr, addrType, name = addr_list[num]
print(f"\nConnecting to {name} ({addr}), type={addrType} ...")

try:
    dev = Peripheral(addr, addrType)
    dev.setDelegate(MyDelegate())
    print("Connected.")

    print("\n--- Services ---")
    for svc in dev.services:
        print("Service:", svc.uuid)

    try:
        testService = dev.getServiceByUUID(UUID(0xfff0))
    except BTLEException:
        print("Service 0xFFF0 not found.")
        dev.disconnect()
        exit(1)

    print("\n--- Characteristics in FFF0 ---")
    for ch in testService.getCharacteristics():
        print(f"  Char {ch.uuid} [{ch.propertiesToString()}]")

    notify_char = None
    for ch in testService.getCharacteristics():
        props = ch.propertiesToString()
        if "NOTIFY" in props:
            notify_char = ch
            break

    if notify_char:
        print(f"\nTarget characteristic found: {notify_char.uuid}")
        cccd_uuid = UUID(0x2902)
        descs = notify_char.getDescriptors(forUUID=cccd_uuid)
        if len(descs) > 0:
            desc = descs[0]
            print(f"Writing CCCD {desc.handle} = 0x0002 ...")
            dev.writeCharacteristic(desc.handle, b"\x02\x00", withResponse=True)
            print("CCCD write success (Notifications enabled).")
        else:
            print("No CCCD descriptor found for this characteristic.")
    else:
        print("No characteristic with NOTIFY property found.")

    print("\nListening for notifications... (Press Ctrl+C to exit)")
    while True:
        if dev.waitForNotifications(3.0):
            continue

except KeyboardInterrupt:
    print("\nDisconnecting...")
    dev.disconnect()
except Exception as e:
    print("Connection failed:", e)
    try:
        dev.disconnect()
    except:
        pass
