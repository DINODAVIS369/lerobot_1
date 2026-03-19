import serial
import time
import sys

def check_servos(port='/dev/ttyACM0', baud=1000000):
    print(f"--- Standalone Serial Diagnostic (Baud: {baud}) ---")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"ERROR: Could not open {port}: {e}")
        return

    # ST3215 Ping Packet: [0xFF, 0xFF, ID, 0x02, 0x01, Checksum]
    def ping(sid):
        packet = [0xFF, 0xFF, sid, 0x02, 0x01]
        checksum = ~(sum(packet[2:]) & 0xFF) & 0xFF
        packet.append(checksum)
        
        start = time.time()
        ser.write(bytearray(packet))
        response = ser.read(6) # Expect [FF, FF, ID, 02, ERROR, CHK]
        end = time.time()
        
        if response and len(response) >= 6:
            return True, (end-start)*1000
        return False, 0

    alive = []
    for i in range(1, 7):
        ok, dt = ping(i)
        if ok:
            print(f"Servo {i}: [ONLINE]  Latency: {dt:.2f}ms")
            alive.append(i)
        else:
            print(f"Servo {i}: [OFFLINE]")

    ser.close()
    print("-" * 40)
    if len(alive) == 6:
        print("RESULT: All 6 servos are communicating perfectly.")
    elif len(alive) > 0:
        print(f"RESULT: Only {len(alive)}/6 servos are responding. Check power/cables.")
    else:
        print("RESULT: NO SERVOS RESPONDING. Check serial port connection and 12V power.")

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000
    check_servos(port, baud)
