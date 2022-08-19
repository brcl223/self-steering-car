import serial

with serial.Serial('/dev/ttyUSB0', 9600, timeout=10) as arduino:
    while True:
        cmd = input("Type command to send to arduino...")
        print(f"Cmd: {cmd}")
        arduino.write(bytes(cmd, 'utf-8'))

        print("Waiting for arduino to respond...")
        for i in range(5):
            print(arduino.readline())
