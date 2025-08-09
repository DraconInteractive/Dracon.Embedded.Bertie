import socket

ESP32_IP = '192.168.4.51'  # Replace with your ESP32's IP if different
ESP32_PORT = 8888

def receive_mic_data():
    counter = 0;
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print(f"Connecting to {ESP32_IP}:{ESP32_PORT}...")
        s.connect((ESP32_IP, ESP32_PORT))
        print("Connected.")

        buffer = ""
        mic_data_started = False
        mic_data = []

        while True:
            print("loop");
            data = s.recv(1024).decode()
            if not data:
                print("No data");
                break

            print("data received");
            counter += 1;
            print(f"C: {counter} ----");
            buffer += data
            lines = buffer.splitlines()
            print(f"Received {len(lines)} lines");

            for line in lines:
                print(f"{line}, ");

            print("Processing...");
            # Keep the last (possibly incomplete) line in the buffer
            buffer = lines.pop() if not data.endswith("\n") else ""

            for line in lines:
                line = line.strip()
                if line == "MIC_BEGIN":
                    mic_data_started = True
                    mic_data.clear()
                    print("Processing data");
                elif line == "MIC_END":
                    mic_data_started = False
                    print(f"Received {len(mic_data)} samples")
                    print(mic_data[:20], "..." if len(mic_data) > 20 else "")
                elif mic_data_started:
                    try:
                        mic_data.append(int(line))
                    except ValueError:
                        pass  # Ignore non-integer lines

if __name__ == "__main__":
    receive_mic_data()
        
    # You can now process `mic_data` however you'd like
