import socket, wave
import numpy as np

ESP32_IP = '192.168.4.51'
ESP32_PORT = 8888

def read_line(sock):
    buf = bytearray()
    while True:
        ch = sock.recv(1)
        if not ch:
            raise ConnectionError("Socket closed")
        if ch == b'\n':
            return buf.decode('utf-8', errors='ignore').rstrip('\r')
        buf += ch

def read_n_bytes(sock, n):
    data = bytearray()
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError("Socket closed")
        data += chunk
    return bytes(data)

def convert_adc12lj_to_pcm16le(payload_bytes: bytes) -> bytes:
    """
    ESP32 I2S+ADC gives 12-bit unsigned samples left-justified in 16-bit words.
    Convert to signed PCM16LE suitable for WAV.
    """
    # Ensure even length (pairs of bytes)
    if len(payload_bytes) & 1:
        payload_bytes = payload_bytes[:-1]

    u16 = np.frombuffer(payload_bytes, dtype='<u2')      # 16-bit little-endian unsigned
    u12 = (u16 >> 4).astype(np.int32)                    # 0..4095
    s12 = u12 - 2048                                     # center to [-2048..+2047]
    s16 = np.clip(s12 * 16, -32768, 32767).astype('<i2') # scale to full int16 range
    return s16.tobytes()

def receive_stream():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print(f"Connecting to {ESP32_IP}:{ESP32_PORT}...")
        s.connect((ESP32_IP, ESP32_PORT))
        print("Connected.")

        # Expect begin + metadata
        line = read_line(s)
        if line != "MIC_STREAM_BEGIN":
            raise RuntimeError(f"Unexpected first line: {line}")

        sr = 16000
        fmt = "PCM16LE"

        # read sr and fmt lines (order not guaranteed)
        for _ in range(2):
            line = read_line(s)
            if line.startswith("sr="):
                sr = int(line.split("=",1)[1])
            elif line.startswith("fmt="):
                fmt = line.split("=",1)[1]

        print(f"Streaming: {sr} Hz, fmt={fmt}")

        if fmt not in ("PCM16LE", "ADC12LJ"):
            raise RuntimeError(f"Unsupported format: {fmt}")

        with wave.open("capture.wav", "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # 16-bit
            wf.setframerate(sr)

            while True:
                header = read_line(s)  # CHUNK N or MIC_STREAM_END
                if header == "MIC_STREAM_END":
                    print("Stream ended.")
                    break
                if not header.startswith("CHUNK "):
                    raise RuntimeError(f"Unexpected header: {header}")

                nbytes = int(header.split(" ", 1)[1])
                payload = read_n_bytes(s, nbytes)

                if fmt == "PCM16LE":
                    wf.writeframes(payload)
                else:  # ADC12LJ
                    pcm = convert_adc12lj_to_pcm16le(payload)
                    wf.writeframes(pcm)

        print("WAV saved: capture.wav")

if __name__ == "__main__":
    receive_stream()
