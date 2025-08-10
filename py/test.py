import socket, wave
import numpy as np

ESP32_IP = '192.168.4.51'
ESP32_PORT = 8888

# ----------- Filters / dynamics -----------

class OnePoleHPF:
    # y[n] = a * (y[n-1] + x[n] - x[n-1])
    def __init__(self, alpha=0.997):   # a bit higher than before to push cutoff lower
        self.a = float(alpha)
        self.px = 0.0
        self.py = 0.0
    def process(self, x_i16: bytes) -> bytes:
        if not x_i16: return x_i16
        x = np.frombuffer(x_i16, dtype='<i2').astype(np.float32)
        y = np.empty_like(x)
        a, px, py = self.a, self.px, self.py
        for i in range(x.size):
            xi = x[i]
            yi = a * (py + xi - px)
            y[i] = yi
            py = yi; px = xi
        self.px, self.py = float(px), float(py)
        y = np.clip(y, -32768, 32767).astype('<i2')
        return y.tobytes()

class BiquadLPF:
    # RBJ low-pass, fs in Hz, fc cutoff in Hz, Q ~ 0.707 (Butterworth)
    def __init__(self, fs: float, fc: float = 4500.0, Q: float = 0.707):
        self.fs = fs
        self.set(fc, Q)
        self.z1 = 0.0; self.z2 = 0.0
    def set(self, fc, Q):
        fs = self.fs
        w0 = 2.0 * np.pi * (fc / fs)
        cosw0 = np.cos(w0); sinw0 = np.sin(w0)
        alpha = sinw0 / (2.0 * Q)
        b0 = (1 - cosw0) / 2.0
        b1 = 1 - cosw0
        b2 = (1 - cosw0) / 2.0
        a0 = 1 + alpha
        a1 = -2 * cosw0
        a2 = 1 - alpha
        # normalize
        self.b0 = b0 / a0; self.b1 = b1 / a0; self.b2 = b2 / a0
        self.a1 = a1 / a0; self.a2 = a2 / a0
    def process(self, x_i16: bytes) -> bytes:
        if not x_i16: return x_i16
        x = np.frombuffer(x_i16, dtype='<i2').astype(np.float32)
        y = np.empty_like(x)
        b0,b1,b2,a1,a2 = self.b0,self.b1,self.b2,self.a1,self.a2
        z1,z2 = self.z1,self.z2
        for i in range(x.size):
            # Direct Form I (transposed would be fine too)
            w = x[i] - a1*z1 - a2*z2
            yi = b0*w + b1*z1 + b2*z2
            y[i] = yi
            z2 = z1; z1 = w
        self.z1,self.z2 = z1,z2
        y = np.clip(y, -32768, 32767).astype('<i2')
        return y.tobytes()

class DownwardExpander:
    """
    Simple noise gate / expander.
    Tracks a running RMS; if the short-term RMS < threshold, attenuate by ratio.
    """
    def __init__(self, fs, frame_ms=10, attack_ms=15, release_ms=120,
                 noise_hang_ms=300, threshold_mul=1.6, ratio=2.0, floor_db=-25.0):
        self.fs = fs
        self.frame = max(1, int(fs * frame_ms / 1000))
        self.attack = np.exp(-1.0 / max(1, fs * attack_ms / 1000))
        self.release = np.exp(-1.0 / max(1, fs * release_ms / 1000))
        self.hang_frames = int(noise_hang_ms / frame_ms)
        self.threshold_mul = threshold_mul
        self.ratio = ratio
        self.floor = 10 ** (floor_db / 20.0)  # min gain
        self.noise_rms = 500.0   # start reasonably high; will adapt down
        self.env = 0.0
        self.hang = 0
        self.rem = np.array([], dtype=np.int16)  # carry-over for framing

    def process(self, x_i16: bytes) -> bytes:
        if not x_i16: return x_i16
        x = np.frombuffer(x_i16, dtype='<i2').astype(np.float32)

        if self.rem.size:
            x = np.concatenate([self.rem.astype(np.float32), x])

        out = np.empty_like(x)
        N = x.size
        hop = self.frame
        idx = 0
        while idx < N:
            seg = x[idx: idx + hop]
            if seg.size < hop:
                break
            # RMS
            rms = np.sqrt(np.mean(seg * seg) + 1e-9)
            # track noise floor when envelope is very low
            self.noise_rms = 0.995 * self.noise_rms + 0.005 * min(self.noise_rms, rms)

            thresh = self.threshold_mul * self.noise_rms
            # envelope follower (peak-ish)
            peak = np.max(np.abs(seg))
            if peak > self.env:
                self.env = self.attack * self.env + (1 - self.attack) * peak
            else:
                self.env = self.release * self.env + (1 - self.release) * peak

            # compute gain
            if rms < thresh:
                # below threshold: apply expansion toward floor
                # gain_db reduces by (ratio-1)x below threshold
                # map rms/thresh to a soft knee-ish curve
                t = max(1e-6, rms / thresh)
                gain = t ** (self.ratio - 1.0)
                gain = max(self.floor, gain)
                self.hang += 1 if self.env < thresh else 0
            else:
                gain = 1.0
                self.hang = 0

            out[idx: idx + hop] = seg * gain
            idx += hop

        # keep remainder
        self.rem = x[idx:].astype(np.int16)
        out = np.clip(out[:idx], -32768, 32767).astype('<i2')
        return out.tobytes()

# ----------- TCP helpers -----------

def apply_gain_i16(pcm_bytes: bytes, gain_db: float) -> bytes:
    if not pcm_bytes:
        return pcm_bytes
    g = 10.0 ** (gain_db / 20.0)
    x = np.frombuffer(pcm_bytes, dtype='<i2').astype(np.int32)
    y = np.clip(x * g, -32768, 32767).astype('<i2')
    return y.tobytes()

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

# ----------- ADC12LJ -> PCM16 -----------

def adc12lj_to_pcm16(payload_bytes: bytes) -> bytes:
    if len(payload_bytes) & 1:
        payload_bytes = payload_bytes[:-1]
    u16 = np.frombuffer(payload_bytes, dtype='<u2')      # LE unsigned
    u12 = (u16 >> 4).astype(np.int32)                    # 0..4095
    s12 = u12 - 2048                                     # center
    s16 = np.clip(s12 * 16, -32768, 32767).astype('<i2')
    return s16.tobytes()

# ----------- Receiver -----------

def rms_dbfs_i16(pcm_bytes: bytes) -> float:
    if not pcm_bytes:
        return -120.0
    x = np.frombuffer(pcm_bytes, dtype='<i2').astype(np.float32)
    rms = np.sqrt(np.mean(x * x) + 1e-12)
    return 20.0 * np.log10(rms / 32767.0 + 1e-12)

def receive_stream():
    chunk_idx = 0;
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print(f"Connecting to {ESP32_IP}:{ESP32_PORT}...")
        s.connect((ESP32_IP, ESP32_PORT))
        print("Connected.")

        if read_line(s) != "MIC_STREAM_BEGIN":
            raise RuntimeError("Unexpected start")

        sr = 16000
        fmt = "PCM16LE"
        for _ in range(2):
            line = read_line(s)
            if line.startswith("sr="):
                sr = int(line.split("=",1)[1])
            elif line.startswith("fmt="):
                fmt = line.split("=",1)[1]
        print(f"Streaming: {sr} Hz, fmt={fmt}")

        hpf = OnePoleHPF(alpha=0.997)
        lpf = BiquadLPF(fs=sr, fc=4500.0, Q=0.707)   # try 4000–5000 Hz
        gate = DownwardExpander(fs=sr, frame_ms=10, threshold_mul=1.6, ratio=2.0, floor_db=-30)

        with wave.open("capture.wav", "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sr)

            while True:
                header = read_line(s)
                if header == "MIC_STREAM_END":
                    print("Stream ended.")
                    break
                if not header.startswith("CHUNK "):
                    raise RuntimeError(f"Unexpected header: {header}")

                nbytes = int(header.split(" ", 1)[1])
                payload = read_n_bytes(s, nbytes)

                if fmt == "ADC12LJ":
                    pcm = adc12lj_to_pcm16(payload)
                else:
                    pcm = payload

                pre = rms_dbfs_i16(pcm)

                # Process chain: HPF -> LPF -> expander
                pcm = hpf.process(pcm)
                pcm = lpf.process(pcm)
                #pcm = gate.process(pcm)
                pcm = apply_gain_i16(pcm, gain_db=18)  # try 6, 9, or 12 dB
                post = rms_dbfs_i16(pcm)

                chunk_idx += 1

                if (chunk_idx % 15) == 0:
                    print(f"[chunk {chunk_idx}] decode {pre:.1f} dBFS -> post {post:.1f} dBFS")
                wf.writeframes(pcm)

        print("WAV saved: capture.wav")

if __name__ == "__main__":
    receive_stream()
