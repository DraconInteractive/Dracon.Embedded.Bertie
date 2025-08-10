import socket, wave
import numpy as np

ESP32_IP = '192.168.4.51'
ESP32_PORT = 8888

class SpectralGate:
    """
    Streaming spectral noise suppressor (overlap-add).
    - Learns a noise profile from the first ~learn_seconds
    - 50% overlap, Hann window
    """
    def __init__(self, fs=16000, frame_len=512, overlap=0.5,
                 learn_seconds=0.25, noise_update_alpha=0.95,
                 suppression_strength=1.5, spectral_floor_db=-12.0):
        self.fs = fs
        self.N = int(frame_len)
        self.H = int(self.N * (1.0 - overlap))  # hop size
        self.M = self.N - self.H                     # overlap length
        self.win = np.hanning(self.N).astype(np.float32)
        self.buf = np.zeros(0, dtype=np.float32)   # input sample buffer
        self.tail = np.zeros(self.M, dtype=np.float32)
        self.noise_mag = None
        self.learn_frames = max(1, int(learn_seconds * fs / self.H))
        self.frames_seen = 0
        self.noise_update_alpha = float(noise_update_alpha)
        self.k = float(suppression_strength)
        self.floor = 10.0 ** (spectral_floor_db / 20.0)

    def process(self, pcm_bytes: bytes) -> bytes:
        if not pcm_bytes:
            return b""
        x = np.frombuffer(pcm_bytes, dtype='<i2').astype(np.float32)
        self.buf = np.concatenate([self.buf, x])

        out_chunks = []
        while self.buf.size >= self.N:
            frame = self.buf[:self.N]
            # shift input by hop (leave N-H for next frame)
            self.buf = self.buf[self.H:]

            # STFT
            X = np.fft.rfft(frame * self.win, n=self.N)
            mag = np.abs(X)
            phase = np.angle(X)

            # learn/track noise magnitude
            if self.noise_mag is None:
                self.noise_mag = mag.copy()
            if self.frames_seen < self.learn_frames:
                # conservative running min-ish
                self.noise_mag = np.minimum(self.noise_mag, mag) * 0.7 + self.noise_mag * 0.3
            else:
                a = self.noise_update_alpha
                self.noise_mag = a * self.noise_mag + (1 - a) * mag
            self.frames_seen += 1

            # soft suppression
            eps = 1e-8
            ratio = self.noise_mag / np.maximum(mag, eps)
            gain = 1.0 - self.k * ratio                   # 1 - k*(noise/mag)
            gain = np.clip(gain, self.floor, 1.0)

            Y = gain * mag * np.exp(1j * phase)
            y = np.fft.irfft(Y, n=self.N).astype(np.float32)

            # proper 50% overlap-add:
            # output first H samples + previous tail; store new tail (last N-H)
            y0 = y[:self.M] + (self.tail if self.tail.size else 0.0)
            out_chunks.append(y0.copy())
            self.tail = y[self.M:].copy()

        if not out_chunks:
            return b""
        ycat = np.concatenate(out_chunks)
        return np.clip(ycat, -32768, 32767).astype('<i2').tobytes()

    def flush(self) -> bytes:
        # Process any remaining samples by zero-padding to a full frame.
        if self.buf.size == 0 and self.tail.size == 0:
            return b""
        if self.buf.size > 0:
            pad = self.N - self.buf.size
            frame = np.zeros(self.N, dtype=np.float32)
            frame[:self.buf.size] = self.buf
            self.buf = np.zeros(0, dtype=np.float32)

            X = np.fft.rfft(frame * self.win, n=self.N)
            mag = np.abs(X); phase = np.angle(X)
            if self.noise_mag is None:
                self.noise_mag = mag.copy()
            eps = 1e-8
            ratio = self.noise_mag / np.maximum(mag, eps)
            gain = np.clip(1.0 - self.k * ratio, self.floor, 1.0)
            Y = gain * mag * np.exp(1j * phase)
            y = np.fft.irfft(Y, n=self.N).astype(np.float32)

            # One last OLA: emit first H + add tail, then keep final tail
            y0 = y[:self.M] + (self.tail if self.tail.size else 0.0)
            self.tail = y[self.M:].copy()
            out = np.clip(y0, -32768, 32767).astype('<i2').tobytes()
        else:
            out = b""
        # emit remaining tail as final block
        if self.tail.size:
            tail = np.clip(self.tail, -32768, 32767).astype('<i2').tobytes()
            self.tail = np.zeros(self.tail.size, dtype=np.float32)
            out += tail
        return out

# ----------- Filters / dynamics -----------

class OnePoleHPF:
    # y[n] = a*(y[n-1] + x[n] - x[n-1])
    def __init__(self, alpha=0.997):
        self.a = float(alpha); self.px = 0.0; self.py = 0.0
    def process(self, x_i16: bytes) -> bytes:
        if not x_i16: return x_i16
        x = np.frombuffer(x_i16, dtype='<i2').astype(np.float32)
        y = np.empty_like(x)
        a, px, py = self.a, self.px, self.py
        for i, xi in enumerate(x):
            yi = a*(py + xi - px)
            y[i] = yi; py = yi; px = xi
        self.px, self.py = float(px), float(py)
        return np.clip(y, -32768, 32767).astype('<i2').tobytes()

class BiquadLPF:
    # RBJ low-pass, fs in Hz, fc cutoff in Hz, Q ~ 0.707 (Butterworth)
    def __init__(self, fs: float, fc: float = 4500.0, Q: float = 0.707):
        self.fs = fs
        self.set(fc, Q)
        self.z1 = 0.0
        self.z2 = 0.0

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
        # normalize by a0
        self.b0 = b0 / a0
        self.b1 = b1 / a0
        self.b2 = b2 / a0
        self.a1 = a1 / a0
        self.a2 = a2 / a0

    def process(self, x_i16: bytes) -> bytes:
        if not x_i16:
            return x_i16
        x = np.frombuffer(x_i16, dtype='<i2').astype(np.float32)
        y = np.empty_like(x)

        b0, b1, b2 = self.b0, self.b1, self.b2
        a1, a2     = self.a1, self.a2
        z1, z2     = self.z1, self.z2

        # DF2T:
        for i in range(x.size):
            xi = x[i]
            yi = b0*xi + z1
            z1 = b1*xi - a1*yi + z2
            z2 = b2*xi - a2*yi
            y[i] = yi

        self.z1, self.z2 = z1, z2
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
    if not pcm_bytes: return -120.0
    x = np.frombuffer(pcm_bytes, dtype='<i2').astype(np.float32)
    rms = np.sqrt(np.mean(x*x) + 1e-12)
    return 20.0 * np.log10(rms / 32767.0 + 1e-12)

def peak_dbfs_i16(pcm_bytes: bytes) -> float:
    if not pcm_bytes: return -120.0
    x = np.frombuffer(pcm_bytes, dtype='<i2').astype(np.int32)
    p = int(np.max(np.abs(x)))
    if p <= 0: return -120.0
    return 20.0 * np.log10(p / 32767.0)

def dc_and_ac_rms_dbfs(pcm_bytes: bytes, alpha=0.995):
    """Return (dc_est_dbFS, ac_rms_dbFS) using a simple leaky DC tracker."""
    if not pcm_bytes: return (-120.0, -120.0)
    x = np.frombuffer(pcm_bytes, dtype='<i2').astype(np.float32)
    dc = 0.0
    y  = np.empty_like(x)
    a  = float(alpha)
    for i, xi in enumerate(x):
        dc = a*dc + (1-a)*xi
        y[i] = xi - dc
    dc_db = 20.0*np.log10((abs(dc)+1e-6)/32767.0 + 1e-12)
    ac_rms = np.sqrt(np.mean(y*y) + 1e-12)
    ac_db = 20.0*np.log10(ac_rms/32767.0 + 1e-12)
    return (dc_db, ac_db)

def normalize_peak_i16(pcm_bytes: bytes, target_dbfs: float = -3.0, max_gain_db: float = 24.0) -> bytes:
    if not pcm_bytes: return pcm_bytes
    x = np.frombuffer(pcm_bytes, dtype='<i2').astype(np.int32)
    p = int(np.max(np.abs(x)))
    if p <= 0: return pcm_bytes
    cur_db = 20.0 * np.log10(p / 32767.0)
    need_db = np.clip(target_dbfs - cur_db, -max_gain_db, max_gain_db)
    if abs(need_db) < 0.1: return pcm_bytes
    g = 10.0 ** (need_db / 20.0)
    y = np.clip(x * g, -32768, 32767).astype('<i2')
    return y.tobytes()

def receive_stream():
    chunk_idx = 0;
    decoder = {"mode": None}   # lock alignment on first audio chunk
    
    
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
        #lpf = BiquadLPF(fs=sr, fc=4500.0, Q=0.707)   # try 4000–5000 Hz
        #gate = DownwardExpander(fs=sr, frame_ms=10, threshold_mul=1.6, ratio=2.0, floor_db=-30)
        den = SpectralGate(fs=sr, frame_len=512, overlap=0.5,
                               learn_seconds=0.25,        # ~250 ms noise learn
                               noise_update_alpha=0.98,   # slow tracking
                               suppression_strength=1.3,  # 1.2–1.6: more = stronger
                               spectral_floor_db=-10.0)   # don’t over-suppress; -8…-12 is good
        with wave.open("capture.wav", "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sr)

            while True:
                header = read_line(s)
                if header == "MIC_STREAM_END":
                    print("Stream ended.")
                    tail = den.flush()
                    if tail:
                        tail = normalize_peak_i16(tail, target_dbfs=-3.0, max_gain_db=12.0)
                        wf.writeframes(tail)
                    break
                if not header.startswith("CHUNK "):
                    raise RuntimeError(f"Unexpected header: {header}")

                nbytes = int(header.split(" ", 1)[1])
                payload = read_n_bytes(s, nbytes)

                if fmt == "ADC12LJ":
                    # lock alignment on first chunk, then keep it
                    u16 = np.frombuffer(payload, dtype='<u2')
                    if decoder["mode"] is None:
                        cand1 = (u16 >> 4) & 0x0FFF             # left-justified
                        cand2 =  u16        & 0x0FFF             # right-justified
                        cand3 = ((u16 ^ 0x8000) >> 4) & 0x0FFF   # xor+left-justified
                        vars_ = [np.var(cand1), np.var(cand2), np.var(cand3)]
                        decoder["mode"] = ["LJ","RJ","XORLJ"][int(np.argmax(vars_))]
                        # print(f"ADC12 mode locked: {decoder['mode']}")
                    mode = decoder["mode"]
                    if mode == "LJ":
                        u12 = (u16 >> 4) & 0x0FFF
                    elif mode == "RJ":
                        u12 = u16 & 0x0FFF
                    else:
                        u12 = ((u16 ^ 0x8000) >> 4) & 0x0FFF
                    s12 = u12.astype(np.int32) - 2048
                    pcm = np.clip(s12 * 16, -32768, 32767).astype('<i2').tobytes()
                else:
                    pcm = payload  # already PCM16LE

                dec_rms = rms_dbfs_i16(pcm)
                dc_db, ac_db = dc_and_ac_rms_dbfs(pcm)
                pcm = hpf.process(pcm)
                pcm = den.process(pcm)
                if (chunk_idx % 15) == 0:
                    mid_rms = rms_dbfs_i16(pcm)
                    print(f"[chunk {chunk_idx}] post-denoise RMS {mid_rms:.1f} dBFS")
                pcm = normalize_peak_i16(pcm, target_dbfs=-3.0, max_gain_db=18.0)

                chunk_idx += 1

                if (chunk_idx % 15) == 0:
                    post_rms = rms_dbfs_i16(pcm); post_peak = peak_dbfs_i16(pcm)
                    print(f"[chunk {chunk_idx}] decode RMS {dec_rms:.1f} dBFS | DC {dc_db:.1f} / AC {ac_db:.1f} dBFS -> post RMS {post_rms:.1f} / peak {post_peak:.1f} dBFS")

                wf.writeframes(pcm)

        print("WAV saved: capture.wav")

if __name__ == "__main__":
    receive_stream()
