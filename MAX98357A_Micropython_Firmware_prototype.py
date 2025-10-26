from machine import I2S, SPI
from machine import Pin
from hardware.sdcard import SDCard
import uos


spi = SPI(
    0,
    baudrate=400000,  # increase SPI baudrate for better SD card performance
    polarity=0,
    phase=0,
    bits=8,
    firstbit=machine.SPI.MSB,
    sck=Pin(2),
    mosi=Pin(3),
    miso=Pin(4),
)
cs = Pin(5, machine.Pin.OUT)
cs.value(1)
sd = SDCard(spi, cs)

uos.mount(sd, "/sd")
print("Contents of SD card:")
print(uos.listdir("/sd"))

sd.init_spi(50000000)


SCK_PIN = 16 #BCLK
WS_PIN = 17  #LRCK
SD_PIN = 18  #DIN
I2S_ID = 1
BUFFER_LENGTH_IN_BYTES = 10000  # Increase the I2S internal buffer size

# ======= AUDIO CONFIGURATION =======
WAV_FILE = "/sd/h01.WAV"
WAV_SAMPLE_SIZE_IN_BITS = 32
FORMAT = I2S.MONO
SAMPLE_RATE_IN_HZ = 24000
# ======= AUDIO CONFIGURATION =======

audio_out = I2S(
    I2S_ID,
    sck=Pin(SCK_PIN),
    ws=Pin(WS_PIN),
    sd=Pin(SD_PIN),
    mode=I2S.TX,
    bits=WAV_SAMPLE_SIZE_IN_BITS,
    format=FORMAT,
    rate=SAMPLE_RATE_IN_HZ,
    ibuf=BUFFER_LENGTH_IN_BYTES,
)

wav = open(WAV_FILE, "rb")
pos = wav.seek(44)  # advance to first byte of Data section in WAV file

# allocate sample array
# memoryview used to reduce heap allocation
wav_samples = bytearray(512)
wav_samples_mv = memoryview(wav_samples)

if __name__ == "__main__":
    repeat_file = True
    try:
        while repeat_file:
            num_read = wav.readinto(wav_samples_mv)
            # end of WAV file?
            if num_read == 0:
                # end-of-file, advance to first byte of Data section
                _ = wav.seek(44)
                repeat_file = False
                wav.close()
                audio_out.deinit()
            else:
                _ = audio_out.write(wav_samples_mv[:num_read])
    except KeyboardInterrupt:
        wav.close()
        audio_out.deinit()
        

