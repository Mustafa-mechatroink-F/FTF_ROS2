import time
import pyads
from ctypes import c_uint16

AMS = "192.168.100.1.1.1"
IP = "192.168.100.1"
PORT = 801

AICArrayType = c_uint16 * 31

def set_bit(word: int, bit: int, state: bool) -> int:
    if state:
        return word | (1 << bit)
    return word & ~(1 << bit)

plc = pyads.Connection(AMS, PORT, IP)
plc.open()

try:
    for i in range(20):
        aic_in = list(plc.read_by_name(".AIC_IN_DATA", AICArrayType))

        # Bit 9 = Ablauf_aktiv
        aic_in[0] = set_bit(aic_in[0], 9, True)

        # Bit 11 = CORE_FMS_AIC_OK
        aic_in[0] = set_bit(aic_in[0], 11, True)

        # Bit 0 = Heartbeat toggeln
        hb = (i % 2 == 0)
        aic_in[0] = set_bit(aic_in[0], 0, hb)

        plc.write_by_name(".AIC_IN_DATA", aic_in, AICArrayType)
        time.sleep(0.25)

    print("Heartbeat-Test gesendet")

finally:
    plc.close()