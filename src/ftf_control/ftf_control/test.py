import pyads

AMS_ID = "192.168.100.1.1.1"
PLC_IP = "192.168.100.1"
PLC_PORT = 801

plc = pyads.Connection(AMS_ID, PLC_PORT, PLC_IP)
plc.open()

print("AIC_OUT_DATA[19] =", plc.read_by_name(".AIC_OUT_DATA[19]", pyads.PLCTYPE_WORD))
print("AIC_OUT_DATA[13] =", plc.read_by_name(".AIC_OUT_DATA[13]", pyads.PLCTYPE_WORD))

try:
    print("Hub L =", plc.read_by_name("HUBTEST_2_MOTOREN.A_ISTPOSITION_L", pyads.PLCTYPE_REAL))
    print("Hub R =", plc.read_by_name("HUBTEST_2_MOTOREN.A_ISTPOSITION_R", pyads.PLCTYPE_REAL))
    print("Sync  =", plc.read_by_name("HUBTEST_2_MOTOREN.A_SERVOS_SYNCHRON", pyads.PLCTYPE_BOOL))
except Exception as e:
    print("Hubtest read error:", e)

plc.close()