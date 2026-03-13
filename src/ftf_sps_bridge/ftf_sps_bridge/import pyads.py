import pyads

AMS = "192.168.100.1.1.1"
IP  = "192.168.100.1"
PORT = 801

plc = pyads.Connection(AMS, PORT, IP)
plc.open()
print("Verbunden — starte Bruteforce-Symbolsuche...\n")

prefixes = [
    "", 
    ".", 
    "MAIN.", 
    "GVL.", 
    "GLOBAL.", 
    "PLC.", 
    "Modul.", 
    "Instanz.", 
    "CORE.", 
    "CORE_OUT_DATA.", 
    "Mapping.", 
    "Counter_Encoder_Links.", 
    "Counter_Encoder_Rechts.",
    "MAPPING_COUNTER.",
]

names = [
    "CORE_OUT_DATA[7]",
    "CORE_OUT_DATA[8]",
    "L_CURRENT_POS1",
    "L_CURRENT_POS2",
    "L_COUNTERDIFFERENCE1",
    "L_COUNTERDIFFERENCE2",
    "CounterValue",
    "Current_Pos",
    "Ist_Velocity",
    "Position",
]

for prefix in prefixes:
    for name in names:
        full = prefix + name
        try:
            info = plc.get_symbol_info(full)
            print(f"FOUND: {full:40}  INDEX={info.index_group}/{info.index_offset} SIZE={info.size}")
        except:
            pass   # Ignore missing

plc.close()
print("\nScan fertig.")
