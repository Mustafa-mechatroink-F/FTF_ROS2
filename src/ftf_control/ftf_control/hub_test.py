#!/usr/bin/env python3
import pyads
import time
import struct

# Verbindungsparameter
PLC_AMS_ID = "192.168.100.1.1.1"
PLC_IP = "192.168.100.1"

def run_hub_test():
    # 1. Verbindung aufbauen
    plc = pyads.Connection(PLC_AMS_ID, 801, PLC_IP)
    try:
        plc.open()
        print("✅ ADS Verbindung offen.")

        # 2. Den "Sperrmodus" vom Drive-Node explizit ausschalten
        print("🔓 Deaktiviere .TEST_OHNE_HUB...")
        plc.write_by_name(".TEST_OHNE_HUB", False, pyads.PLCTYPE_BOOL)

        # 3. Heartbeat & Predominance Loop (für 5 Sekunden)
        # Wir müssen der SPS zeigen, dass wir "leben"
        target_height = 600 # Ziel in mm
        print(f"🚀 Starte Test-Zyklus: Ziel {target_height}mm")

        for i in range(50): # 5 Sekunden lang (100ms Takt)
            # --- WORD 0 (Offset 300) schreiben ---
            # Bit 0: Heartbeat (toggelt), Bit 1: AIC_Predominance (AN)
            hb_bit = i % 2
            control_word = hb_bit | (1 << 1)
            plc.write(0xF020, 300, control_word, pyads.PLCTYPE_WORD)

            # --- ZIEL & KOMMANDO schreiben ---
            if i == 5: # Nach 0.5 Sekunde den Fahrbefehl schicken
                print(f"  -> Sende Ziel {target_height} an Offset 326")
                plc.write(0xF020, 326, target_height, pyads.PLCTYPE_INT)
                print("  -> Sende Start-Kommando (3) an Offset 328")
                plc.write(0xF020, 328, 3, pyads.PLCTYPE_WORD)

            # --- STATUS LESEN ---
            try:
                ist_pos = plc.read(0xF030, 326, pyads.PLCTYPE_INT)
                status_word = plc.read(0xF030, 328, pyads.PLCTYPE_WORD)
                ready = bool(status_word & (1 << 0))
                moving = bool(status_word & (1 << 1))
                print(f"  [{i}] Ist: {ist_pos}mm | Ready: {ready} | Moving: {moving}", end="\r")
            except:
                pass

            time.sleep(0.1)

        # 4. Stop & Abschluss
        print("\n🛑 Test beendet. Sende Stop.")
        plc.write(0xF020, 328, 0, pyads.PLCTYPE_WORD)
        plc.close()

    except Exception as e:
        print(f"❌ Fehler: {e}")

if __name__ == "__main__":
    run_hub_test()