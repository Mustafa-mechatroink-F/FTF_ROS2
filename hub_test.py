#!/usr/bin/env python3
import pyads
import time

# Verbindungsparameter
PLC_AMS_ID = "192.168.100.1.1.1"
PLC_IP = "192.168.100.1"

def run_hub_test():
    plc = pyads.Connection(PLC_AMS_ID, 801, PLC_IP)
    try:
        plc.open()
        print("✅ ADS Verbindung offen.")

        print("🔓 Deaktiviere .TEST_OHNE_HUB...")
        plc.write_by_name(".TEST_OHNE_HUB", False, pyads.PLCTYPE_BOOL)

        target_height = 600 
        print(f"🚀 Starte Test-Zyklus: Ziel {target_height}mm")

        for i in range(50):
            # Bit 0: Heartbeat, Bit 1: AIC_Predominance
            hb_bit = i % 2
            control_word = hb_bit | (1 << 1)
            plc.write(0xF020, 300, control_word, pyads.PLCTYPE_WORD)

            if i == 5:
                print(f"\n  -> Sende Ziel {target_height} an Offset 326")
                plc.write(0xF020, 326, target_height, pyads.PLCTYPE_INT)
                print("  -> Sende Start-Kommando (3) an Offset 328")
                plc.write(0xF020, 328, 3, pyads.PLCTYPE_WORD)

            try:
                ist_pos = plc.read(0xF030, 326, pyads.PLCTYPE_INT)
                status_word = plc.read(0xF030, 328, pyads.PLCTYPE_WORD)
                ready = bool(status_word & (1 << 0))
                moving = bool(status_word & (1 << 1))
                print(f"  [{i}] Ist: {ist_pos}mm | Ready: {ready} | Moving: {moving}", end="\r")
            except:
                pass

            time.sleep(0.1)

        print("\n🛑 Test beendet. Sende Stop.")
        plc.write(0xF020, 328, 0, pyads.PLCTYPE_WORD)
        plc.close()

    except Exception as e:
        print(f"\n❌ Fehler: {e}")

if __name__ == "__main__":
    run_hub_test()
