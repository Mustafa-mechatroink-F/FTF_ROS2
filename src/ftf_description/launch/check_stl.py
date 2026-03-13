import struct

def convert_stl_to_binary(input_file, output_file):
    try:
        with open(input_file, 'r', encoding='utf-8', errors='ignore') as f:
            line = f.readline().strip()
            is_ascii = line.startswith('solid')
        
        if is_ascii:
            print(f"Datei {input_file} ist im ASCII-Format. Konvertiere zu Binary...")
            # Hinweis: Eine echte Konvertierung benötigt Bibliotheken wie 'numpy-stl'
            # Einfachster Weg: Nutze MeshLab oder Blender und 'Export as Binary STL'
        else:
            print(f"Datei {input_file} ist bereits im Binary-Format. Das ist gut!")
    except Exception as e:
        print(f"Fehler beim Lesen: {e}")

# Pfad anpassen!
convert_stl_to_binary('/home/ftf2/fh_ws/src/ftf_description/meshes/FRF_Roboter.STL', 'FTF_Roboter_fixed.STL')