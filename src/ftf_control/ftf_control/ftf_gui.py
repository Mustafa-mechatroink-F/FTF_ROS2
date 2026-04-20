import tkinter as tk
import subprocess
import os
from PIL import Image, ImageTk
from ament_index_python.packages import get_package_share_directory
import rclpy

def main():
    if not rclpy.ok(): rclpy.init()
    
    root = tk.Tk()
    root.title("HAW Kiel - FTF2_CIMTT FB Maschienenwesen")
    root.geometry("500x750")
    root.configure(bg='#ffffff')

    try:
        pkg_share = get_package_share_directory('ftf_control')
        path_ftf = os.path.join(pkg_share, 'images', 'FTF2.jpeg')
        path_logo = os.path.join(pkg_share, 'images', 'HAW.png')
    except:
        path_ftf = path_logo = ""

    # HAW Logo
    try:
        logo_img = ImageTk.PhotoImage(Image.open(path_logo).resize((180, 80), Image.LANCZOS))
        tk.Label(root, image=logo_img, bg='white').pack(pady=10, anchor="ne", padx=20)
        root.logo_img = logo_img # Garbage Collection Schutz
    except: tk.Label(root, text="HAW HAMBURG", bg='white').pack()

    # Roboter Foto
    try:
        robot_img = ImageTk.PhotoImage(Image.open(path_ftf).resize((380, 280), Image.LANCZOS))
        tk.Label(root, image=robot_img, bg='white', bd=3, relief="ridge").pack(pady=20)
        root.robot_img = robot_img
    except: tk.Label(root, text="[ FOTO FEHLT ]", bg='grey').pack()

    # Buttons
    tk.Button(root, text="LADEN STARTEN", bg="#00549f", fg="white", font=("Arial", 14, "bold"), 
              width=25, height=2, command=lambda: subprocess.run(['ros2', 'topic', 'pub', '-1', '/ftf/start_mission', 'std_msgs/msg/Bool', '{data: true}'])).pack(pady=15)

    tk.Button(root, text="ENTLADEN STARTEN", bg="#333333", fg="white", font=("Arial", 14, "bold"), 
              width=25, height=2, command=lambda: subprocess.run(['ros2', 'topic', 'pub', '-1', '/ftf/start_unload', 'std_msgs/msg/Bool', '{data: true}'])).pack(pady=15)

    root.mainloop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()