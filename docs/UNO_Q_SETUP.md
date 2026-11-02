# Arduino UNO Q setup

UNO Q runs Linux processing and the native GTK dashboard. Use its powered USB host arrangement to connect the LiDAR. Replace `192.168.1.21` with your UNO Q address.

## Install

On UNO Q:

```bash
sudo apt update
sudo apt install python3-gi gir1.2-gtk-3.0 python3-cairo python3-numpy python3-serial python3-opengl
mkdir -p ~/lidar-dashboard ~/lidar-python
df -h "$HOME"
```

From the repository root on the laptop (matches the article):

```bash
scp -r uno-q/lidar-dashboard/. arduino@192.168.1.21:lidar-dashboard/
scp -r uno-q/lidar-python/. arduino@192.168.1.21:lidar-python/
```

The dependency snapshot contains pyserial and PyOpenGL; NumPy, GTK and Cairo come from the system packages. The supplied ARM library is already in the UNO Q snapshot's `build/` directory. These home directories survive reboot.

On UNO Q, check dependencies and USB:

```bash
PYTHONPATH="$HOME/lidar-python" python3 -c 'import gi, cairo, numpy, serial; print("Dependencies OK")'
lsusb
ls -l /dev/serial/by-id/
```

If serial access is denied, run `sudo usermod -aG dialout "$USER"`, then log out and back in.

## Run inside the UNO Q VNC desktop

```bash
~/lidar-dashboard/run_gui.sh --log
```

The launcher defaults to DISPLAY=:1. For a different VNC display:

```bash
DISPLAY=:2 ~/lidar-dashboard/run_gui.sh --log
```

`CPU renderer fallback: CPU rendering selected` is expected on the tested UNO Q VNC setup. Close the GUI to stop; nothing is installed as an automatic boot service.

## Optional laptop receiver

The LiDAR stays plugged into UNO Q. Close the UNO Q local dashboard first. On a Linux laptop, from this repository:

```bash
sudo apt install python3-gi gir1.2-gtk-3.0 python3-cairo python3-numpy python3-serial python3-opengl
mkdir -p dashboard/build
cp dashboard/prebuilt/linux-x86_64/libvl53l9_transform.so dashboard/build/
ssh-copy-id arduino@192.168.1.21
./dashboard/view_remote.sh arduino@192.168.1.21
```

Use the aarch64 prebuilt instead if your laptop is ARM64. The launcher requires working SSH key authentication and starts its own relay and tunnel on port 7000. Do not also run socat or another tunnel. ST processing and rendering run on the laptop in this mode. For direct USB to the laptop, run `python3 -B dashboard/lidar_gui.py --usb auto --log` instead.

## Troubleshooting

- **Address already in use:** close the previous launcher/tunnel before restarting. Do not SSH from UNO Q back into itself to create the laptop tunnel.
- **Could not exclusively lock port:** another GUI or relay owns the sensor. Close it; only one viewer may acquire frames.
- **USB board not found:** check the data cable, power and USB host role. Do not start the dashboard until the board enumerates.
- On the tested UNO Q image, inspect `cat /sys/class/usb_role/4e00000.usb-role-switch/role`. If this path exists and says `device`, select host with `echo host | sudo tee /sys/class/usb_role/4e00000.usb-role-switch/role`, then check USB again. This is image-dependent and may need repeating after reboot.
- If the board only boots with ST-Link attached, inspect reset, power and boot configuration and test USB-only cold startup. A dashboard reinstall cannot fix a hardware startup fault.
- **Connection refused:** the relay/tunnel is absent. Use the single laptop launcher above.

Space pauses/resumes, drag rotates, Shift-drag pans, wheel zooms, F fits the cloud. Confidence and surface controls change presentation/filtering, not sensor resolution. Received FPS is not display FPS.
