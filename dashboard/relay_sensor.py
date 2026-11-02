#!/usr/bin/env python3
"""Local-only USB relay for SSH forwarding; exactly one serial client at a time."""
import argparse,glob,os,select,socket,time
import serial

def serve(port=7000):
    paths=glob.glob('/dev/serial/by-id/*VL53L9*')
    if len(paths)!=1:raise RuntimeError('Expected exactly one VL53L9 USB device')
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        listener.bind(('127.0.0.1',port));listener.listen(2)
        print(f'LiDAR relay listening on localhost:{port}',flush=True)
        while True:
            client,_=listener.accept()
            try:
                with client,serial.Serial(paths[0],115200,timeout=.1,write_timeout=2,exclusive=True) as usb:
                    client.settimeout(2);client.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                    try:
                        # Stop leftovers from an interrupted viewer before negotiation.
                        usb.write(b'P');deadline=time.monotonic()+2
                        while usb.read(16384):
                            if time.monotonic()>deadline:raise RuntimeError('Sensor did not stop')
                        while True:
                            ready,_,_=select.select([client,usb.fileno(),listener],[],[],.2)
                            if listener in ready:
                                extra,_=listener.accept();extra.close()
                            if client in ready:
                                data=client.recv(4096)
                                if not data:break
                                usb.write(data)
                            if usb.fileno() in ready:
                                data=os.read(usb.fileno(),32768)
                                if data:client.sendall(data)
                    finally:
                        usb.write(b'P');usb.flush()
            except (OSError,serial.SerialException,RuntimeError) as exc:
                client.close();print(f'Client disconnected: {exc}',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--port',type=int,default=7000)
    serve(p.parse_args().port)
