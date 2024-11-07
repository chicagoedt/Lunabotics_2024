import serial
import time
import socket
import subprocess

def get_ip_address():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))  # googles dns
        ip_address = s.getsockname()[0]
    except Exception as e:
        ip_address = 'Error: No IP'
    finally:
        s.close()
    return ip_address

def get_ssid():
    try:
        ssid = subprocess.check_output(['iwgetid', '-r']).decode().strip()
        if not ssid:
            ssid = 'Error: No SSID'
    except Exception as e:
        ssid = 'Error: No SSID'
    return ssid

def main():
    
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    time.sleep(2) 

    while True:
        ip_address = get_ip_address()
        ssid = get_ssid()

        
        line1 = 'IP: ' + ip_address
        line2 = 'SSID: ' + ssid

       
        line1 = line1[:16]
        line2 = line2[:16]

        
        ser.write((line1 + '\n').encode())
        time.sleep(0.1)
        ser.write((line2 + '\n').encode())

        
        time.sleep(5)

if __name__ == '__main__':
    main()
