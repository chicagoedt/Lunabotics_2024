import serial
import time
import socket
import subprocess

def get_ip_address():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))  # googles dns
        ip_address = s.getsockname()[0]
    except Exception:
        ip_address = 'Error: No IP'
    finally:
        s.close()
    return ip_address

def get_ssid():
    try:
        ssid = subprocess.check_output(['iwgetid', '-r']).decode().strip()
        if not ssid:
            ssid = 'Error: No SSID'
    except Exception:
        ssid = 'Error: No SSID'
    return ssid
# switching ip to hex to display (bypass 16 characters, pls work)
def ip_to_hex(ip_address, with_dots=False):
    try:
        octets = ip_address.split('.')
        hex_octets = [format(int(octet), '02X') for octet in octets]
        if with_dots:
            hex_ip = '.'.join(hex_octets)
        else:
            hex_ip = ''.join(hex_octets)
    except:
        hex_ip = 'Error'
    return hex_ip

def get_jetson_temperature():
    try:
        temp_str = subprocess.check_output(
            ['cat', '/sys/devices/virtual/thermal/thermal_zone1/temp']).decode().strip()
        temp_c = int(temp_str) / 1000.0  
        temp_f = (temp_c * 9/5) + 32
        return temp_c, temp_f
    except:
        return None, None

def main():
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    time.sleep(2)  # timesleep to wait for intliization, testing / googling this said todo this.

    while True:
        if ser.in_waiting > 0:
            request = ser.readline().decode().strip()
            if request == "REQUEST_IP_SSID":
                ip_address = get_ip_address()
                ssid = get_ssid()

                if 'Error' not in ip_address:
                    hex_ip = ip_to_hex(ip_address, with_dots=False)
                    display_ip = 'IP: 0x' + hex_ip
                else:
                    display_ip = ip_address  

                line1 = display_ip[:16]
                line2 = 'SSID: ' + ssid[:9]  

                ser.write(('IP_SSID\n').encode())
                time.sleep(0.1)
                ser.write((line1 + '\n').encode())
                time.sleep(0.1)
                ser.write((line2 + '\n').encode())

            elif request == "REQUEST_TEMP":
                temp_c, temp_f = get_jetson_temperature()

                if temp_c is not None:
                    line1 = 'Jetson Temp:'
                    line2 = '{:.1f}C/{:.1f}F'.format(temp_c, temp_f)
                else:
                    line1 = 'Temp Error'
                    line2 = ''

                line1 = line1[:16]
                line2 = line2[:16]

                ser.write(('TEMP\n').encode())
                time.sleep(0.1)
                ser.write((line1 + '\n').encode())
                time.sleep(0.1)
                ser.write((line2 + '\n').encode())

        time.sleep(0.1)  # optimization here, high cpu usage otherwise.

if __name__ == '__main__':
    main()
