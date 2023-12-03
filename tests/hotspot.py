import subprocess


def create_hotspot():

    try:
        # Configure access point
        subprocess.run(["sudo", "systemctl", "stop", "hostapd"])
        subprocess.run(["sudo", "systemctl", "stop", "dnsmasq"])

        with open("/etc/dhcpcd.conf", "a") as file:
            file.write("\ninterface wlan0\n    static ip_address=192.168.4.1/24\n    nohook wpa_supplicant\n")

        # Configure access point settings
        subprocess.run(["sudo", "cp", "/etc/dnsmasq.conf", "/etc/dnsmasq.conf.orig"])
        subprocess.run(["sudo", "sed", "-i", "s/^interface=.*$/interface=wlan0/", "/etc/dnsmasq.conf"])
        subprocess.run(["sudo", "sed", "-i", "s/^dhcp-range=.*$/dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h/", "/etc/dnsmasq.conf"])

        # Start access point services
        subprocess.run(["sudo", "systemctl", "start", "hostapd"])
        subprocess.run(["sudo", "systemctl", "start", "dnsmasq"])

        print(f"Hotspot created with password")
    except Exception as e:
        print(f"Error creating hotspot: {e}")


def stop_hotspot():
    try:
        subprocess.run(["sudo", "systemctl", "stop", "hostapd"])
        subprocess.run(["sudo", "systemctl", "stop", "dnsmasq"])
        print("Hotspot stopped successfully")
    except Exception as e:
        print(f"Error stopping hotspot: {e}")


if __name__ == "__main__":
    while True:
        num = int(input("Enter value"))
        if num == 1:
            create_hotspot()
        else: 
            stop_hotspot()
