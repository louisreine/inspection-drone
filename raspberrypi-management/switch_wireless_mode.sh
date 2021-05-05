#! /bin/bash
arg1=$1

if [ $arg1 == "ON" ] ; then
        sed -i '1,4 s/^#*//'  /etc/dhcpcd.conf  &&
        systemctl daemon-reload &&
        service dhcpcd restart &&
        systemctl start dnsmasq &&
        systemctl start hostapd &&
        systemctl enable dnsmasq &&
        systemctl enable hostapd &&
elif [ $arg1 == "OFF" ] ; then
        systemctl stop dnsmasq &&
        systemctl stop hostapd &&
        systemctl disable dnsmasq &&
        systemctl disable hostapd &&
        sed -i '1,4 s/^/#/'  /etc/dhcpcd.conf
        systemctl daemon-reload &&
        service dhcpcd restart &&
else
        printf "UNKNOWN input."
fi