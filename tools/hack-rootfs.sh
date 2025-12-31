#!/bin/bash

set -x


if [ -f /lib/systemd/system/lightdm.service ];then
	systemctl enable lightdm
fi

if [ -f /lib/systemd/system/gdm.service ];then
	systemctl enable gdm
fi

chmod +x /etc/rc.local

echo ok > /etc/hack

