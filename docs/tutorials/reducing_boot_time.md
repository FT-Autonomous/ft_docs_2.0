---
title: Speeding up Ubuntu Boot Times
---

## Speeding up Ubuntu Boot Times

Time is your most valuable asset at competition. The last thing you want to be doing is waiting 2 minutes for ubuntu to boot up, or wait another two minutes mid-race to reboot the computer. To this end you want to minimise your boot time.

[https://askubuntu.com/questions/1417176/change-ubuntu-splash-screen](https://askubuntu.com/questions/1417176/change-ubuntu-splash-screen)

Remove “quiet” and “splash” from `GRUB_CMDLINE_LINUX_DEFAULT`. [https://askubuntu.com/questions/25022/how-to-enable-boot-messages-to-be-printed-on-screen-during-boot-up](https://askubuntu.com/questions/25022/how-to-enable-boot-messages-to-be-printed-on-screen-during-boot-up).

You should then be able to see where the boot process is stalling. In our case, ubuntu was waiting on a disk that did not exist. This drive was present in the fstab entry but had not actually existed for a long time.

Consider trying to disable snap at boot: <https://forum.snapcraft.io/t/extented-boot-time-due-to-snap/26900/9>.

You want to look at the output of `journalctl –boot`.

Disable useless processes

```bash
sudo systemctl disable openvpn.service
sudo systemctl disable snapd.service snapd.seeded.service
sudo systemctl disable bluetooth
sudo systemctl disable cups.path cups.socket cups-browsed.service cups.service
sudo systemctl disable snapd.apparmor.service
sudo systemctl disable nfs-server.service
```

Consider switching to the multi-user.target (<https://www.tecmint.com/change-runlevels-targets-in-systemd/>).

```
sudo systemctl set-default multi-user.target
```

Enable automatic login <https://help.ubuntu.com/stable/ubuntu-help/user-autologin.html.en>.

You may need to manually change /boot/grub/grub.cfg. Search for timeout=10 – this is the default value timeout is set to in cases where timeout is set to zero. Change this line to retain the timeout=0 option.

```bash
sudo systemctl disable networkd-dispatcher.service
sudo systemctl disable apparmor
```

Consider switching to another window manager. Using:

```bash
sudo update-alternatives --config x-session-manager
```

Doesn’t seem to work, so consider just logging out and changing the window manager manually if you are using Ubuntu.

## Startup Scripts

Technically, you are not supposed to be allowed to use a keyboard and mouse to interface with the car at competition. There was an effort during the 2023/2024 competition to enforce this but they slowly stopped caring seeing as most teams were not prepared for that. Despite this, it is still a good idea to invest time in creating reliable startup scripts because they do indeed make things much more streamlined. You should be able to use the Ubuntu GUI to configure scripts to start after login. Combined with automatic login, you get a script that starts on boot.