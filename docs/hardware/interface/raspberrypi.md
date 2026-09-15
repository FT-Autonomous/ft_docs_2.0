# Raspberry Pi Setup Guide

This guide details how to boot into a GUI using the Raspberry Pi that we are using for the AI/ADS interface. In reality we are cheap so it is a Rock C4+ but aside from Radxa dropping support for it in like 2021 it is basically the same thing.

## Booting

1. Find any USB C power supply (EG. phone charger) and plug it in. In my experience, ones capable of fast charging and that only have one USB port on them are less likely to lead to random crashes etc. but any one should work in a pinch.
2. A green LED will show it is being powered, and a blue LED will blink to show that it is reading data from memory and starting to boot. If either of these doesn't work, try a different power supply and cable.
3. Once booted (will take a minute or two) type in the usename and password (both "fta")
4. Once logged in, type "startx". Give it another minute and it will display a GUI. We are using i3wm due to it being very lightweight.
5. If you want to connect to the internet, open the terminal and type in nmtui and connect through that.

## i3wm Keybinds

If you are unfamiliar with i3wm's keybinds, here is some of the most important ones:

`$mod` = Win key

| Keybind | Action |
|---|---|
| `$mod+Enter` | Open terminal |
| `$mod+d` | Open dmenu (app launcher) |
| `$mod+q` | Kill focused window |
| **Navigation** | |
| `$mod+j/k/l/;` | Focus left/down/up/right |
| `$mod+Arrow keys` | Focus left/down/up/right |
| `$mod+Shift+j/k/l/;` | Move window left/down/up/right |
| `$mod+Shift+Arrow keys` | Move window left/down/up/right |
| **Layout** | |
| `$mod+h` | Split horizontally |
| `$mod+v` | Split vertically |
| `$mod+s` | Stacking layout |
| `$mod+w` | Tabbed layout |
| `$mod+e` | Toggle split layout |
| `$mod+f` | Toggle fullscreen |
| `$mod+Shift+Space` | Toggle floating |
| `$mod+Space` | Toggle focus between tiling/floating |
| **Workspaces** | |
| `$mod+1-9` | Switch to workspace 1-9 |
| `$mod+Shift+1-9` | Move window to workspace 1-9 |
| **Resize** | |
| `$mod+r` | Enter resize mode |
| (in resize mode) `j/k/l/;` or arrows | Resize window |
| (in resize mode) `Escape` or `Enter` | Exit resize mode |
| **Session** | |
| `$mod+Shift+c` | Reload config |
| `$mod+Shift+r` | Restart i3 in-place |
| `$mod+Shift+e` | Exit i3 |
