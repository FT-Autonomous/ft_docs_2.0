---
title: How to SSH into the FTA PC
---

## How to SSH into the FTA PC

The conventional way to access the FTA PC is over the Remote Desktop Protocol (RDP) interface. How to do this is documented in the team google drive. While RDP allows you to accomplish a lot of tasks on the FTA PC, would SSH interface allows us to accomplish a similar set of tasks and also comes with its own benefits that are impossible to achieve using the RDP protocol alone.

Some of the inconveniences associated with RDP include:

- You can't do things such as use the vscode dev server over SSH.
- It's non trivial to beam files directly from your computer to the FTA PC or vice versa.
- The RDP interface itself is quite sluggish and annoying to use especially when all you need is a terminal with tmux.

### SSH Tunnelling Method

Generally, to make the FTA PC accessible over the internet, we would open port 22 (the TCP port used for SSH) in the firewall, spin up the `ssh` daemon in the FTA PC get right to it. The issue is that we do not control the firewall that TCD uses to secure the college network and TCD is averse to opening ports to the world outside of the college network if it does not nead to, understandably so. Long story short, the FTA PC cannot be reached directly from our own PC on port 22 over the internet!

There is a work around for this however, whch is more secure than opening ports to the internet. We can leverage a mechanism called **ssh tunnelling**. The underlying principle is that if we have a server accessible on the internet to both the FTA PC and our own PC, we can leverage a mechanism in the `ssh` protocol that allows us to proxy through SSH servers to get to our destination.

The only prerequisite to this method is that you have access to a globally accessible SSH server. Both computer science and engineering students at TCD students should have such a server in the form of `macneill`.

{% image src="/SSH INTO FTA PC.drawio.svg" /%}

#### On the FTA PC

The first step is to establish the right hand side of the connection. This is actually done by the FTA PC calling out to the globally accessible SSH server (macneill). This sounds a bit backwards, but remember that the FTA PC can't be accessed inbound over SSH.

Technically, only one person needs to establish the right hand side of the tunnel. It would be easier to just rely on one single connection on the right side but this relies on coordination between different people so it's just reccomended to make your own. The only issue that arises with people creting their own connections is that each person has to select their own TCP port to use on macneill to represent the outbound connection. To slect such ap port, pick a random number from 1024-65535.

Thus there will be three "variables" in play which you will have to replace while following this guide:

- `PORT` - The random port that you chose.
- `MACNEILL-USERNAME` - Generally the same as your username for submitty. The password is the same as well.
- `FTA-PC-USERNAME` - This is generally your first name, or whatever username you have used to connect to the FTA PC in the past.

To set up the link, open a terminal on the FTA PC and use the following command:

```bash
ssh -NR PORT:0.0.0.0:22 MACNEILL-USERNAME@macneill.scss.tcd.ie
```

#### Locally

All that's left is to create and use the left hand side of the connection. On the other host (your laptop), to connect use the command:

```bash
ssh -o ProxyCommand="ssh -W %h:%p MACNEILL-USERNAME@macneill.scss.tcd.ie" FTA-PC-USERNAME@localhost -p PORT
```

This is a bit much to type out every time we want to connect to the FTA PC. The `ssh` program provides a way of defining `ssh` setups in the `~/.ssh/config` file.  You may then want to add the following to that file:

```config
Host ft
     HostName localhost
     Port PORT
     ProxyCommand ssh -W %h:%p MACNEILL-USERNAME@macneill.scss.tcd.ie
     User FTA-PC-USERNAME
```

Now, locally, you can run the command `ssh ft` to SSH into the FTA PC so long as the terminal in your user account establishing the right hand side of the connection is active!

#### Further Reading

- [General SSH Tips and Resources](../resources/ssh.md)

#### Making the FTA PC Side Connection More Reliable

You can use the following systemd user unit in `~/.config/systemd/user/tunnel.service`.

```
[Unit]
Description=Setup a secure tunnel to macneill
After=network.target

[Service]
ExecStart=/usr/bin/ssh -NTR PORT:0.0.0.0:22 MACNEILL-USERNAME@macneill.scss.tcd.ie
RestartSec=5
Restart=always

[Install]
WantedBy=multi-user.target
```

You can then enable it with:

```
systemctl --user enable tunnel
systemctl --user restart tunnel
```