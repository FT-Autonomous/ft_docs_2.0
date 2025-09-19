# General SSH Resources

### Tmux

Sometimes you want to have multilpe terminal windows on the remote host that you're SSHing into. You could start multiple terminals and run the `ssh TARGET` command in each one, but there are better ways of doing this. It's reccomended to use the `tmux` (terminal multiplexer) program.

{% image src="https://github.com/tmux/tmux/wiki/images/tmux_with_panes.png" width="400px" /%}

At a surface level, just from the image above, you can see that tmux allows you to define multiple terminal panes in a single SSH session. This is good but this is not even the main advantage of tmux. The main advantage of tmux is the concept of an SSH session whose state is stored on the remote host. Essentially, you can run the following command to start a tmux sessin called `my-session`:

```
ssh TARGET -t tmux new -t my-session
```

Then, in the case where the SSH connection is terminated, either intentionally as you decided to close your laptop or the connection was dropped, you can simply SSH in again and run:

```
ssh TARGET -t tmux attach -t my-session
```

This command will resume the SSH connection with all your panes already opened and everything you were working on will be restored! `tmux` has even more powerful features like allowing you to define scripts that spin up panes automatically ([see how we use this here](https://github.com/FT-Autonomous/ft_misperception/blob/ed28699216ea9891955376fd6dd1f73e374db0ff/src/ft_calibration/puppetmaster.py#L33-L39)).

Here are some usful links for learning about how `tmux` works:

- [`tmux` getting started guide on GitHub](https://github.com/tmux/tmux/wiki/Getting-Started)
- [`tmux` manual page](https://duckduckgo.com/?t=ffab&q=tmux+manpage&ia=web)
- [How to scroll in `tmux`](https://superuser.com/a/209608)

### Copying Files

You can use rsync to copy files to or from an SSH target.

- [Archwiki entry on rsync](https://wiki.archlinux.org/title/Rsync#As_cp/mv_alternative)
- [USAGE section of the `rsync` manpage](https://man.archlinux.org/man/rsync.1.html#USAGE)

### X11 Forwarding

It's possible for graphics to be done over SSH though it's extremely slow ([and also insecure!](https://security.stackexchange.com/questions/14815/security-concerns-with-x11-forwarding)). You can do it with the following command:

```
ssh -XY TARGET
```

Runing graphical applications on your laptop will then open windows on your computer, provided you are using WSL, are on Linux or are on MacOS with some X11 server running. Only use this technique with trusted servers.

### Passwordless Authentication

If you have an SSH key set up on your comptuer, you can configure SSH targets to accept that key instead of a password for authentication. To do this, first get the public key of your SSH key. This is done using the `ssh-add -L` command. The output should be somthing like the following:

```
ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAACAQDlLE … (none)
```

You then want to navigate to the remote host (for example, macneill or the FTA PC) and add this as an entry in the `~/.ssh/authorized_keys` file.