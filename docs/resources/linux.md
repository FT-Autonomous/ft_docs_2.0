# Introduction

The terminal, file system & package management will be the biggest changes coming from Windows/Mac. You might be familiar with the terminal already but there are some Ubuntu/Linux specific commands to know. There are similarities coming from Mac as its operating system is also Unix based.

## Terminal

- [60 Linux Commands you NEED to know (in 10 minutes)](https://youtu.be/gd7BXuUQ91w?si=U5b7RRBQ0lt8CLDd)
- [Linux Command Line Cheat Sheet: All the Commands You Need](https://www.stationx.net/linux-command-line-cheat-sheet/)

## File System

- [the Linux File System explained in 1,233 seconds // Linux for Hackers // EP 2](https://www.youtube.com/watch?v=A3G-3hp88mo)

## Ubuntu Package Management 

- [apt, dpkg, git, Python PiP (Linux Package Management) // Linux for Hackers // EP 5](https://youtu.be/vX3krP6JmOY?si=OMlhfmYDZBWd-srq)
- [apt Command Examples for Ubuntu/Debian Linux \- nixCraft](https://www.cyberciti.biz/faq/ubuntu-lts-debian-linux-apt-command-examples/)

## Gedit

To use gedit in Ubuntu, open the app directly or from your terminal by running '`gedit &`'. In unix-based systems liike linux, the  '`&`' opens the program in the background. This way you can use the same terminal tab to do other things. 

### Adding plugins to gedit

- [10 Tweaks to Supercharge Gedit as Code Editor](https://itsfoss.com/gedit-tweaks/)

## Extra

### Vim

- [Getting started with Vim: The basics | Opensource.com](https://opensource.com/article/19/3/getting-started-vim)

## Getting Faster in the Terminal

The following sections are a couble of easy to implement tips and tricks that will speed up a lot of actions on the terminal.

## Shell Aliases

There are a lot of actions in formula trinity that you'll have to do almost any time you do something in the terminal formula trinity related. For example, you're always going to have to navigate to your workspace folder and source the setup script as follows:

```
cd ~/ft
. /opt/ros/humble/setup.bash
. install/setup.bash
```

A shell alias allows us to define a new command that does all this stuff for us so we spend less time doing repetitive menial labour. To define a shell alias for this particular case, you would add the following to your `~/.bashrc` file:

```
alias .w='cd ~/ft ; . /opt/ros/humble/setup.bash ; . install/setup.bash'
```

Now, when you start a new terminal, typing in `.w'` will do the three commands in one fell swoop! You can define other aliases as well, the following are some cool ones that seen around FT:

```
alias .r='. install/setup.bash'
alias .c='colcon build --symlink-install'
```

## Reverse Search

Adding reverse search to your linux workflow is **possibly the most significant thing you can do to speed things up**. Reverse search is a skill so powerful that it can even replace aliases in your workflow.

To perform reverse search in your terminal, you use the keybind `Ctrl + R`. You can then type out any fragment of any previous command and reverse search will try to find it in your shell history!

To navigate through the past for more candidates, press `Ctrl + R` again while in reverse search mode. Once you've found a candidate, you can press any of the arrow keys to select the match.

When the terminal is in reverse search mode, you can press `Ctrl + C` or `Ctrl + G` to leave reverse search and discard the results of the search. 

## Vibe Navigating

You should watch the demo video and read the README for zoxide, which is a modern alternative to the `cd` command -- it's the future of navigation: https://github.com/ajeetdsouza/zoxide (written by Cleavon's brother from another mother). The installation instructions are there also. In summary:

```
curl -sSfL https://raw.githubusercontent.com/ajeetdsouza/zoxide/main/install.sh | sh
```

Then add the following to your `~/.bashrc`:

```
eval "$(zoxide init bash)"
```