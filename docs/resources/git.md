# General Git Info

Git is a tool for version control & collaboration. Without tools like Git, resolving issues arising from multiple people working in the same codebase is difficult. A problem that can occur is conflicts due to different people making edits to the same file.

Git also preserves the file history. This enables us to restore a file to any point in its history.

## Git Credentials

A lot of our git repositories are private. In order to acquire these repositories for local use through Git, you need to configure git on your local machine such that it can authenticate itself to GitHub. Possibly the easiest way to do this is by just using the GitHub CLI.

- [Installation Guide for the GitHub CLI](https://cli.github.com/)

Once you have installed the GitHub CLI, to configure your git parameters, run the command:

```
gh auth login
```

Another way you can set up git credentials is by using something called an SSH key. There's a guide on how to use that approach here:

- [Adding a new SSH key to your GitHub account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)

## Learning Git

Here's a useful video for learning git: [Git Tutorial for Beginners: Learn Git in 1 Hour](https://www.youtube.com/watch?v=8JJ101D3knE)

### Useful List of Commands

```
# Initialize a new Git repository in the current directory
git init
```

```
# Display the state of the working directory and staging area
git status
```

```
# Create a new branch and switch to it
git checkout -b <branch_name>
```

```
# Add specific files to the staging area
git add <file_1> <file_2> … <file_n>
```

```
# Add all changes (tracked and untracked) in the current directory to the staging area
git add .
```

```
# Commit staged changes with a descriptive commit message
git commit -m “commit message”
```

```
# Push local commits to the remote repository`
git push
```

```
# Push the current branch to the remote repository and set the upstream tracking reference for future pushes. It is equivalent to the command that Git displays after you try to run git push on a new branch.
git ush -u origin HEAD`
```

```
# Show changes between the working directory and the last commit
git diff`
```

```
# Show commit history in reverse chronological order
git log
```

```
# Merge the specified branch into the current branch
git merge <branch_name>
```

```
# Reapply commits from the specified branch on top of the current branch (rebase)
git rebase <branch_name>
```

### Extra Useful Commands

```
# Modify the most recent commit, updating the message or adding changes
git commit --amend -m “new commit message”
```

```
# Undo the last commit, but keep the changes in the staging area
git reset --soft HEAD~1
```

```
# Undo the last commit and discard all changes from the working directory
git reset --hard HEAD~1
```

```
# Interactively stage parts of a file
git add -p
```

```
# Display the commit history in a condensed, one-line format
git log --oneline
```

```
# Show a formatted log of commits with specific details (hash, author, date, message). See [1] in the Reference section
git log --pretty=ormat:"%h%x09%an%x09%ad%x09%s"`
```

```
# Show the log along with the patch of differences for each commit
git log -p
```

```
# List all local branches in the repository
git branch --list
```

```
# List all branches, both local and remote
git branch -a
```

```
# Save uncommitted changes temporarily and remove them from the working directory
git stash
```

```
# Apply the most recently stashed changes and remove them from the stash list
git stash pop
```

```
# Set the global Git username
git config --global user.nae “First Last”`
```

```
# Set the global Git email
git config --global useremail “name@gmail.com”`
```

```
# Restore file contents in the working directory to the latest committed state
git restore` `<file_1> <file_2> … <file_n>
```

```
# Unstage changes for the specified files, keeping them in the working directory
git restore --staged <file_1> <file_2> … <file_n>
```

```
# Interactively rebase commits, allowing you to edit, squash, or reorder them
git rebase -i
```

### Note

Most commands listed above have multiple options for different behaviours. See the documentation for more.

## Tips

- There can be a learning curve with getting used to git, this is normal. 
- Leveraging tools in a code editor that make using Git easy without the terminal is fine\! Even if you prefer to use the terminal these tools can be useful for handling merge conflicts, viewing the file diffs, and seeing the author of each line of code in a file. However, learning the terminal commands is useful as you might need to develop on remote machines that might not have these editors/tools available.
- If using VS Code, install the git lens extension
- Test dangerous commands you are unsure of in a new branch and see if they solve your problem before running them in the current branch.
- When rebasing to a branch already in the remote repository, you need to force push (add `--force` to your git push command).

### Important Tip

There are ways to undo mistakes from running almost every git command, just google search the situation and you will find a solution. What will worsen the situation is running commands you are unsure of in the hope that it solves the problem.

- [Oh Shit, Git\!?\!](https://ohshitgit.com/)  
- [How to undo (almost) anything with Git \- The GitHub Blog](https://github.blog/open-source/git/how-to-undo-almost-anything-with-git/)  
- [Common Git Problems and Their Fixes \- GeeksforGeeks](https://www.geeksforgeeks.org/common-git-problems-and-their-fixes/)

The links above have tips to fix mistakes that occur while using git.

## Extra

### GitHub

### Git Alias

[10 levels of Git aliases: Beginner to intermediate concepts](https://www.eficode.com/blog/10-levels-of-git-aliases-beginner-to-intermediate-concepts)

### Merge vs Rebase

[Merging vs. Rebasing | Atlassian Git Tutorial](https://www.atlassian.com/git/tutorials/merging-vs-rebasing)

### The `git cherry-pick` command

[Git Cherry Pick | Atlassian Git Tutorial](https://www.atlassian.com/git/tutorials/cherry-pick)

### Refs & Reflog

[Git Refs: What You Need to Know | Atlassian Git Tutorial](https://www.atlassian.com/git/tutorials/refs-and-the-reflog)

### The `git rerere` command

[Resolving conflicts with git-rerere](https://bitbucket.org/blog/resolving-conflicts-with-git-rerere)

### How Git Works

[10.1 Git Internals \- Plumbing and Porcelain](https://git-scm.com/book/en/v2/Git-Internals-Plumbing-and-Porcelain) (The whole chapter 10 not just 10.1)

## Reference

1. [git log formatted stack overflow](https://stackoverflow.com/questions/1441010/the-shortest-possible-output-from-git-log-containing-author-and-date#:~:text=git%20log%20%2D%2Dpretty%3Dformat%3A%22%25h%25x09%25an%25x09%25ad%25x09%25s%22)