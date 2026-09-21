# Automated Documentation Agent

## Overview

This update introduces an **automated documentation system** that generates documentation updates whenever changes are made to the pure pursuit controller. The system uses Claude AI to analyze code changes and automatically creates pull requests in the documentation repository.

## How It Works

When a pull request is opened or updated in the `ft_pure_pursuit` repository, a GitHub Actions workflow automatically:

1. **Fetches the PR diff** - Retrieves all changed files and their modifications
2. **Sends changes to Claude AI** - The diff is analyzed by Claude (claude-opus-4-5 model) to generate relevant documentation
3. **Creates a documentation PR** - A new pull request is automatically opened in the `ft_docs_2.0` repository with the suggested documentation updates

## Components

### GitHub Actions Workflow (`.github/workflows/doc-agent.yml`)

| Trigger | Description |
|---------|-------------|
| `pull_request: opened` | Runs when a new PR is created |
| `pull_request: synchronize` | Runs when new commits are pushed to an existing PR |

### Doc Agent Script (`.github/scripts/doc_agent.py`)

The Python script that orchestrates the documentation generation process.

#### Required Environment Variables

| Variable | Description |
|----------|-------------|
| `ANTHROPIC_API_KEY` | API key for Claude AI authentication |
| `GH_PAT` | GitHub Personal Access Token with repo permissions |
| `PR_NUMBER` | The pull request number (set automatically) |
| `REPO_NAME` | The source repository name (set automatically) |

#### Key Behaviours

- **Branch Management**: Creates a new branch `auto-docs/pr-{PR_NUMBER}` in the docs repository. If the branch already exists, it is deleted and recreated to ensure a clean state.
- **File Creation**: Documentation is saved to `docs/pure_pursuit/pr-{PR_NUMBER}-changes.md`
- **Idempotent Updates**: If the documentation file already exists, it will be updated rather than failing

## For New Team Members

### What This Means for You

- **No manual documentation required** for pure pursuit changes - the system handles initial drafts automatically
- **Always review generated docs** - AI-generated content should be verified for accuracy before merging
- **Documentation PRs link back** to the original code PR for context

### Reviewing Auto-Generated Documentation

1. Navigate to the `ft_docs_2.0` repository
2. Find the PR titled `[Auto Docs] Update for ft_pure_pursuit PR #XXX`
3. Review the suggested changes for technical accuracy
4. Edit if necessary, then merge

## Dependencies

- `anthropic` - Python client for Claude AI
- `PyGithub` - Python client for GitHub API