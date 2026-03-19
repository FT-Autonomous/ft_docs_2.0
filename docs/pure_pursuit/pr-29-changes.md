# Automated Documentation Agent

## Overview

A new automated documentation system has been added to the `ft_pure_pursuit` repository. This system automatically generates and proposes documentation updates whenever code changes are made via pull requests.

## How It Works

When a developer opens or updates a pull request in the `ft_pure_pursuit` repository, the following automated process is triggered:

1. **Diff Extraction** – The agent fetches all changed files and their patches from the PR
2. **AI Analysis** – The code changes are sent to Claude (Anthropic's AI) for analysis
3. **Documentation Generation** – Claude generates markdown documentation explaining the changes
4. **PR Creation** – A new pull request is automatically created in the `ft_docs_2.0` repository with the suggested documentation

## Components

### GitHub Actions Workflow (`.github/workflows/doc-agent.yml`)

The workflow triggers on:
- `opened` – When a new PR is created
- `synchronize` – When new commits are pushed to an existing PR

### Doc Agent Script (`.github/scripts/doc_agent.py`)

The Python script that orchestrates the documentation generation process.

#### Required Environment Variables

| Variable | Description |
|----------|-------------|
| `ANTHROPIC_API_KEY` | API key for Claude AI access |
| `GH_PAT` | GitHub Personal Access Token with repo permissions |
| `PR_NUMBER` | The pull request number (auto-populated by GitHub Actions) |
| `REPO_NAME` | The repository name (auto-populated by GitHub Actions) |

#### Key Behaviours

- **Branch Management** – Creates a new branch `auto-docs/pr-{PR_NUMBER}` in the docs repository. If the branch already exists, it is deleted and recreated to ensure a clean state.
- **File Handling** – Documentation is saved to `docs/pure_pursuit/pr-{PR_NUMBER}-changes.md`. Existing files are updated rather than duplicated.
- **PR Linking** – The generated docs PR includes a link back to the original code PR for traceability.

## For New Team Members

You don't need to do anything special to use this system. Simply:

1. Create a pull request in `ft_pure_pursuit` as normal
2. Wait for the workflow to complete (check the Actions tab)
3. Review the auto-generated documentation PR in `ft_docs_2.0`
4. Edit the docs PR if needed, then merge

## Dependencies

- `anthropic` – Python client for Claude AI
- `PyGithub` – Python client for GitHub API