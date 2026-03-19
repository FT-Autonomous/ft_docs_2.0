# Automated Documentation Agent

## Overview

This PR introduces an **automated documentation generation system** that uses AI to create documentation updates whenever code changes are made to the pure pursuit controller. This reduces manual documentation burden and helps keep docs in sync with code changes.

## How It Works

When a pull request is opened or updated in the `ft_pure_pursuit` repository, a GitHub Actions workflow automatically:

1. **Fetches the code diff** from the pull request
2. **Sends the diff to Claude AI** (Anthropic's claude-opus-4-5 model) with a documentation prompt
3. **Creates a new branch** in the `ft_docs_2.0` documentation repository
4. **Commits the generated documentation** as a markdown file
5. **Opens a pull request** in the docs repo for team review

## New Files

### `.github/scripts/doc_agent.py`

The main Python script that orchestrates the documentation generation.

#### Required Environment Variables

| Variable | Description |
|----------|-------------|
| `ANTHROPIC_API_KEY` | API key for Claude AI authentication |
| `GH_PAT` | GitHub Personal Access Token with repo access |
| `PR_NUMBER` | The pull request number that triggered the workflow |
| `REPO_NAME` | The source repository (e.g., `FT-Autonomous/ft_pure_pursuit`) |

#### Key Behaviours

- **Branch Management**: Creates a branch named `auto-docs/pr-{PR_NUMBER}`. If the branch already exists, it deletes and recreates it to ensure a clean state.
- **File Naming**: Generated docs are saved to `docs/pure_pursuit/pr-{PR_NUMBER}-changes.md`
- **Idempotency**: If the doc file already exists, it updates rather than fails

### `.github/workflows/doc-agent.yml`

GitHub Actions workflow configuration.

#### Trigger Events

- `opened` — Runs when a new PR is created
- `synchronize` — Runs when new commits are pushed to an existing PR

#### Dependencies

- Python 3.11
- `anthropic` — Claude AI SDK
- `PyGithub` — GitHub API wrapper

## For New Team Members

### What This Means for You

- **You don't need to manually write documentation** for every code change
- After your PR is opened, check the `ft_docs_2.0` repo for an auto-generated docs PR
- **Review the generated docs** — AI suggestions should be verified for accuracy
- Merge the docs PR once you've confirmed the content is correct

### Secrets Setup

If you're an admin setting this up for a new repo, ensure these secrets are configured in the repository settings:

1. `ANTHROPIC_API_KEY` — Obtain from the team's Anthropic account
2. `GH_PAT` — A personal access token with `repo` scope for both source and docs repositories