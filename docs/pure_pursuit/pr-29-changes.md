# Automated Documentation Agent

## Overview

This PR introduces an **automatic documentation generation system** that uses Claude AI to create documentation updates whenever changes are made to the pure pursuit controller codebase.

## What Changed

A new GitHub Actions workflow and Python script have been added that automatically:

1. **Detect code changes** when a PR is opened or updated
2. **Generate documentation** using Claude (Anthropic's AI) based on the code diff
3. **Create a PR in the docs repository** (`FT-Autonomous/ft_docs_2.0`) with the suggested documentation updates

## How It Works

### Workflow Trigger

The automation runs on the `pull_request` event with the following triggers:
- `opened` — When a new PR is created
- `synchronize` — When new commits are pushed to an existing PR

### Process Flow

```
PR Created/Updated → GitHub Actions → Doc Agent Script → Claude AI → Docs PR Created
```

## New Files

### `.github/workflows/doc-agent.yml`

The GitHub Actions workflow configuration that:
- Runs on Ubuntu latest
- Sets up Python 3.11
- Installs required dependencies (`anthropic`, `PyGithub`)
- Executes the doc agent script with necessary secrets

### `.github/scripts/doc_agent.py`

The main documentation generation script containing the following logic:

| Function | Description |
|----------|-------------|
| Diff extraction | Fetches all file changes from the triggering PR |
| AI prompt construction | Builds a context-aware prompt for Claude |
| Branch management | Creates (or recreates) a branch in the docs repo |
| File creation | Commits generated markdown to `docs/pure_pursuit/pr-{PR_NUMBER}-changes.md` |
| PR creation | Opens a pull request in the docs repository |

## Required Secrets

The following secrets must be configured in the repository settings:

| Secret | Purpose |
|--------|---------|
| `ANTHROPIC_API_KEY` | API key for Claude AI access |
| `GH_PAT` | GitHub Personal Access Token with repo access to both source and docs repositories |

## Environment Variables

| Variable | Source | Description |
|----------|--------|-------------|
| `PR_NUMBER` | `github.event.pull_request.number` | The triggering PR number |
| `REPO_NAME` | `github.repository` | The source repository (e.g., `FT-Autonomous/ft_pure_pursuit`) |

## Output

Documentation PRs are created in `FT-Autonomous/ft_docs_2.0` with:
- **Branch name**: `auto-docs/pr-{PR_NUMBER}`
- **File path**: `docs/pure_pursuit/pr-{PR_NUMBER}-changes.md`
- **PR title**: `[Auto Docs] Update for ft_pure_pursuit PR #{PR_NUMBER}: {original_title}`

## For New Team Members

This system means you don't need to manually write documentation for every code change. When you open a PR:

1. The doc agent automatically runs
2. Check the `ft_docs_2.0` repo for a new PR
3. Review the AI-generated documentation
4. Merge if accurate, or edit as needed

> **Note:** Always review auto-generated documentation before merging — AI suggestions should be verified for technical accuracy.