# Automated Documentation Agent

## Overview

This update introduces an **automated documentation system** that generates documentation updates whenever changes are made to the `ft_pure_pursuit` repository. The system uses Claude AI to analyze code changes and automatically creates pull requests in the documentation repository.

## How It Works

When a pull request is opened or updated in `ft_pure_pursuit`, the following process occurs automatically:

1. **Trigger**: GitHub Actions detects the PR event (`opened` or `synchronize`)
2. **Diff Extraction**: The agent fetches all changed files and their diffs from the PR
3. **AI Analysis**: Claude (claude-opus-4-5) analyzes the changes and generates documentation
4. **PR Creation**: A new branch and pull request are created in `ft_docs_2.0` with the suggested documentation

## Components

### GitHub Actions Workflow (`.github/workflows/doc-agent.yml`)

| Setting | Value | Description |
|---------|-------|-------------|
| Trigger Events | `opened`, `synchronize` | Runs on new PRs and when existing PRs receive new commits |
| Runner | `ubuntu-latest` | GitHub-hosted Ubuntu environment |
| Python Version | `3.11` | Required for running the doc agent script |

### Doc Agent Script (`.github/scripts/doc_agent.py`)

The Python script that orchestrates the documentation generation process.

#### Required Environment Variables

| Variable | Source | Description |
|----------|--------|-------------|
| `ANTHROPIC_API_KEY` | Repository Secret | API key for Claude AI |
| `GH_PAT` | Repository Secret | GitHub Personal Access Token with repo permissions |
| `PR_NUMBER` | Workflow Context | The triggering PR number |
| `REPO_NAME` | Workflow Context | The source repository (e.g., `FT-Autonomous/ft_pure_pursuit`) |

#### Key Behaviours

- **Branch Management**: Creates a new branch named `auto-docs/pr-{PR_NUMBER}` in the docs repository. If the branch already exists, it is deleted and recreated to ensure a clean state.

- **File Naming**: Documentation is saved to `docs/pure_pursuit/pr-{PR_NUMBER}-changes.md`

- **Idempotent Updates**: If the documentation file already exists, the script updates it rather than failing

- **PR Linking**: The generated PR body includes links back to the original source PR for traceability

## For New Team Members

### What You Need to Know

1. **You don't need to run this manually** — it triggers automatically on every PR to `ft_pure_pursuit`

2. **Review is still required** — the generated documentation PRs should be reviewed before merging

3. **The docs appear in `ft_docs_2.0`** — check that repository for pending documentation PRs

### Required Secrets Setup

Ensure the following secrets are configured in the repository settings:

```
ANTHROPIC_API_KEY  →  Your team's Anthropic API key
GH_PAT             →  A GitHub PAT with write access to both repositories
```

## Limitations

- Only processes changes to `ft_pure_pursuit` (target docs repo is hardcoded)
- Maximum response length is 1024 tokens
- Requires manual review and merge of generated documentation