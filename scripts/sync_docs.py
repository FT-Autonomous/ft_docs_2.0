#!/usr/bin/env python3
"""
CI script that fully re-syncs all docs/*.md files to the OpenAI vector store.
Runs on every push to main that modifies docs/** via sync-docs.yml.

Strategy: delete all existing files from the vector store, then re-upload
everything. At ~74 files this is fast (~10s) and avoids stale-file bugs
from incremental diffing.

Usage (GitHub Actions sets these env vars automatically):
    OPENAI_API_KEY=sk-... VECTOR_STORE_ID=vs-... python scripts/sync_docs.py
"""
import os
from pathlib import Path
from openai import OpenAI

client = OpenAI()
vector_store_id = os.environ["VECTOR_STORE_ID"]

# --- Remove all existing files from the vector store ---
print("Removing existing files from vector store...")
removed = 0
while True:
    existing = client.beta.vector_stores.files.list(vector_store_id=vector_store_id)
    if not existing.data:
        break
    for vf in existing.data:
        client.beta.vector_stores.files.delete(
            vector_store_id=vector_store_id,
            file_id=vf.id,
        )
        client.files.delete(vf.id)
        removed += 1
print(f"Removed {removed} file(s).")

# --- Upload current docs ---
docs_dir = Path(__file__).parent.parent / "docs"
md_files = sorted(docs_dir.rglob("*.md"))
print(f"Uploading {len(md_files)} markdown files...")

file_streams = [open(f, "rb") for f in md_files]

try:
    batch = client.beta.vector_stores.file_batches.upload_and_poll(
        vector_store_id=vector_store_id,
        files=file_streams,
    )
    print(f"Sync complete. Status: {batch.status}")
    print(f"Files: {batch.file_counts}")
    if batch.status != "completed":
        raise SystemExit(f"Sync did not complete successfully: {batch.status}")
finally:
    for f in file_streams:
        f.close()
