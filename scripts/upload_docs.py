#!/usr/bin/env python3
"""
One-time initial upload of all docs/*.md files to the OpenAI vector store.
Run this after create_assistant.py.

Usage:
    OPENAI_API_KEY=sk-... VECTOR_STORE_ID=vs-... python scripts/upload_docs.py
"""
import json
import os
from pathlib import Path
from openai import OpenAI

# #region agent log
def _debug_log(message: str, data: dict, hypothesis_id: str = "") -> None:
    with open("/Users/emmamurphy/Documents/ft/ft_docs_2.0/.claude/worktrees/intelligent-austin/.cursor/debug-a77455.log", "a") as log:
        log.write(json.dumps({"sessionId": "a77455", "location": "upload_docs.py", "message": message, "data": data, "hypothesisId": hypothesis_id}) + "\n")
# #endregion

client = OpenAI()
vector_store_id = os.environ["VECTOR_STORE_ID"]
docs_dir = Path(__file__).parent.parent / "docs"

md_files = sorted(docs_dir.rglob("*.md"))
# #region agent log
sizes = [(str(f), f.stat().st_size) for f in md_files]
empty_files = [p for p, s in sizes if s == 0]
_debug_log("md_files_with_sizes", {"count": len(md_files), "sizes": sizes[:5], "empty_count": len(empty_files), "empty_paths": empty_files}, "H1")
_debug_log("empty_files_list", {"empty_paths": empty_files}, "H4")
# #endregion
# Skip empty files; API rejects them with "File is empty"
md_files_to_upload = [f for f in md_files if f.stat().st_size > 0]
# #region agent log
_debug_log("after_filter", {"upload_count": len(md_files_to_upload), "skipped": len(md_files) - len(md_files_to_upload), "runId": "post-fix"}, "H5")
# #endregion
print(f"Found {len(md_files)} markdown files ({len(md_files_to_upload)} non-empty). Uploading...")

file_streams = [open(f, "rb") for f in md_files_to_upload]
# #region agent log
_debug_log("before_upload", {"stream_count": len(file_streams), "first_stream_pos": file_streams[0].tell() if file_streams else None}, "H2")
# #endregion

try:
    batch = client.vector_stores.file_batches.upload_and_poll(
        vector_store_id=vector_store_id,
        files=file_streams,
    )
    print(f"\nUpload complete.")
    print(f"Status: {batch.status}")
    print(f"Files: {batch.file_counts}")
finally:
    for f in file_streams:
        f.close()
