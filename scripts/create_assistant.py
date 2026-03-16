#!/usr/bin/env python3
"""
One-time setup script. Run this locally to create the OpenAI Assistant and
vector store. Record the two IDs it prints and add them as secrets:
  - Vercel: OPENAI_API_KEY, ASSISTANT_ID, VECTOR_STORE_ID
  - GitHub repo: OPENAI_API_KEY, VECTOR_STORE_ID

Usage:
    OPENAI_API_KEY=sk-... python scripts/create_assistant.py
"""
from openai import OpenAI

client = OpenAI()

vs = client.vector_stores.create(name="ft-docs")

assistant = client.beta.assistants.create(
    name="Formula Trinity Docs Bot",
    instructions="""You are a technical assistant for the Formula Trinity autonomous racing team.

First, search the provided documentation to answer questions. If the answer is in the docs, cite the relevant section name.

For general technical questions about external tools the team uses (ROS2, Python, C++, Git, Linux, etc.) that are not covered in the docs, you may draw on your own training knowledge — but make clear that the answer is from general knowledge, not FT-specific docs.

Be concise and technical. If you genuinely don't know, say so clearly.""",
    model="gpt-4o-mini",
    tools=[{"type": "file_search"}],
    tool_resources={"file_search": {"vector_store_ids": [vs.id]}},
)

print("\nSetup complete. Add these as secrets:\n")
print(f"VECTOR_STORE_ID={vs.id}")
print(f"ASSISTANT_ID={assistant.id}")
print("\nNext: run `python scripts/upload_docs.py` to index the documentation.")
