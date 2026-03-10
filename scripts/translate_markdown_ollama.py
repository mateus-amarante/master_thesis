#!/usr/bin/env python3
import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

PLACEHOLDER_FMT = "@@PH_{:06d}@@"

MATH_PATTERNS = [
    re.compile(r"\$\$(.+?)\$\$", re.DOTALL),
    re.compile(r"(?<!\$)\$(?!\$)(.+?)(?<!\$)\$(?!\$)", re.DOTALL),
]

REF_PATTERNS = [
    re.compile(r"\b(Eq\.?|Equa(?:c|ç)(?:a|ã)o)\s*\(?\d+(?:\.\d+)*\)?", re.IGNORECASE),
    re.compile(r"\b(Figura|Figure|Tabela|Table|Quadro|Ap[eê]ndice|Appendix)\s+\d+(?:\.\d+)*(?:-[a-z])?(?:\s*\([a-z]\))?", re.IGNORECASE),
    re.compile(r"\b\(\d+\.\d+\)"),
]

NON_TRANSLATABLE_LINE_PATTERNS = [
    re.compile(r"^\s*<img\b", re.IGNORECASE),
    re.compile(r"^\s*<table>\s*$", re.IGNORECASE),
    re.compile(r"^\s*</?t(?:head|body|r|h|d|able)\b", re.IGNORECASE),
    re.compile(r"^\s*\|[- :|]+\|\s*$"),
]


def should_translate_block(block: str) -> bool:
    stripped = block.strip()
    if not stripped:
        return False
    lines = stripped.splitlines()
    if all(any(p.search(line) for p in NON_TRANSLATABLE_LINE_PATTERNS) for line in lines):
        return False
    # Skip markdown-only blocks like separators/lists with no alphabetic content.
    return bool(re.search(r"[A-Za-zÀ-ÿ]", stripped))


def protect(text: str):
    items = []

    def replacer(match):
        token = PLACEHOLDER_FMT.format(len(items))
        items.append(match.group(0))
        return token

    for pat in MATH_PATTERNS:
        text = pat.sub(replacer, text)
    for pat in REF_PATTERNS:
        text = pat.sub(replacer, text)
    return text, items


def restore(text: str, items):
    for i, original in enumerate(items):
        text = text.replace(PLACEHOLDER_FMT.format(i), original)
    return text


def translate_with_ollama(model: str, content: str, timeout_s: int) -> str:
    prompt = (
        "Translate the following Markdown content from Brazilian Portuguese (pt-BR) "
        "to American English (en-US).\\n"
        "Rules:\\n"
        "1) Preserve Markdown structure exactly.\\n"
        "2) Keep placeholder tokens unchanged (format @@PH_xxxxxx@@).\\n"
        "3) Do not add, remove, summarize, or reorder information.\\n"
        "4) Keep citations and names unchanged unless language translation is required.\\n"
        "5) Preserve every placeholder token exactly once.\\n"
        "6) Preserve every dollar-delimited math expression exactly.\\n"
        "7) Output only the translated Markdown.\\n\\n"
        "CONTENT:\\n"
        f"{content}"
    )

    proc = subprocess.run(
        ["ollama", "run", model],
        input=prompt,
        text=True,
        capture_output=True,
        timeout=timeout_s,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"ollama failed: {proc.stderr.strip()}")
    out = proc.stdout.strip()
    if not out:
        raise RuntimeError("ollama returned empty output")
    return out


def placeholder_count(text: str) -> int:
    return len(re.findall(r"@@PH_\d{6}@@", text))


def read_state(path: Path):
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def write_state(path: Path, state):
    path.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--model", default="translategemma:latest")
    ap.add_argument("--state", default="translation_state.json")
    ap.add_argument("--max-blocks", type=int, default=0, help="0 means all")
    ap.add_argument("--timeout", type=int, default=240)
    args = ap.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)
    state_path = Path(args.state)

    text = input_path.read_text(encoding="utf-8")
    blocks = text.split("\n\n")

    state = read_state(state_path)
    done = state.get("done", {})

    translated_blocks = []
    translated_count = 0

    start = time.time()
    for idx, block in enumerate(blocks):
        key = str(idx)
        if key in done:
            translated_blocks.append(done[key])
            continue

        if args.max_blocks and translated_count >= args.max_blocks:
            translated_blocks.append(block)
            continue

        if not should_translate_block(block):
            done[key] = block
            translated_blocks.append(block)
            continue

        protected, placeholders = protect(block)
        try:
            translated = translate_with_ollama(args.model, protected, args.timeout)
            expected_placeholders = len(placeholders)
            actual_placeholders = placeholder_count(translated)
            if actual_placeholders != expected_placeholders:
                raise RuntimeError(
                    f"placeholder mismatch: expected {expected_placeholders}, got {actual_placeholders}"
                )
            translated = restore(translated, placeholders)
        except Exception as exc:
            print(f"[WARN] block {idx} translation failed, keeping original: {exc}", file=sys.stderr)
            translated = block

        done[key] = translated
        translated_blocks.append(translated)
        translated_count += 1

        if translated_count % 10 == 0:
            elapsed = time.time() - start
            print(f"[INFO] translated {translated_count} new blocks in {elapsed:.1f}s", file=sys.stderr)
            write_state(state_path, {"done": done})

    write_state(state_path, {"done": done})
    output_path.write_text("\n\n".join(translated_blocks), encoding="utf-8")
    print(f"[OK] wrote {output_path} with {len(blocks)} blocks")


if __name__ == "__main__":
    main()
