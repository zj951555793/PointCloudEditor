#!/usr/bin/env python3
"""按工程根目录 .clang-format 格式化全部 C/C++ 文件。

优先调用 clang-format；若环境只有 clangd（例如部分 LLVM/Swift 工具链），
则通过 clangd 的 textDocument/formatting 使用同一 clang-format 引擎。
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Dict, List, Optional
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SUFFIXES = {".h", ".hh", ".hpp", ".cpp", ".cc", ".cxx"}


def source_files() -> List[Path]:
    return sorted(
        path
        for path in ROOT.rglob("*")
        if path.is_file()
        and path.suffix.lower() in SUFFIXES
        and not any(part.startswith("build") or part == "third_party" for part in path.relative_to(ROOT).parts)
    )


def find_clang_format() -> Optional[str]:
    names = ["clang-format"] + [f"clang-format-{version}" for version in range(30, 9, -1)]
    for name in names:
        executable = shutil.which(name)
        if executable:
            return executable
    return None


def format_with_clang_format(executable: str, files: List[Path]) -> None:
    for path in files:
        subprocess.run([executable, "-i", "-style=file", str(path)], cwd=ROOT, check=True)


class ClangdClient:
    def __init__(self, executable: str) -> None:
        self.process = subprocess.Popen(
            [executable, "--log=error", "--enable-config=false"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.next_id = 1

    def send(self, payload: dict) -> None:
        assert self.process.stdin is not None
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.process.stdin.write(f"Content-Length: {len(data)}\r\n\r\n".encode("ascii"))
        self.process.stdin.write(data)
        self.process.stdin.flush()

    def receive(self) -> dict:
        assert self.process.stdout is not None
        headers: Dict[str, str] = {}
        while True:
            line = self.process.stdout.readline()
            if not line:
                raise RuntimeError("clangd unexpectedly exited")
            if line in (b"\r\n", b"\n"):
                break
            key, value = line.decode("ascii", "replace").split(":", 1)
            headers[key.lower()] = value.strip()
        size = int(headers["content-length"])
        return json.loads(self.process.stdout.read(size).decode("utf-8"))

    def request(self, method: str, params: Optional[dict]) -> dict:
        request_id = self.next_id
        self.next_id += 1
        self.send({"jsonrpc": "2.0", "id": request_id, "method": method, "params": params})
        while True:
            message = self.receive()
            if message.get("id") == request_id:
                return message

    def notify(self, method: str, params: Optional[dict]) -> None:
        self.send({"jsonrpc": "2.0", "method": method, "params": params})

    def close(self) -> None:
        self.request("shutdown", None)
        self.notify("exit", None)
        assert self.process.stdin is not None
        self.process.stdin.close()
        self.process.wait(timeout=30)


def utf16_column_to_index(line: str, column: int) -> int:
    units = 0
    for index, character in enumerate(line):
        units += 2 if ord(character) > 0xFFFF else 1
        if units >= column:
            return index + 1
    return len(line)


def position_to_offset(text: str, position: dict) -> int:
    lines = text.splitlines(keepends=True)
    line_number = position["line"]
    if line_number >= len(lines):
        return len(text)
    offset = sum(len(line) for line in lines[:line_number])
    line = lines[line_number].rstrip("\r\n")
    return offset + utf16_column_to_index(line, position["character"])


def apply_edits(text: str, edits: List[dict]) -> str:
    spans = []
    for edit in edits:
        begin = position_to_offset(text, edit["range"]["start"])
        end = position_to_offset(text, edit["range"]["end"])
        spans.append((begin, end, edit["newText"]))
    for begin, end, replacement in sorted(spans, reverse=True):
        text = text[:begin] + replacement + text[end:]
    return text


def format_with_clangd(executable: str, files: List[Path]) -> None:
    client = ClangdClient(executable)
    client.request(
        "initialize",
        {
            "processId": os.getpid(),
            "rootUri": ROOT.as_uri(),
            "capabilities": {"general": {"positionEncodings": ["utf-16"]}},
        },
    )
    client.notify("initialized", {})
    try:
        for path in files:
            text = path.read_text(encoding="utf-8")
            uri = path.as_uri()
            client.notify(
                "textDocument/didOpen",
                {"textDocument": {"uri": uri, "languageId": "cpp", "version": 1, "text": text}},
            )
            response = client.request(
                "textDocument/formatting",
                {
                    "textDocument": {"uri": uri},
                    "options": {"tabSize": 4, "insertSpaces": True, "insertFinalNewline": True},
                },
            )
            if "error" in response:
                raise RuntimeError(f"{path}: {response['error']}")
            edits = response.get("result") or []
            if edits:
                path.write_text(apply_edits(text, edits), encoding="utf-8")
            client.notify("textDocument/didClose", {"textDocument": {"uri": uri}})
    finally:
        client.close()


def main() -> int:
    files = source_files()
    clang_format = find_clang_format()
    if clang_format:
        print(f"Using {clang_format}; files={len(files)}")
        format_with_clang_format(clang_format, files)
        return 0

    clangd = shutil.which("clangd")
    if clangd:
        print(f"clang-format not found; using clangd formatter: {clangd}; files={len(files)}")
        format_with_clangd(clangd, files)
        return 0

    print("error: neither clang-format nor clangd was found", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
