#!/usr/bin/env python3
"""Vendor official PoissonRecon 18.76 into third_party/PoissonRecon.

Default source is the official Johns Hopkins versioned archive. urllib honors
HTTP_PROXY / HTTPS_PROXY automatically. For fully controlled/offline use, pass
an already-downloaded archive with --archive.
"""
from __future__ import annotations

import argparse
import io
from pathlib import Path
import shutil
import sys
import tempfile
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "third_party" / "PoissonRecon"
OFFICIAL_URL = "https://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version18.76/AdaptiveSolvers.zip"
EXPECTED_VERSION = "18.76"


def locate_root(extracted: Path) -> Path:
    matches = list(extracted.rglob("Src/Reconstructors.h"))
    if not matches:
        matches = [p for p in extracted.rglob("Reconstructors.h") if p.parent.name == "Src"]
    if len(matches) != 1:
        raise RuntimeError(f"expected exactly one Src/Reconstructors.h, found {len(matches)}")
    return matches[0].parent.parent


def validate(root: Path) -> None:
    required = [
        root / "Src" / "Reconstructors.h",
        root / "Src" / "FEMTree.h",
        root / "Src" / "MultiThreading.h",
        root / "Src" / "PreProcessor.h",
    ]
    missing = [str(p.relative_to(root)) for p in required if not p.is_file()]
    if missing:
        raise RuntimeError("incomplete PoissonRecon source: missing " + ", ".join(missing))
    readme = root / "README.md"
    if readme.is_file():
        text = readme.read_text(encoding="utf-8", errors="ignore")
        if f"Version {EXPECTED_VERSION}" not in text:
            raise RuntimeError(f"archive does not advertise PoissonRecon {EXPECTED_VERSION}")


def download(url: str) -> bytes:
    print(f"Downloading official PoissonRecon {EXPECTED_VERSION}: {url}")
    request = urllib.request.Request(url, headers={"User-Agent": "PointCloudEditor-vendor/2.2.0"})
    with urllib.request.urlopen(request, timeout=120) as response:
        return response.read()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--archive", type=Path, help="Use an already downloaded AdaptiveSolvers.zip")
    parser.add_argument("--url", default=OFFICIAL_URL, help="Override download URL (development only)")
    parser.add_argument("--force", action="store_true", help="Replace an existing vendor tree")
    args = parser.parse_args()

    if (DEST / "Src" / "Reconstructors.h").is_file() and not args.force:
        validate(DEST)
        print(f"PoissonRecon {EXPECTED_VERSION} is already vendored: {DEST}")
        return 0

    if args.archive:
        data = args.archive.expanduser().resolve().read_bytes()
        print(f"Using local archive: {args.archive}")
    else:
        data = download(args.url)

    with tempfile.TemporaryDirectory(prefix="pceditor-poisson-") as td:
        temp = Path(td)
        with zipfile.ZipFile(io.BytesIO(data)) as zf:
            bad = zf.testzip()
            if bad:
                raise RuntimeError(f"corrupt ZIP member: {bad}")
            zf.extractall(temp)
        source_root = locate_root(temp)
        validate(source_root)

        if DEST.exists():
            shutil.rmtree(DEST)
        shutil.copytree(source_root, DEST)

    validate(DEST)
    marker = DEST / "PCEDITOR_VENDOR_VERSION.txt"
    marker.write_text(
        "PoissonRecon 18.76\n"
        f"Source: {args.url if not args.archive else args.archive}\n"
        "Vendored by tools/vendor_poissonrecon.py\n",
        encoding="utf-8",
    )
    print(f"Vendored PoissonRecon {EXPECTED_VERSION} -> {DEST}")
    print("Next: delete/reconfigure your CMake build directory, then Configure/Generate again.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
