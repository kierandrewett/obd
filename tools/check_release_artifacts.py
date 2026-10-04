#!/usr/bin/env python3
"""Fail a release if packaged dashboard artifacts contain credential material."""

from __future__ import annotations

import argparse
import io
from pathlib import Path
import re
import sys
import tarfile
import zipfile


FORBIDDEN_NAME = re.compile(
    r"(?:^|[/\\])(?:\.env(?:\..*)?|local_config\.h|.*(?:credential|secret|private[-_]?key).*)$",
    re.IGNORECASE,
)
SECRET_PATTERNS = (
    re.compile(rb"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"),
    re.compile(rb"\b(?:AKIA|ASIA)[A-Z0-9]{16}\b"),
    re.compile(rb"\bgh[pousr]_[A-Za-z0-9_]{20,}\b"),
    re.compile(rb"\bgithub_pat_[A-Za-z0-9_]{20,}\b"),
    re.compile(rb"\bxox[baprs]-[A-Za-z0-9-]{20,}\b"),
    re.compile(
        rb"(?i)\b(?:access[_-]?token|api[_-]?(?:key|token)|password|passwd|secret|"
        rb"freematics[_-]?token)\b\s*[:=]\s*[\"']?"
        rb"([A-Za-z0-9_./+=:-]{16,})"
    ),
)
MAX_MEMBER_BYTES = 512 * 1024 * 1024


class ArtifactError(ValueError):
    pass


def _scan_bytes(label: str, payload: bytes) -> None:
    for pattern in SECRET_PATTERNS:
        if pattern.search(payload):
            raise ArtifactError(f"credential-like data detected in {label}")


def _check_name(name: str) -> None:
    if FORBIDDEN_NAME.search(name):
        raise ArtifactError(f"private configuration or credential file found: {name}")


def _scan_tar(payload: bytes, label: str, depth: int) -> None:
    try:
        with tarfile.open(fileobj=io.BytesIO(payload), mode="r:*") as archive:
            for member in archive:
                _check_name(member.name)
                if not member.isfile():
                    continue
                if member.size > MAX_MEMBER_BYTES:
                    raise ArtifactError(f"oversized archive member: {label}/{member.name}")
                stream = archive.extractfile(member)
                if stream is not None:
                    scan_payload(f"{label}/{member.name}", stream.read(), depth + 1)
    except tarfile.TarError:
        return


def _scan_zip(payload: bytes, label: str, depth: int) -> None:
    try:
        with zipfile.ZipFile(io.BytesIO(payload)) as archive:
            for member in archive.infolist():
                _check_name(member.filename)
                if member.is_dir():
                    continue
                if member.file_size > MAX_MEMBER_BYTES:
                    raise ArtifactError(f"oversized archive member: {label}/{member.filename}")
                scan_payload(f"{label}/{member.filename}", archive.read(member), depth + 1)
    except zipfile.BadZipFile:
        return


def scan_payload(label: str, payload: bytes, depth: int = 0) -> None:
    if depth > 4:
        raise ArtifactError(f"archive nesting limit exceeded: {label}")
    _scan_bytes(label, payload)
    if payload.startswith(b"PK\x03\x04"):
        _scan_zip(payload, label, depth)
    # The tar magic lives inside a 512-byte header, not at byte zero; opening
    # in auto-detect mode also covers gzip/xz-compressed tar assets.
    _scan_tar(payload, label, depth)
    # Debian packages are ar containers whose data.tar.* member is a tarball.
    if payload.startswith(b"!<arch>\n"):
        offset = 8
        while offset + 60 <= len(payload):
            header = payload[offset:offset + 60]
            if header[58:60] != b"`\n":
                raise ArtifactError(f"malformed ar archive: {label}")
            try:
                size = int(header[48:58].strip())
            except ValueError as error:
                raise ArtifactError(f"malformed ar member size: {label}") from error
            start = offset + 60
            end = start + size
            if end > len(payload):
                raise ArtifactError(f"truncated ar member: {label}")
            member_name = header[:16].decode("ascii", errors="replace").strip().rstrip("/")
            _check_name(member_name)
            scan_payload(f"{label}/{member_name}", payload[start:end], depth + 1)
            offset = end + (size & 1)
    # RPM and several installer formats embed long strings in opaque binary
    # containers. Their raw bytes are covered above even where extraction is
    # not available in the Python standard library.


def scan_directory(directory: Path) -> int:
    if not directory.is_dir():
        raise ArtifactError("artifact directory does not exist")
    files = sorted(path for path in directory.rglob("*") if path.is_file())
    if not files:
        raise ArtifactError("artifact directory is empty")
    for path in files:
        relative = path.relative_to(directory).as_posix()
        _check_name(relative)
        if path.stat().st_size > MAX_MEMBER_BYTES:
            raise ArtifactError(f"oversized release asset: {relative}")
        scan_payload(relative, path.read_bytes())
    return len(files)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifact_directory", type=Path)
    args = parser.parse_args(argv)
    try:
        count = scan_directory(args.artifact_directory)
    except (ArtifactError, OSError) as error:
        print(f"release artifact check failed: {error}", file=sys.stderr)
        return 1
    print(f"Release artifact check passed ({count} assets; no detected credentials or private config files).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
