from __future__ import annotations

import gzip
import io
import tarfile
from pathlib import Path

import build_backend


def _write_nondeterministic_sdist(path: Path, *, timestamp: int) -> None:
    with path.open("wb") as output:
        with gzip.GzipFile(filename=path.name, mode="wb", fileobj=output, mtime=timestamp) as compressed:
            with tarfile.open(fileobj=compressed, mode="w|", format=tarfile.PAX_FORMAT) as archive:
                root = tarfile.TarInfo("unirobosim-0.9.2")
                root.type = tarfile.DIRTYPE
                root.mode = 0o755
                root.mtime = timestamp
                root.uid = 1000
                root.gid = 1000
                root.uname = "builder"
                root.gname = "builder"
                archive.addfile(root)
                payload = b"portable source payload\n"
                member = tarfile.TarInfo("unirobosim-0.9.2/example.txt")
                member.mode = 0o644
                member.mtime = timestamp + 1
                member.uid = 1000
                member.gid = 1000
                member.uname = "builder"
                member.gname = "builder"
                member.size = len(payload)
                archive.addfile(member, io.BytesIO(payload))


def test_sdist_rewriter_normalizes_gzip_and_tar_metadata(tmp_path: Path) -> None:
    first = tmp_path / "first.tar.gz"
    second = tmp_path / "second.tar.gz"
    _write_nondeterministic_sdist(first, timestamp=100)
    _write_nondeterministic_sdist(second, timestamp=200)
    assert first.read_bytes() != second.read_bytes()

    epoch = 1_787_376_579
    build_backend._rewrite_sdist(first, epoch)
    build_backend._rewrite_sdist(second, epoch)
    assert first.read_bytes() == second.read_bytes()
    with tarfile.open(first, "r:gz") as archive:
        members = archive.getmembers()
        assert all(member.mtime == epoch for member in members)
        assert all((member.uid, member.gid, member.uname, member.gname) == (0, 0, "", "") for member in members)


def test_source_distribution_manifest_includes_complete_test_support() -> None:
    project_root = Path(__file__).resolve().parents[1]
    manifest = (project_root / "MANIFEST.in").read_text(encoding="utf-8")
    assert "include build_backend.py" in manifest
    assert "recursive-include tests *.py" in manifest
    assert (project_root / "tests" / "__init__.py").is_file()
    assert (project_root / "tests" / "helpers.py").is_file()
