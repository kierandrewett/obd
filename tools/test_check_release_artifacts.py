"""Tests for the pre-publication release artifact gate."""

from pathlib import Path
import sys
import tarfile
import tempfile
import unittest
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_release_artifacts import ArtifactError, scan_directory, scan_payload


class ReleaseArtifactTests(unittest.TestCase):
    def test_plain_binary_is_accepted(self):
        scan_payload("dashboard", b"ELF\x00ordinary compiled application")

    def test_private_key_is_rejected(self):
        with self.assertRaisesRegex(ArtifactError, "credential-like data"):
            scan_payload("dashboard", b"-----BEGIN OPENSSH PRIVATE KEY-----")

    def test_token_assignment_is_rejected_without_echoing_value(self):
        with self.assertRaises(ArtifactError) as raised:
            scan_payload("dashboard", b"api_token=secret-value-which-must-not-echo")
        self.assertNotIn("secret-value", str(raised.exception))

    def test_credential_in_zip_member_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            archive_path = Path(temporary) / "dashboard.zip"
            with zipfile.ZipFile(archive_path, "w") as archive:
                archive.writestr("bin/dashboard.exe", b"ordinary")
                archive.writestr("config/.env", b"not allowed")
            with self.assertRaisesRegex(ArtifactError, "private configuration"):
                scan_directory(Path(temporary))

    def test_credential_bytes_in_zip_member_are_rejected(self):
        import io

        output = io.BytesIO()
        with zipfile.ZipFile(output, "w") as archive:
            archive.writestr("dashboard", b"github_pat_" + b"A" * 30)
        with self.assertRaisesRegex(ArtifactError, "credential-like data"):
            scan_payload("bundle.zip", output.getvalue())

    def test_private_config_in_tar_member_is_rejected(self):
        import io

        output = io.BytesIO()
        with tarfile.open(fileobj=output, mode="w:gz") as archive:
            contents = b"restricted"
            info = tarfile.TarInfo("private/local_config.h")
            info.size = len(contents)
            archive.addfile(info, io.BytesIO(contents))
        with self.assertRaisesRegex(ArtifactError, "private configuration"):
            scan_payload("bundle.tar.gz", output.getvalue())


if __name__ == "__main__":
    unittest.main()
