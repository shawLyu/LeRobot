"""Install the pinned Linux x86_64 uv binary inside this project."""

import argparse
import hashlib
import platform
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VERSION = "0.12.23"
SHA256 = "565c6e2874dbeae86c02f3dea97255e878fec672659a73d4930c6b93fcab2fff"
URL = (
    "https://files.pythonhosted.org/packages/e5/83/"
    "85a6c63c24905af4924fddb11a499b934913f59a134248367a1ef1a4716f/"
    "uv-0.12.23-py3-none-manylinux_2_17_x86_64.manylinux2014_x86_64.whl"
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wheel", type=Path, help="Previously downloaded Linux uv wheel")
    args = parser.parse_args()
    if platform.system() != "Linux" or platform.machine() != "x86_64":
        parser.error("Run this installer on the Linux x86_64 SSH host.")
    if (ROOT / ".uv-version").read_text().strip() != VERSION:
        parser.error("Update the installer URL/hash together with .uv-version.")
    wheel = args.wheel
    if wheel is None:
        wheel = ROOT / ".cache/downloads" / URL.rsplit("/", 1)[1]
        wheel.parent.mkdir(parents=True, exist_ok=True)
        if not wheel.exists():
            with urllib.request.urlopen(URL, timeout=60) as response:
                wheel.write_bytes(response.read())
    if hashlib.sha256(wheel.read_bytes()).hexdigest() != SHA256:
        raise SystemExit("uv wheel SHA256 mismatch; installation aborted.")
    destination = ROOT / ".tools/bin"
    destination.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(wheel) as archive:
        for binary in ("uv", "uvx"):
            member = next(
                n for n in archive.namelist() if ".data/scripts/" in n and n.endswith("/" + binary)
            )
            path = destination / binary
            path.write_bytes(archive.read(member))
            path.chmod(0o755)
    print(f"Installed uv {VERSION} to {destination}")


if __name__ == "__main__":
    main()
