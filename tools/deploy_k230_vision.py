"""Safely inspect, deploy, and roll back the K230 steel-ball vision bundle.

The deployment is transactional at the file-bundle level: every file is first
uploaded as ``.new`` and SHA-256 verified. The active files are replaced only
after the complete bundle passes verification, and prior files remain as
``.bak`` for rollback.

Usage:
    python tools/deploy_k230_vision.py inspect --port COM6
    python tools/deploy_k230_vision.py deploy --port COM6
    python tools/deploy_k230_vision.py rollback --port COM6
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial
from serial.tools import list_ports


BAUD = 115200
PROJECT_ROOT = Path(__file__).resolve().parents[1]
CHUNK_BYTES = 1500  # Matches the proven 2000-character base64 REPL chunks.
RAW_PROMPT = b"\x04>"


@dataclass(frozen=True)
class BundleFile:
    local_path: Path | None
    remote_path: str
    content: bytes | None = None

    def read_bytes(self) -> bytes:
        if self.content is not None:
            return self.content
        if self.local_path is None:
            raise RuntimeError("bundle file has no content")
        return self.local_path.read_bytes()


AUTOSTART = (
    b"import sys\n"
    b"sys.path.insert(0, '/sdcard/app')\n"
    b"exec(open('/sdcard/app/k230_yolo.py').read())\n"
)

BUNDLE = (
    BundleFile(
        PROJECT_ROOT / "reference_code" / "laoguigui2" / "yolo11n_det_320.kmodel",
        "/sdcard/kmodel/yolo11n_det_320.kmodel",
    ),
    BundleFile(PROJECT_ROOT / "k230_code" / "libs" / "AIBase.py", "/sdcard/app/libs/AIBase.py"),
    BundleFile(PROJECT_ROOT / "k230_code" / "libs" / "AI2D.py", "/sdcard/app/libs/AI2D.py"),
    BundleFile(PROJECT_ROOT / "k230_code" / "libs" / "PipeLine.py", "/sdcard/app/libs/PipeLine.py"),
    BundleFile(PROJECT_ROOT / "k230_code" / "libs" / "Utils.py", "/sdcard/app/libs/Utils.py"),
    BundleFile(PROJECT_ROOT / "k230_code" / "libs" / "YOLO.py", "/sdcard/app/libs/YOLO.py"),
    BundleFile(PROJECT_ROOT / "k230_code" / "k230_yolo.py", "/sdcard/app/k230_yolo.py"),
    BundleFile(None, "/sdcard/main.py", AUTOSTART),
)


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _resolve_port(requested: str | None) -> str:
    if requested:
        return requested
    ports = [item.device for item in list_ports.comports()]
    if len(ports) == 1:
        return ports[0]
    if not ports:
        raise RuntimeError("no serial port detected; connect the K230 USB serial port")
    raise RuntimeError("multiple serial ports detected; pass --port (found: {})".format(", ".join(ports)))


class RawRepl:
    def __init__(self, port: str):
        self.port = port
        self.ser = self._open_port()

    def _open_port(self) -> serial.Serial:
        return serial.Serial(
            self.port,
            BAUD,
            timeout=0.05,
            write_timeout=2,
            dsrdtr=False,
            rtscts=False,
        )

    def close(self) -> None:
        self.ser.close()

    def _read_until(self, suffix: bytes, timeout: float) -> bytes:
        data = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            waiting = self.ser.in_waiting
            chunk = self.ser.read(waiting if waiting else 1)
            if chunk:
                data.extend(chunk)
                if data.endswith(suffix):
                    return bytes(data)
        raise TimeoutError("timed out waiting for K230 raw REPL; received {!r}".format(bytes(data[-200:])))

    def enter(self) -> None:
        print("Capturing K230 raw REPL...")
        # Do not toggle DTR. This CanMV build re-enumerates USB CDC when DTR
        # changes, invalidating the Windows handle before REPL can be captured.
        time.sleep(0.25)
        self.ser.reset_input_buffer()
        # Drain boot output while interrupting so the CDC endpoints both flow.
        for _ in range(80):
            if self.ser.in_waiting:
                self.ser.read(self.ser.in_waiting)
            self.ser.write(b"\x03")
            time.sleep(0.02)
        time.sleep(0.4)
        self.ser.reset_input_buffer()
        self.ser.write(b"\r\x01")
        response = self._read_until(b">", 3.0)
        if b"raw REPL" not in response:
            self.ser.write(b"\r\x03\x03\x01")
            response += self._read_until(b">", 3.0)
        if b"raw REPL" not in response:
            raise RuntimeError("K230 did not enter raw REPL: {!r}".format(response[-200:]))
        print("Raw REPL ready.")

    def execute(self, code: str, timeout: float = 10.0) -> bytes:
        self.ser.write(code.encode("utf-8"))
        self.ser.write(b"\x04")
        response = self._read_until(RAW_PROMPT, timeout)
        if not response.startswith(b"OK"):
            raise RuntimeError("raw REPL rejected command: {!r}".format(response[-500:]))
        # Raw REPL separates stdout and stderr with EOT bytes.
        parts = response[2:].split(b"\x04")
        stdout = parts[0] if parts else b""
        stderr = parts[1] if len(parts) > 1 else b""
        if stderr.strip():
            raise RuntimeError("K230 command failed: {}".format(stderr.decode("utf-8", "replace")))
        return stdout

    def soft_reset_and_monitor(self, timeout: float = 35.0) -> str:
        self.ser.write(b"\x04")
        deadline = time.monotonic() + timeout
        data = bytearray()
        last_data = time.monotonic()
        while time.monotonic() < deadline:
            waiting = self.ser.in_waiting
            chunk = self.ser.read(waiting if waiting else 1)
            if chunk:
                data.extend(chunk)
                last_data = time.monotonic()
                sys.stdout.write(chunk.decode("utf-8", "replace"))
                sys.stdout.flush()
                if b"[K230] Running: YOLO NPU" in data:
                    break
            elif data and time.monotonic() - last_data > 8.0:
                break
        return data.decode("utf-8", "replace")


def _remote_sha256(repl: RawRepl, path: str, timeout: float = 30.0) -> tuple[int, str]:
    code = (
        "import os,ubinascii\r\n"
        "try:\r\n import hashlib\r\n"
        "except:\r\n import uhashlib as hashlib\r\n"
        "p={!r}\r\n"
        "f=open(p,'rb')\r\n"
        "h=hashlib.sha256()\r\n"
        "n=0\r\n"
        "while True:\r\n"
        " b=f.read(16384)\r\n"
        " if not b: break\r\n"
        " h.update(b)\r\n"
        " n+=len(b)\r\n"
        "f.close()\r\n"
        "print('__HASH__',n,ubinascii.hexlify(h.digest()).decode())\r\n"
    ).format(path)
    output = repl.execute(code, timeout=timeout).decode("utf-8", "replace")
    for line in output.splitlines():
        if line.startswith("__HASH__"):
            _, size, digest = line.split()
            return int(size), digest.lower()
    raise RuntimeError("missing hash response for {}: {}".format(path, output))


def _inspect(repl: RawRepl) -> None:
    print("\nK230 vision files:")
    for item in BUNDLE:
        code = (
            "import os\r\n"
            "p={!r}\r\n"
            "try:\r\n print('__STAT__',os.stat(p)[6])\r\n"
            "except Exception as e:\r\n print('__MISSING__')\r\n"
        ).format(item.remote_path)
        output = repl.execute(code).decode("utf-8", "replace")
        state = "missing"
        for line in output.splitlines():
            if line.startswith("__STAT__"):
                state = line.split()[1] + " bytes"
        print("  {:<52} {}".format(item.remote_path, state))

    output = repl.execute(
        "import os\r\n"
        "try:\r\n"
        " s=os.statvfs('/sdcard')\r\n"
        " print('__FREE__',s[0]*s[3])\r\n"
        "except:\r\n print('__FREE__',-1)\r\n"
    ).decode("utf-8", "replace")
    for line in output.splitlines():
        if line.startswith("__FREE__"):
            free = int(line.split()[1])
            if free >= 0:
                print("  SD card free: {:.1f} MiB".format(free / 1024 / 1024))


def _preflight() -> list[tuple[BundleFile, bytes, str]]:
    prepared = []
    print("Local recovery bundle:")
    for item in BUNDLE:
        if item.local_path is not None and not item.local_path.is_file():
            raise FileNotFoundError(item.local_path)
        data = item.read_bytes()
        digest = _sha256(data)
        prepared.append((item, data, digest))
        source = str(item.local_path.relative_to(PROJECT_ROOT)) if item.local_path else "generated autostart"
        print("  {:<52} {:>8} bytes  {}  ({})".format(item.remote_path, len(data), digest[:12], source))
    return prepared


def _upload(repl: RawRepl, item: BundleFile, data: bytes) -> None:
    temporary = item.remote_path + ".new"
    parent = temporary.rsplit("/", 1)[0]
    repl.execute(
        "import os\r\n"
        "try:\r\n os.makedirs({!r})\r\n"
        "except:\r\n pass\r\n"
        "try:\r\n os.remove({!r})\r\n"
        "except:\r\n pass\r\n"
        "print('__READY__')\r\n".format(parent, temporary)
    )

    total_chunks = (len(data) + CHUNK_BYTES - 1) // CHUNK_BYTES
    for index in range(total_chunks):
        chunk = data[index * CHUNK_BYTES:(index + 1) * CHUNK_BYTES]
        encoded = base64.b64encode(chunk).decode("ascii")
        code = (
            "import ubinascii\r\n"
            "d=ubinascii.a2b_base64({!r})\r\n"
            "f=open({!r},'ab')\r\n"
            "n=f.write(d)\r\n"
            "f.close()\r\n"
            "print('__WROTE__',n)\r\n"
        ).format(encoded, temporary)
        output = repl.execute(code, timeout=8.0)
        marker = "__WROTE__ {}".format(len(chunk)).encode()
        if marker not in output:
            raise RuntimeError("K230 did not confirm chunk {}/{} for {}".format(index + 1, total_chunks, temporary))
        if total_chunks > 20 and (index == 0 or (index + 1) % 100 == 0 or index + 1 == total_chunks):
            print("    {:>3}% ({}/{})".format((index + 1) * 100 // total_chunks, index + 1, total_chunks))


def _activate(repl: RawRepl) -> None:
    paths = [item.remote_path for item in BUNDLE]
    code = (
        "import os\r\n"
        "paths={!r}\r\n"
        "done=[]\r\n"
        "try:\r\n"
        " for p in paths:\r\n"
        "  try:\r\n os.remove(p+'.bak')\r\n"
        "  except:\r\n pass\r\n"
        "  try:\r\n os.rename(p,p+'.bak')\r\n"
        "  except:\r\n pass\r\n"
        "  os.rename(p+'.new',p)\r\n"
        "  done.append(p)\r\n"
        " print('__ACTIVATED__',len(done))\r\n"
        "except Exception as e:\r\n"
        " for p in done:\r\n"
        "  try:\r\n os.remove(p)\r\n"
        "  except:\r\n pass\r\n"
        "  try:\r\n os.rename(p+'.bak',p)\r\n"
        "  except:\r\n pass\r\n"
        " print('__ROLLED_BACK__',repr(e))\r\n"
    ).format(paths)
    output = repl.execute(code, timeout=30.0)
    marker = "__ACTIVATED__ {}".format(len(paths)).encode()
    if marker not in output:
        raise RuntimeError("bundle activation failed: {!r}".format(output))


def _rollback(repl: RawRepl) -> None:
    paths = [item.remote_path for item in BUNDLE]
    code = (
        "import os\r\n"
        "paths={!r}\r\n"
        "n=0\r\n"
        "for p in paths:\r\n"
        " try:\r\n"
        "  os.remove(p+'.failed') if p+'.failed' in [] else None\r\n"
        "  os.rename(p,p+'.failed')\r\n"
        "  os.rename(p+'.bak',p)\r\n"
        "  n+=1\r\n"
        " except:\r\n"
        "  try:\r\n os.rename(p+'.failed',p)\r\n"
        "  except:\r\n pass\r\n"
        "print('__ROLLBACK__',n)\r\n"
    ).format(paths)
    output = repl.execute(code, timeout=30.0).decode("utf-8", "replace")
    print(output.strip())


def deploy(repl: RawRepl) -> None:
    prepared = _preflight()
    print("\nCurrent device state before deployment:")
    _inspect(repl)
    print("\nUploading complete bundle to temporary files...")
    for item, data, digest in prepared:
        print("  Uploading {} ({} bytes)".format(item.remote_path, len(data)))
        _upload(repl, item, data)
        remote_size, remote_digest = _remote_sha256(repl, item.remote_path + ".new")
        if remote_size != len(data) or remote_digest != digest:
            raise RuntimeError(
                "verification failed for {}: local {} {}, remote {} {}".format(
                    item.remote_path, len(data), digest, remote_size, remote_digest
                )
            )
        print("    SHA-256 verified: {}".format(digest))

    print("\nAll temporary files verified; activating bundle...")
    _activate(repl)
    for item, data, digest in prepared:
        remote_size, remote_digest = _remote_sha256(repl, item.remote_path)
        if remote_size != len(data) or remote_digest != digest:
            raise RuntimeError("post-activation verification failed for {}".format(item.remote_path))
    print("Deployment and post-activation verification succeeded.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("inspect", "deploy", "rollback"))
    parser.add_argument("--port", help="K230 serial port, for example COM6; auto-detects if unique")
    parser.add_argument("--no-start", action="store_true", help="do not reboot and start vision after deploy/rollback")
    args = parser.parse_args()

    try:
        port = _resolve_port(args.port)
        print("Using {} at {} baud.".format(port, BAUD))
        repl = RawRepl(port)
        try:
            repl.enter()
            if args.command == "inspect":
                _preflight()
                _inspect(repl)
            elif args.command == "deploy":
                deploy(repl)
            else:
                _rollback(repl)

            if args.command != "inspect" and not args.no_start:
                print("\nSoft-resetting K230 and monitoring startup...")
                boot_log = repl.soft_reset_and_monitor()
                if args.command == "deploy" and "[K230] Running: YOLO NPU" not in boot_log:
                    raise RuntimeError("deployment verified, but YOLO startup marker was not observed")
        finally:
            repl.close()
    except (OSError, RuntimeError, TimeoutError, FileNotFoundError, serial.SerialException) as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
