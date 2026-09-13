import json
import secrets
import socket
import subprocess
import sys
import time

from physical_ai_lab.bridge_host import PROTOCOL_VERSION


def _unused_local_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


def test_host_bridge_authenticates_commands_and_streams_state() -> None:
    port = _unused_local_port()
    token = secrets.token_urlsafe(32)
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "physical_ai_lab.bridge_host",
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
            "--token",
            token,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    client: socket.socket | None = None
    try:
        deadline = time.monotonic() + 5.0
        while client is None and time.monotonic() < deadline:
            if process.poll() is not None:
                raise AssertionError("host bridge stopped before accepting a client")
            try:
                client = socket.create_connection(("127.0.0.1", port), timeout=0.25)
            except OSError:
                time.sleep(0.05)
        assert client is not None, "host bridge did not open its TCP port"
        client.settimeout(2.0)
        reader = client.makefile("rb")
        client.sendall(
            (
                json.dumps(
                    {"type": "hello", "protocol_version": PROTOCOL_VERSION, "token": "wrong"}
                )
                + "\n"
            ).encode()
        )
        assert reader.readline() == b"", "bridge accepted an invalid authentication token"
        reader.close()
        client.close()
        client = None

        deadline = time.monotonic() + 5.0
        while client is None and time.monotonic() < deadline:
            try:
                client = socket.create_connection(("127.0.0.1", port), timeout=0.25)
            except OSError:
                time.sleep(0.05)
        assert client is not None, "bridge did not accept a client after rejecting authentication"
        client.settimeout(2.0)
        reader = client.makefile("rb")

        client.sendall(
            (
                json.dumps({"type": "hello", "protocol_version": PROTOCOL_VERSION, "token": token})
                + "\n"
            ).encode()
        )
        acknowledgement = json.loads(reader.readline())
        assert acknowledgement["type"] == "hello_ack"

        state = None
        command = (
            json.dumps({"type": "cmd_vel", "linear_x": 0.22, "angular_z": 0.0}).encode() + b"\n"
        )
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            client.sendall(command)
            message = json.loads(reader.readline())
            if message.get("type") == "state":
                state = message
                if state["odometry"]["x"] > 0.05:
                    break
        assert state is not None
        assert state["odometry"]["x"] > 0.05
        assert len(state["scan"]["ranges"]) == 181

        stopped_state = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            message = json.loads(reader.readline())
            if message.get("type") == "state" and message["command"] == {
                "linear_x": 0.0,
                "angular_z": 0.0,
            }:
                stopped_state = message
                break
        assert stopped_state is not None, "command watchdog did not return the robot to zero speed"
    finally:
        if client is not None:
            reader.close()
            client.close()
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)
