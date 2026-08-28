"""Bot container management (host side).

The bot container is fully airgapped: --network=none, plain Debian +
Python.  Its only windows to the world are the bind mounts we give it:

    /dev/robot   FIFO device tree served by the sim daemon (host)
    /bot         the robot repo the agent wakes up in (per episode)
    /memory      persistent scratch, survives episodes and rebuilds

Everything the harness does inside goes through `docker exec`.
"""

import subprocess

IMAGE = "mazebot-bot"


def image_exists(image=IMAGE):
    r = subprocess.run(["docker", "image", "inspect", image],
                       capture_output=True)
    return r.returncode == 0


def build_image(repo_root, image=IMAGE):
    with open(f"{repo_root}/Dockerfile.bot", "rb") as f:
        subprocess.run(["docker", "build", "-t", image, "-"],
                       stdin=f, check=True)


class BotContainer:
    def __init__(self, name, mounts, image=IMAGE, workdir="/bot"):
        """mounts: {host_path: container_path}"""
        self.name = name
        self.mounts = mounts
        self.image = image
        self.workdir = workdir

    def start(self):
        self.stop()
        cmd = ["docker", "run", "-d", "--network=none",
               "--name", self.name, "-w", self.workdir]
        for host, cont in self.mounts.items():
            cmd += ["-v", f"{host}:{cont}"]
        cmd += [self.image, "sleep", "infinity"]
        subprocess.run(cmd, check=True, capture_output=True)

    def exec(self, command, timeout_s=60, workdir=None):
        """Run a shell command inside; returns (exit_code, output_bytes).

        The in-container `timeout` wraps the command so a blocking device
        read can't wedge the harness; the outer subprocess timeout is a
        backstop for a wedged docker exec itself.
        """
        inner = f"timeout {int(timeout_s)}s bash -c {shell_quote(command)}"
        cmd = ["docker", "exec", "-w", workdir or self.workdir,
               self.name, "bash", "-c", inner]
        try:
            r = subprocess.run(cmd, capture_output=True,
                               timeout=timeout_s + 20)
            return r.returncode, r.stdout + r.stderr
        except subprocess.TimeoutExpired as e:
            out = (e.stdout or b"") + (e.stderr or b"")
            return 124, out + b"\n[harness] exec timed out"

    def cp_in(self, host_path, container_path):
        subprocess.run(["docker", "cp", host_path,
                        f"{self.name}:{container_path}"],
                       check=True, capture_output=True)

    def cp_out(self, container_path, host_path):
        subprocess.run(["docker", "cp", f"{self.name}:{container_path}",
                        host_path], check=True, capture_output=True)

    def stop(self):
        subprocess.run(["docker", "rm", "-f", self.name],
                       capture_output=True)


def shell_quote(s):
    return "'" + s.replace("'", "'\\''") + "'"
