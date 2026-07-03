#!/usr/bin/env python3
import os
import stat
import subprocess
import tempfile
import textwrap
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
BUILD_SCRIPT = REPO_ROOT / "integrations/igibson/scripts/03_build_igibson_ros_image.sh"


class BuildIgibsonRosImageTest(unittest.TestCase):
    def test_patches_stale_miniconda_download_before_build(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            vendor_root = Path(temp_dir)
            docker_dir = vendor_root / "iGibson/docker/igibson-ros"
            docker_dir.mkdir(parents=True)

            dockerfile = docker_dir / "Dockerfile"
            dockerfile.write_text(
                textwrap.dedent(
                    """\
                    FROM scratch
                    RUN curl -LO http://repo.continuum.io/miniconda/Miniconda-latest-Linux-x86_64.sh
                    RUN bash Miniconda-latest-Linux-x86_64.sh -p /miniconda -b
                    RUN conda update -y conda
                    RUN pip install --no-cache-dir pytest ray[default,rllib] stable-baselines3 && rm -rf /root/.cache
                    """
                ),
                encoding="utf-8",
            )

            build_marker = docker_dir / "build-called"
            build_sh = docker_dir / "build.sh"
            build_sh.write_text(
                f"#!/usr/bin/env bash\nset -euo pipefail\ntouch {build_marker}\n",
                encoding="utf-8",
            )
            build_sh.chmod(build_sh.stat().st_mode | stat.S_IXUSR)

            env = os.environ.copy()
            env["IGIBSON_VENDOR_ROOT"] = str(vendor_root)
            result = subprocess.run(
                ["bash", str(BUILD_SCRIPT)],
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            patched = dockerfile.read_text(encoding="utf-8")
            self.assertIn("https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh", patched)
            self.assertIn("curl -fsSL -o Miniconda-latest-Linux-x86_64.sh", patched)
            self.assertNotIn("http://repo.continuum.io/miniconda/Miniconda-latest-Linux-x86_64.sh", patched)
            tos_accept = (
                "RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \\\n"
                "    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r"
            )
            self.assertIn(tos_accept, patched)
            self.assertLess(patched.index(tos_accept), patched.index("RUN conda update -y conda"))
            pinned_rl_deps = (
                "RUN pip install --no-cache-dir pip==23.3.2 setuptools==65.5.0 wheel==0.38.4 && "
                "pip install --no-cache-dir pytest 'ray[default,rllib]==1.13.0' "
                "stable-baselines3==1.5.0 && rm -rf /root/.cache"
            )
            self.assertIn(pinned_rl_deps, patched)
            self.assertNotIn("pytest ray[default,rllib] stable-baselines3", patched)
            self.assertTrue(build_marker.exists())


if __name__ == "__main__":
    unittest.main()
