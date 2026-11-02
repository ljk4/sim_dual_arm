# Learning image targets

| Target | Purpose |
| --- | --- |
| `protocol`, `policy` | Existing ROS-free Policy protocol/fake-service checks; unchanged. |
| `act-system` | Ubuntu 24.04 Python 3.12, FFmpeg, libpython3.12 and libgomp runtime layer. |
| `act-deps` | Rebuilds the pinned 94-package environment from `../learning-requirements.lock` using hashes. |
| `act` | Cross-machine entry; source, pinned LeRobot, weights and data are read-only mounts, output is RW. |
| `act-dev` | This-machine entry; mounts the already verified host venv read-only instead of reinstalling 94 packages. |

The default `act-system` base is `ubuntu:24.04` so the file is portable. A
machine with the verified probe image can build with:

```bash
docker build --target act-dev \
  --build-arg ACT_SYSTEM_IMAGE=sim-dual-arm/learning:system-probe-20261101-v3 \
  -f deploy/docker/learning/Dockerfile -t sim-dual-arm/learning:act-dev .
```

The verified probe image ID is
`sha256:5546a1acc44e5e6d0581903fe37bec311996429498bca8826008f2305c43ffac`.
Do not copy a host virtualenv whose ABI is unknown; use the lock.

Development mounts (all except output are read-only):

```bash
docker run --rm --gpus all --network none \
  -e LEARNING_CACHE_ROOT=/output/cache \
  -v "$PWD":/workspace:ro \
  -v /home/ljk/work/sim_dual_arm/third_party/src/lerobot-act-v0.6.1:/opt/lerobot:ro \
  -v /home/ljk/work/sim_dual_arm/.build/cache:/opt/learning-cache:ro \
  -v /home/ljk/work/sim_dual_arm/.build/learning-dev-venv:/opt/host-learning-venv:ro \
  -v /path/to/learning-data:/data:ro \
  -v /path/to/learning-output:/output:rw \
  sim-dual-arm/learning:act-dev \
  python tools/run/learning.py --project-root /workspace \
    train --config /workspace/configs/learning/act_training.container.yaml
```

The container is offline; the trainer never downloads a backbone or logs to a
Hub/W&B service. The task budget ledger, report and checkpoints belong under the
RW `/output` mount; the HF cache is `/output/cache`. The container configs bind
data/select to `/data` and writable output to `/output`.

For another machine, build `--target act` with its own tag to rebuild the locked dependencies inside the container; `act-dev` requires the exact verified venv mount. Accepted source-export references must also be mounted read-only at their recorded paths (prefer the stable `/source-data` mapping).
