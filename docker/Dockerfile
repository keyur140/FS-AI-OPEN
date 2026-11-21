# fsai-open: one pinned image with ROS 2 Humble, the bridge, the emulator and all tests
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
#
# ackermann_msgs is built from vendored source (ros2/src/third_party) so no access to packages.ros.org is needed.
# Base image pinned by digest (ros:humble-ros-base, Ubuntu 22.04, Python 3.10) so every build starts identical.
FROM ros:humble-ros-base@sha256:d9f14e84747bea639a65e00cf871d2c68ae55385d2acf6239614de0cadb41dcc

ENV DEBIAN_FRONTEND=noninteractive PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip can-utils iproute2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/fsai-open
COPY requirements.txt .
# Optional: a build secret "extra_ca" adds a company/proxy CA certificate if a network needs one
# (docker build --secret id=extra_ca,src=/path/ca.crt ...). Without it, normal TLS verification is used.
RUN --mount=type=secret,id=extra_ca,required=false \
    if [ -s /run/secrets/extra_ca ]; then export PIP_CERT=/run/secrets/extra_ca; fi; \
    pip3 install --no-cache-dir -r requirements.txt

COPY . .
# The 2026 DBC is not redistributable; fetch the pinned version (hash-checked). Offline build still works:
# the DBC cross-check tests then skip with a clear message.
RUN --mount=type=secret,id=extra_ca,required=false \
    if [ -s /run/secrets/extra_ca ]; then export SSL_CERT_FILE=/run/secrets/extra_ca; fi; \
    python3 tools/fetch_dbc.py || true
# The fsai package is used in place (setuptools on Ubuntu 22.04 predates pyproject metadata).
ENV PYTHONPATH=/opt/fsai-open
# One maths thread per process: the matrices are small, and several ROS 2 nodes share the CPU. Without
# this, each node starts a thread per core and they fight (measured: 280% CPU and 0.3 s stalls).
ENV OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
RUN . /opt/ros/humble/setup.sh \
    && cd ros2 && colcon build --event-handlers console_direct- \
    && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["demo"]
