FROM ghcr.io/prefix-dev/pixi:noble-cuda-13.0.0


SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV MUJOCO_GL=egl
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY . .

RUN pixi install --all

RUN pixi run build

CMD pixi run -e artefacts artefacts run "$ARTEFACTS_JOB_NAME"