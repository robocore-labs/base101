FROM python:3.12-slim

# Docker CLI + compose plugin only — no dockerd. This container talks to the
# HOST's Docker daemon over the mounted socket (docker-outside-of-docker),
# it never runs one itself.
ARG TARGETARCH
ARG DOCKER_VERSION=27.3.1
ARG COMPOSE_VERSION=2.29.7

RUN set -eux; \
    apt-get update && apt-get install -y --no-install-recommends curl ca-certificates && rm -rf /var/lib/apt/lists/*; \
    case "$TARGETARCH" in \
      amd64) DOCKER_ARCH=x86_64; COMPOSE_ARCH=x86_64 ;; \
      arm64) DOCKER_ARCH=aarch64; COMPOSE_ARCH=aarch64 ;; \
      *) echo "unsupported TARGETARCH: $TARGETARCH" >&2; exit 1 ;; \
    esac; \
    curl -fsSL "https://download.docker.com/linux/static/stable/${DOCKER_ARCH}/docker-${DOCKER_VERSION}.tgz" \
      | tar -xz -C /usr/local/bin --strip-components=1 docker/docker; \
    mkdir -p /root/.docker/cli-plugins; \
    curl -fsSL "https://github.com/docker/compose/releases/download/v${COMPOSE_VERSION}/docker-compose-linux-${COMPOSE_ARCH}" \
      -o /root/.docker/cli-plugins/docker-compose; \
    chmod +x /root/.docker/cli-plugins/docker-compose

WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY app ./app
COPY static ./static

EXPOSE 8090
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8090"]
