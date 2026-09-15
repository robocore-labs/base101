# Zenoh router with SERIAL transport for the Axon board.
#
# The stock `eclipse/zenoh` image is built without the (non-default)
# `transport_serial` Cargo feature, so it cannot speak to the Axon's
# zenoh-pico serial link over USB CDC. This image rebuilds `zenohd` with that
# feature and ships it on a minimal Debian base.
#
# NOTE: we deliberately do NOT overlay onto `eclipse/zenoh:latest` — that image
# is Alpine/musl, and our glibc-built binary fails there at runtime with
# "exec /usr/local/bin/zenohd: no such file or directory" (the musl image has
# no ld-linux glibc loader). The runtime base below matches the glibc builder.
#
# IMPORTANT: keep the router protocol-compatible with the firmware's vendored
# zenoh-pico. That is currently 1.9.0 (see lib/zenoh-pico/include/zenoh-pico.h,
# `#define ZENOH_PICO`). Override with --build-arg ZENOH_REF=<tag> if you bump
# zenoh-pico.
ARG ZENOH_REF=1.9.0

# --- builder: compile zenohd with transport_serial -------------------------
FROM rust:bookworm AS builder
ARG ZENOH_REF
RUN apt-get update && apt-get install -y --no-install-recommends \
        git clang libclang-dev && rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 --branch "${ZENOH_REF}" https://github.com/eclipse-zenoh/zenoh /src
WORKDIR /src
# `transport_serial` is a feature of the `zenoh` crate, NOT the `zenohd` binary
# crate (zenohd 1.x only defines `default`/`shared-memory`). Enable it on the
# dependency with the `zenoh/<feature>` syntax, else cargo errors with
# "the package 'zenohd' does not contain this feature: transport_serial".
RUN cargo build --release -p zenohd --features zenoh/transport_serial \
 && cp target/release/zenohd /zenohd

# --- runtime: minimal glibc base + the serial-enabled binary ---------------
FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /zenohd /usr/local/bin/zenohd
COPY zenoh-serial.json5 /etc/zenoh/serial.json5

ENTRYPOINT ["/usr/local/bin/zenohd"]
CMD ["-c", "/etc/zenoh/serial.json5"]
