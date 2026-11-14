```yaml
version: '2.4'

services:
  zenoh-router:
    build:
      context: .
      dockerfile: docker/Dockerfile.zenoh
    privileged: true
    network_mode: host
    restart: on-failure
    environment:
      BASE101_ENABLE_HARDWARE: 'false'
      BASE101_ZENOH_DEVICE: /dev/serial/by-id/usb-RoboCore_Link101_ROS_Node_983DBCDBA0D7E993-if00

  bringup:
    build:
      context: .
      dockerfile: docker/Dockerfile.hardware
    privileged: true
    network_mode: host
    restart: on-failure
    environment:
      BASE101_ENABLE_HARDWARE: 'false'
      BASE101_ARM: 'false'
      BASE101_LIDAR_DEVICE: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
    command: [base101-hardware-service, bringup]

  camera:
    build:
      context: .
      dockerfile: docker/Dockerfile.hardware
    privileged: true
    network_mode: host
    restart: on-failure
    environment:
      BASE101_ENABLE_CAMERA: 'false'
    command: [base101-hardware-service, camera]

  navigation:
    build:
      context: .
      dockerfile: docker/Dockerfile.hardware
    network_mode: host
    restart: on-failure
    environment:
      BASE101_ENABLE_MAPPING: 'false'
      BASE101_ENABLE_NAVIGATION: 'false'
    volumes:
      - robocore-maps:/var/lib/robocore/maps
    command: [base101-hardware-service, navigation]

  visualiser:
    build:
      context: .
      dockerfile: docker/Dockerfile.hardware
    network_mode: host
    restart: on-failure
    command: [base101-hardware-service, visualiser]

  robocore-agent:
    build:
      context: .
      dockerfile: docker/Dockerfile.agent
    network_mode: host
    restart: on-failure
    stop_grace_period: 75s
    depends_on:
      - zenoh-router
    volumes:
      - robocore-state:/var/lib/robocore
      - robocore-maps:/var/lib/robocore/maps

  ringfour-daemon:
    image: ringfour-daemon:latest
    restart: on-failure
    depends_on:
      - ringfour-runner
    environment:
      DAEMON_DB_PATH: /data/ringfour.db
      RUNNER_URL: http://ringfour-runner:8082
      RUNNER_STAGE_DIR: /staged
      # Set PLATFORM_API_URL and RUNNER_AUTH_TOKEN in Balena.
    volumes:
      - ringfour-daemon-data:/data
      - ringfour-staged-data:/staged
    ports:
      - "127.0.0.1:8081:8081"

  ringfour-runner:
    image: ringfour-runner:latest
    restart: on-failure
    environment:
      RUNNER_STAGE_DIR: /staged
      # Set the same RUNNER_AUTH_TOKEN in Balena.
    volumes:
      - ringfour-runner-data:/data
      - ringfour-staged-data:/staged:ro

volumes:
  robocore-maps:
  robocore-state:
  ringfour-daemon-data:
  ringfour-runner-data:
  ringfour-staged-data:
```

Set `PLATFORM_API_URL` for the daemon and the same random `RUNNER_AUTH_TOKEN` for both Ringfour services in Balena.
