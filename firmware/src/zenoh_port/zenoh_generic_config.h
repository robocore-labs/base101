/*
 * zenoh-pico configuration for the Axon port (ZENOH_GENERIC).
 *
 * With a generic platform, zenoh-pico's generated config.h delegates the
 * ENTIRE configuration to this header — the CMake cache variables do not
 * apply. Keep this in sync with the intent in the top-level
 * CMakeLists.txt: single-threaded, serial link only, client mode against
 * a zenohd router with rmw_zenoh-compatible lease settings.
 */

#ifndef ZENOH_GENERIC_CONFIG_AXON_H
#define ZENOH_GENERIC_CONFIG_AXON_H

/*--- Sizes and timing ---*/
#define Z_FRAG_MAX_SIZE 4096
#define Z_BATCH_UNICAST_SIZE 2048
#define Z_BATCH_MULTICAST_SIZE 2048
#define Z_CONFIG_SOCKET_TIMEOUT 100
// rmw_zenohd-compatible lease (pico-ros recommendation)
#define Z_TRANSPORT_LEASE 60000
#define Z_TRANSPORT_LEASE_EXPIRE_FACTOR 2
#define Z_RUNTIME_MAX_TASKS 64
#define Z_TRANSPORT_ACCEPT_TIMEOUT 1000
#define Z_TRANSPORT_CONNECT_TIMEOUT 10000

/*--- Features ---*/
#define Z_FEATURE_CONNECTIVITY 0
#define Z_FEATURE_MULTI_THREAD 0
#define Z_FEATURE_PUBLICATION 1
#define Z_FEATURE_ADVANCED_PUBLICATION 0
#define Z_FEATURE_SUBSCRIPTION 1
#define Z_FEATURE_ADVANCED_SUBSCRIPTION 0
#define Z_FEATURE_QUERY 1
#define Z_FEATURE_QUERYABLE 1
#define Z_FEATURE_LIVELINESS 1
#define Z_FEATURE_RAWETH_TRANSPORT 0
#define Z_FEATURE_INTEREST 1

/*--- Links: serial only, over the Axon USB CDC port ---*/
#define Z_FEATURE_LINK_TCP 0
#define Z_FEATURE_LINK_BLUETOOTH 0
#define Z_FEATURE_LINK_WS 0
#define Z_FEATURE_LINK_SERIAL 1
#define Z_FEATURE_LINK_SERIAL_USB 0
#define Z_FEATURE_LINK_TLS 0
#define Z_FEATURE_SCOUTING 0
#define Z_FEATURE_LINK_UDP_MULTICAST 0
#define Z_FEATURE_LINK_UDP_UNICAST 0
#define Z_FEATURE_MULTICAST_TRANSPORT 0
#define Z_FEATURE_UNICAST_TRANSPORT 1

#define Z_FEATURE_FRAGMENTATION 1
#define Z_FEATURE_ENCODING_VALUES 1
#define Z_FEATURE_TCP_NODELAY 1
#define Z_FEATURE_LOCAL_SUBSCRIBER 0
#define Z_FEATURE_LOCAL_QUERYABLE 0
#define Z_FEATURE_SESSION_CHECK 1
#define Z_FEATURE_BATCHING 1
#define Z_FEATURE_BATCH_TX_MUTEX 0
#define Z_FEATURE_BATCH_PEER_MUTEX 0
#define Z_FEATURE_MATCHING 1
#define Z_FEATURE_RX_CACHE 0
#define Z_FEATURE_UNICAST_PEER 0
#define Z_FEATURE_AUTO_RECONNECT 1
#define Z_FEATURE_MULTICAST_DECLARATIONS 0
#define Z_FEATURE_ADMIN_SPACE 0

#endif /* ZENOH_GENERIC_CONFIG_AXON_H */
