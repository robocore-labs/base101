/*
 * ROS message types used by the Axon firmware (picoserdes USER_TYPE_FILE).
 *
 * Subset of pico-ros examples/example_types.h:
 *   - sensor_msgs/JointState        (joint state telemetry)
 *   - std_msgs/Float64MultiArray    (motor command topics)
 *   - std_msgs/Float32, Int32       (per-motor telemetry)
 *   - sensor_msgs/Imu, MagneticField, Temperature (BNO055 IMU)
 *   - geometry_msgs/Quaternion, Vector3 (Imu/MagneticField members)
 *
 * Names and RIHS01 hashes must match the ROS 2 definitions exactly or
 * rmw_zenoh will not match the endpoints.
 */

#ifndef AXON_TYPES_H
#define AXON_TYPES_H

#define MSG_LIST(BTYPE, CTYPE, TTYPE, FIELD, ARRAY, SEQUENCE) \
    CTYPE(ros_Time, \
        "builtin_interfaces::msg::dds_::Time", \
        "b106235e25a4c5ed35098aa0a61a3ee9c9b18d197f398b0e4206cea9acf9c197", \
        FIELD(int32_t, sec) \
        FIELD(uint32_t, nanosec) \
    ) \
    CTYPE(ros_Header, \
        "std_msgs::msg::dds_::Header", \
        "f49fb3ae2cf070f793645ff749683ac6b06203e41c891e17701b1cb597ce6a01", \
        FIELD(ros_Time, stamp) \
        FIELD(rstring, frame_id) \
    ) \
    CTYPE(ros_Vector3, \
        "geometry_msgs::msg::dds_::Vector3", \
        "cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d", \
        FIELD(double, x) \
        FIELD(double, y) \
        FIELD(double, z) \
    ) \
    CTYPE(ros_Quaternion, \
        "geometry_msgs::msg::dds_::Quaternion", \
        "8a765f66778c8ff7c8ab94afcc590a2ed5325a1d9a076ffff38fbce36f458684", \
        FIELD(double, x) \
        FIELD(double, y) \
        FIELD(double, z) \
        FIELD(double, w) \
    ) \
    BTYPE(ros_Float32, \
        "std_msgs::msg::dds_::Float32", \
        "7170d3d8f841f7be3172ce5f4f59f3a4d7f63b0447e8b33327601ad64d83d6e2", \
        float \
    ) \
    BTYPE(ros_Int32, \
        "std_msgs::msg::dds_::Int32", \
        "b6578ded3c58c626cfe8d1a6fb6e04f706f97e9f03d2727c9ff4e74b1cef0deb", \
        int32_t \
    ) \
    CTYPE(ros_MultiArrayDimension, \
        "std_msgs::msg::dds_::MultiArrayDimension", \
        "5e773a60a4c7fc8a54985f307c7837aa2994252a126c301957a24e31282c9cbe", \
        FIELD(rstring, label) \
        FIELD(uint32_t, size) \
        FIELD(uint32_t, stride) \
    ) \
    CTYPE(ros_MultiArrayLayout, \
        "std_msgs::msg::dds_::MultiArrayLayout", \
        "4c66e6f78e740ac103a94cf63259f968e48c617e7699e829b63c21a5cb50dac6", \
        SEQUENCE(ros_MultiArrayDimension, dim) \
        FIELD(uint32_t, data_offset) \
    ) \
    CTYPE(ros_Float64MultiArray, \
        "std_msgs::msg::dds_::Float64MultiArray", \
        "1025ddc6b9552d191f89ef1a8d2f60f3d373e28b283d8891ddcc974e8c55397f", \
        FIELD(ros_MultiArrayLayout, layout) \
        SEQUENCE(double, data) \
    ) \
    CTYPE(ros_JointState, \
        "sensor_msgs::msg::dds_::JointState", \
        "a13ee3a330e346c9d87b5aa18d24e11690752bd33a0350f11c5882bc9179260e", \
        FIELD(ros_Header, header) \
        SEQUENCE(rstring, name) \
        SEQUENCE(double, position) \
        SEQUENCE(double, velocity) \
        SEQUENCE(double, effort) \
    ) \
    CTYPE(ros_Imu, \
        "sensor_msgs::msg::dds_::Imu", \
        "7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b", \
        FIELD(ros_Header, header) \
        FIELD(ros_Quaternion, orientation) \
        ARRAY(double, orientation_covariance, 9) \
        FIELD(ros_Vector3, angular_velocity) \
        ARRAY(double, angular_velocity_covariance, 9) \
        FIELD(ros_Vector3, linear_acceleration) \
        ARRAY(double, linear_acceleration_covariance, 9) \
    ) \
    CTYPE(ros_MagneticField, \
        "sensor_msgs::msg::dds_::MagneticField", \
        "e80f32f56a20486c9923008fc1a1db07bbb273cbbf6a5b3bfa00835ee00e4dff", \
        FIELD(ros_Header, header) \
        FIELD(ros_Vector3, magnetic_field) \
        ARRAY(double, magnetic_field_covariance, 9) \
    ) \
    CTYPE(ros_Temperature, \
        "sensor_msgs::msg::dds_::Temperature", \
        "72514a14126ab9f8a9abec974c78e5610a367b59db5da355ff1fb982d5bad4b8", \
        FIELD(ros_Header, header) \
        FIELD(double, temperature) \
        FIELD(double, variance) \
    )

#define SRV_LIST(SRV, REQUEST, REPLY, FIELD, ARRAY, SEQUENCE)

#endif // AXON_TYPES_H
