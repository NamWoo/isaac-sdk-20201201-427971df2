load("//bzl:module.bzl", "isaac_app")

isaac_app(
    name = "carter_data_collection",
    data = [
        "//apps/assets/maps",
        "//apps/carter:2d_carter_subgraph",
        "//apps/carter/robots",
        "//packages/cask/apps:record_subgraph",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
    ],
    modules = [
        "imu",
        "navigation",
        "perception",
        "planner",
        "realsense",
        "rgbd_processing",
        "segway",
        "sensors:joystick",
        "velodyne_lidar",
        "viewers",
    ],
)
