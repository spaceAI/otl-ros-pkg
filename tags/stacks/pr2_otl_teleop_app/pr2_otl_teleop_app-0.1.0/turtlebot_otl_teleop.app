display: Roomba Sensor Controller
description: Drive your roomba by tilting your android phone!
platform: turtlebot
launch: pr2_otl_teleop_app/dummy.launch
interface: pr2_otl_teleop_app/otl_teleop.interface
icon: pr2_otl_teleop_app/otl_roomba.png
clients:
  - type: android
    manager:
      api-level: 9
      intent-action: com.ogutti.ros.android.roomba_app.RoombaController
    app:
      base_control_topic: /turtlebot_node/cmd_vel
      linear_rate: 0.08
      angular_rate: 0.2

