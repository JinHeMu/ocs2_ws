# ocs2_tracer_jaka

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix tracer_jaka_gazebo)/share

xhost +local:docker

sudo ip link set can0 up type can bitrate 500000


ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py 



ros2 run tracer_jaka_ocs2 tracer_jaka_trajectory_target_node 


sshfs ras@192.168.3.17:/home/ras/ /home/a/nuc_mount/

ros2 launch jaka_driver servo.launch.py

ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true

ros2 launch realsense2_camera rs_launch.py
pointcloud.enable:=true
align_depth.enable:=true \

ros2 launch path_servo_control servo_control.launch.py ros2 service call /path_servo/start std_srvs/srv/Trigger "{}" ros2 service call /path_servo/stop std_srvs/srv/Trigger "{}"

ros2 launch force_admittance_servo force_admittance_servo.launch.py



# 1. 一键启动全系统（含 RViz2）
ros2 launch point_cloud_processor processor_launch.py

# 2. 开始采集点云
ros2 service call /pcd_saver/set_saving \
  point_cloud_processor/srv/SetSaving \
  "{enable: true, save_dir: 'data', max_frames: 1}"

# 步骤 1：裁剪
ros2 service call /crop_point_cloud \
  point_cloud_processor/srv/CropPointCloud \
  "{}"

# 步骤 2：预处理重建
ros2 action send_goal /process_point_cloud \
  point_cloud_processor/action/ProcessPointCloud \
  "{input_pcd_path: '', output_ply_path: ''}" --feedback

# 步骤 3：路径规划
ros2 action send_goal /plan_coverage_path \
  point_cloud_processor/action/PlanCoveragePath \
  "{input_ply_path: '', output_csv_path: '', output_gcode_path: ''}" --feedback


# 4. 规划完成后热重载可视化（无需重启节点）
ros2 service call /coverage_visualizer/reload std_srvs/srv/Trigger



export MUJOCO_GL=egl
ros2 launch tracer_jaka_mujoco bridge.launch.py viewer:=false

export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev
export ISAAC_ROS_NVBLOX_PLUGIN_FORCE_FALLBACK_MATERIAL=1