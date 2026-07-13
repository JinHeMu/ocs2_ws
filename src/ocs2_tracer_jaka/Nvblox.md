下面按你贴的 **Isaac ROS release-3.2 / ROS 2 Humble** 文档，给你一套最简单的 **Debian 安装 nvblox** 流程。

## 1. 准备 Isaac ROS 工作区

假设你的工作区是：

```bash
export ISAAC_ROS_WS=~/workspaces/isaac_ros-dev
mkdir -p ${ISAAC_ROS_WS}/src
cd ${ISAAC_ROS_WS}/src
```

克隆 `isaac_ros_common`：

```bash
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
```

## 2. 进入 Isaac ROS Docker 容器

```bash
cd ${ISAAC_ROS_WS}
./src/isaac_ros_common/scripts/run_dev.sh
```

后面的命令都建议在这个容器里执行。

## 3. 安装 isaac_ros_nvblox

在容器内执行：

```bash
sudo apt-get update
sudo apt update
sudo apt-get install -y ros-humble-isaac-ros-nvblox
```

然后安装依赖：

```bash
rosdep update
rosdep install isaac_ros_nvblox
```

Clone isaac_ros_nvblox under ${ISAAC_ROS_WS}/src.

cd ${ISAAC_ROS_WS}/src
git clone --recursive -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git isaac_ros_nvblox
Launch the Docker container using the run_dev.sh script:

cd $ISAAC_ROS_WS/src/isaac_ros_common && \
./scripts/run_dev.sh
Use rosdep to install the package’s dependencies.

sudo apt-get update
rosdep update && rosdep install -i -r --from-paths ${ISAAC_ROS_WS}/src/isaac_ros_nvblox/ --rosdistro humble -y
Build and source the ROS workspace

cd /workspaces/isaac_ros-dev
colcon build --symlink-install --base-paths ${ISAAC_ROS_WS}/src/isaac_ros_nvblox/
source install/setup.bash

## 4. 下载 Quickstart 示例数据

安装下载工具：

```bash
sudo apt-get install -y curl jq tar
```

然后执行文档里的下载脚本：

```bash
NGC_ORG="nvidia"
NGC_TEAM="isaac"
PACKAGE_NAME="isaac_ros_nvblox"
NGC_RESOURCE="isaac_ros_nvblox_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION=3
MINOR_VERSION=2

VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$NGC_RESOURCE&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"

AVAILABLE_VERSIONS=$(curl -s -H "Accept: application/json" "$VERSION_REQ_URL")

LATEST_VERSION_ID=$(echo $AVAILABLE_VERSIONS | jq -r "
    .recipeVersions[]
    | .versionId as \$v
    | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
    | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
    | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
    | \$v
    " | sort -V | tail -n 1
)

if [ -z "$LATEST_VERSION_ID" ]; then
    echo "No corresponding version found for Isaac ROS $MAJOR_VERSION.$MINOR_VERSION"
    echo "Found versions:"
    echo $AVAILABLE_VERSIONS | jq -r '.recipeVersions[].versionId'
else
    mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets
    FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/versions/$LATEST_VERSION_ID/files/$NGC_FILENAME"
    curl -LO --request GET "${FILE_REQ_URL}"
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets
    rm ${NGC_FILENAME}
fi
```

## 5. 运行 nvblox 示例

```bash
ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py \
rosbag:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_nvblox/quickstart \
navigation:=False
```

如果一切正常，你应该能在 RViz 里看到机器人重建出的 mesh，以及上面叠加的 2D ESDF slice。

## 常见问题

如果提示找不到包，先确认你在 Docker 容器里，并且 ROS 环境已经加载：

```bash
source /opt/ros/humble/setup.bash
```

如果 `rosdep install isaac_ros_nvblox` 报错，可以先试：

```bash
sudo apt-get update
rosdep update
```

然后重新执行安装命令。

最推荐你先用 **Debian 安装方式** 跑通 quickstart，再考虑源码安装。

export ISAAC_ROS_NVBLOX_PLUGIN_FORCE_FALLBACK_MATERIAL=1
