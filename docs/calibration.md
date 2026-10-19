# 三类标定

日常标定只保留 `table`、`camera`、`angles` 三个入口。它们都在本地工作站启动；
需要相机或编码器时通过 SSH 使用远端设备，拟合与模型显示留在本地。

| 类别 | 解决的问题 | 不包含的内容 |
| --- | --- | --- |
| 桌面相对位置 `table` | 相机相对桌面的位姿、两臂基座相对桌面的位置/朝向、两臂基座间距 | 不需要电机角度，不操作电机 |
| 相机 `camera` | 内参、畸变、标定图像分辨率与焦距条件 | 不用内参拟合代替基座位置测量 |
| 机械臂角度 `angles` | 每台机械臂独立的 raw tick → rad 零点、方向、比例和第三点检验 | 不写 EEPROM 原点或自动把当前位置当作零点 |

## 1. 桌面相对位置

```sh
./scripts/lerobot-arm calibrate table
```

使桌面主标记及两臂基座标记可见，保持桌面、相机和基座不动。
此入口使用 camera-only 模式，不连接机械臂串口，因此机械臂未上电也能标定其基座位置。
它重新收集两臂基座标定，沿用现有安装几何和质量阈值；旧基座记录由原有流程先备份。
观察两臂状态均变为 `fixed`，再关闭窗口。未见标记、安装拟合不通过或采样不稳定时不会接受新基座。

看见图案不等于已接受基座：桌面主参考为 AprilTag `36h11` / ID 3（黑边 150 mm），
黑臂与白臂分别为 ArUco `4X4_50` / ID 7、6（黑边 50 mm，2026-10-19 更换，中心及安装不变）。
基座检测根据本帧候选四边形寻找真实黑边并精确核对编码，不再依赖旧照片中的固定像素位置。
桌面重投影误差须不超过 2 px、基座不超过 1.5 px（均为处理图像坐标）。
主参考可见时，各基座独立累计 20 个有效观测，覆盖至少 1 秒；
样本相对中值的最大位置偏差须不超过 2 mm、朝向偏差不超过约 1.43°。
`markers_lost` 表示还没获得合格基座标记，`pose_rejected` 表示已见标记但几何不符，
`tracked` 表示正在积累，`fixed` 才表示标定已接受。
更换前的 42 mm 基座记录与当前角点配置签名不符，不会作为当前基座标定加载；
2026-10-19 已重新采集并同步两臂基座记录和当前场景。新贴纸的编码朝向与旧贴纸不同，
角点顺序已按双臂基座都朝向 T 的实物关系核对，不能只靠低重投影误差判断朝向正确。
随后桌面主标记改为 `36h11` / ID 3 并移动；桌面布局、双臂基座和场景均已在新坐标系重采集。
静止桌面拟合只使用训练帧的两张标记中值联合优化相机姿态与辅助标记刚性布局，
保留独立留出验证和原有 2 原生像素、2 mm、0.025 rad 验收门槛。

桌面标记或静态布局移动后，显式重建相关缓存：

```sh
./scripts/lerobot-arm calibrate table --reset-table
```

通过相机观测计算桌面坐标系中的相机位姿；两臂基座位置也在同一坐标系内。
远端分别保存 `data/calibration/<arm>_table_base.json`，本地会话保存在
`logs/table-calibration-*/`，包含 `scene_parameters.json` 中的相机位姿、两臂基座和间距。
该相机专用流程里的关节姿态是预览，不能当成实测角度。
按[双臂基座说明](two-arm-calibration.md)同步接受的基座记录到本地，再使用依赖它们的规划任务。

已有 camera-only 观测也可用于严格的双桌面标记布局拟合：

```sh
./scripts/lerobot-arm calibrate table --capture logs/table-observations \
  --output logs/table-reference-fit
```

输出目录必须未存在，结果写入该目录的 `table-reference.json`，保留原有训练/留出验证。
离线拟合不连接设备，也不自动替换当前运行记录。

## 2. 相机

Linux 相机可以直接打开实时标定页面（不连接电机）：

```sh
./scripts/lerobot-arm calibrate camera --live \
  --device /dev/v4l/by-id/<camera>-video-index0 \
  --capture data/calibration/camera-session --square-mm 30 \
  --output data/calibration/camera-candidate.json --fixed-focus --port 8879
```

在本机浏览器打开 `http://127.0.0.1:8879`。如果相机连接在远端电脑，先用
`ssh -L 8879:127.0.0.1:8879 <robot-host>` 转发，再在远端运行上面的命令。
页面显示实时角点、已保存姿态、覆盖图和停稳进度。点击“开始采集”，在不同位置与倾角
停稳 1–2 秒；点击“暂停并验证内参”计算训练及独立留出误差。支持继续同一目录，
但设备、分辨率、方格尺寸和镜头设置必须一致。新会话需要明确确认固定焦距；
Linux 曝光设置会保存并读回核对，不具备对应曝光控制的相机需单独配置采集。
验证只保存未激活的候选，不修改正在运行的相机参数。

采集不同位置、倾角的 ChArUco 标定板图像，保持镜头焦距锁定。
可以用现有相机专用采集入口录制，然后拟合：

```sh
./scripts/lerobot-arm view tabletop --camera-only --duration 60 --output logs/camera-capture
./scripts/lerobot-arm calibrate camera --capture logs/camera-capture \
  --square-mm 30 --output logs/camera-intrinsics-candidate.json
```

`30` 是示例，必须换成实测棋盘格边长。输出包含内参、畸变、分辨率和独立验证结果；
候选不会自行激活。接受并同步相机配置后，重新做桌面相对位置标定。
已有采集和拟合判据见[相机说明](camera.md)。

## 3. 机械臂角度

```sh
./scripts/lerobot-arm calibrate angles --arm follower
./scripts/lerobot-arm calibrate angles --arm arm1
```

两臂分别标定，窗口直接进入参考姿态/采集模式，无需先打开控制界面再按 B。
手托机械臂，确认六轴扭矩已关闭：`1–6` 选关节，`R` 切换参考角度，`C` 采集。
沿用 0/30/60° 三点测量与独立检验；白臂扳机使用其独立的 0/15/30° 参考点。
采集点不等于扳机两端行程。该入口拒绝运动、夹爪测试和仿真后端参数。

记录仍分别保存为 `data/calibration/follower_mujoco.json` 与 `arm1_mujoco.json`。
接受新记录后同步两端文件及 `robot.json` 的资源指纹，检查相关限位、回位和轨迹身份。
详见[角度标定](angle-calibration.md)和[配置核对](robot-config.md)。

高度与末端标记只是角度映射的精度细化，不再各占一个主标定入口：

```sh
./scripts/lerobot-arm calibrate angles refine-height --help
./scripts/lerobot-arm calibrate angles sample-markers --help
./scripts/lerobot-arm calibrate angles fit-markers --help
```

原先顶层的 `calibrate height`、`calibrate markers`、`calibrate sample-markers` 已移除。
细化流程保留原有的采集、运动许可、独立验证和候选应用条件。
本次整理没有重新测量或覆盖现有已接受的标定数据。
