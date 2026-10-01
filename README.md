# UR5e-robotic-arm-interaction
To facilitate the understanding of the DH parameter model of the robotic arm, the orientation of the coordinate axes has been determined
# UR5e 标准 DH 交互演示

# UR5e 标准 DH 交互演示

## 打开与操作

1. 下载整个文件，用浏览器打开 `index.html`。
2. 页面可离线使用，无须联网、安装依赖或启动服务器。
3. 拖动六个关节滑块，或在右侧数值框输入角度，分别调节 q₁～q₆。界面角度单位为度，位置单位为米。
4. 在机械臂画面中拖拽可旋转观察视角；也可选择斜视、正视或俯视。

改变关节角、切换姿态或选择坐标系时，镜头的缩放和观察中心保持不变。如果机械臂转到了画面外，点击 **“完整显示”**，将当前姿态重新放入画面。

**“DH 零角姿态”** 表示 q = (0, 0, 0, 0, 0, 0)°；**“数值例子”** 为 q = (30, −60, 90, −30, 45, 60)°。这里的 DH 零角姿态不等同于机器人示教器中同名或类似名称的回零、Home 姿态。

选择坐标系可显示该坐标系和基座坐标系。O₀～O₆ 表示 DH 原点；x、y、z 轴使用不同颜色。q₆ 会改变法兰的方向，但不会改变 O₆ 的位置，可通过法兰上的方向标记观察。

虚线坐标轴表示朝屏幕里，轴方向说明会随着观察视角变化。箭头在画面上的左、右、上、下表示二维投影，与朝里、朝外的深度方向可以同时存在。正对视线的轴用 ⊗ 表示朝里、⊙ 表示朝外。

## 文件说明

- `index.html`：可直接打开的完整离线演示。
- `src/ur5e.html`：交互图源码，包含几何绘制、关节控制和相机逻辑。
- `build.py`：将源码重新生成完整的 `index.html`；所需渲染支持文件已随包附带。
- `calculation/ur5e_standard_dh.py`：独立的 Python 正运动学计算程序，仅使用 Python 标准库。
- `calculation/UR5e-DH计算详解.md`：中文参数、矩阵推导和数值例子。
- `tools/`：本地页面导出器、样式与状态保存支持代码；已去除未使用的外部 CDN 加载。

## 修改演示

安装 Python 3.10 或更新版本后，在解压目录打开终端。修改 `src/ur5e.html`，然后运行：

```bash
python build.py
```

重新打开或刷新 `index.html` 即可看到修改结果。构建过程会重新生成 `index.html`，因此请在 `src/ur5e.html` 中保留修改。构建使用包内文件，无须原始 Codex 环境。

## 运行数值计算

打印 DH 零角姿态与默认数值例子：

```bash
python calculation/ur5e_standard_dh.py
```

计算自定义关节角，参数单位为度：

```bash
python calculation/ur5e_standard_dh.py --q 17 -83 42 111 -27 63
```

同时保存完整的计算数据：

```bash
python calculation/ur5e_standard_dh.py --q 30 -60 90 -30 45 60 --json calculations.json
```

程序输出各节变换 Aᵢ、累计变换、T₀₆ 和各个 DH 原点的位置，并校验基本变换连乘结果与旋转矩阵。

## 参数与约定

参数来自 [Universal Robots 官方 DH 参数页面](https://www.universal-robots.com/developer/hardware-and-motion/robot-motion-dh-parameters/)，使用其中 UR5e 的标准 DH 标称参数：

| i | aᵢ / m | dᵢ / m | αᵢ / ° |
|---|---:|---:|---:|
| 1 | 0 | 0.1625 | 90 |
| 2 | −0.425 | 0 | 0 |
| 3 | −0.3922 | 0 | 0 |
| 4 | 0 | 0.1333 | 90 |
| 5 | 0 | 0.0997 | −90 |
| 6 | 0 | 0.0996 | 0 |

使用 `Aᵢ = Rz(qᵢ) · Tz(dᵢ) · Tx(aᵢ) · Rx(αᵢ)`，不额外增加关节角偏置；`T₀₆ = A₁ · A₂ · A₃ · A₄ · A₅ · A₆` 将第 6 坐标系中的点转换到基座 DH 坐标系。

图中的机械外壳是简化示意，DH 原点和坐标轴按上述参数计算。模型不含具体机器的出厂标定修正、安装坐标变换或额外工具/TCP 变换。


## MuJoCo 实体模型与渲染

`index.html` 使用本地内嵌网格和 WebGL 实时显示，可直接离线打开；它不是在浏览器内运行 MuJoCo 物理引擎。

`mujoco/model` 含 Menagerie 原始 OBJ、原始 `ur5e.xml`、官方 DH 对齐版 `ur5e_dh.xml`、场景和 BSD-3-Clause 许可证。`mujoco/model/source.json` 记录固定来源版本。实体材质均不透明。

原版 XML 使用略微取整的长度。为了与前文官方 DH 计算完全对应，调整版只改了五处位置：shoulder_link.z = 0.1625；wrist_1_link.z = 0.3922；wrist_2_link.y = 0.1263；wrist_3_link.z = 0.0997；attachment_site.y = 0.0996（单位米）。网格外形和固定旋转保持原来源数据。

图中“立起姿态”为 q = (0, −90, 90, −90, −90, 0)°。它用于观察实体和第4坐标系；DH 零角姿态仍然是所有 q = 0。

原生 MuJoCo 渲染图片在 `mujoco/renders`。如需重新渲染，在解压根目录执行：

```bash
python -m pip install -r mujoco/requirements.txt
python mujoco/render.py
```

渲染脚本使用 Windows 的微软雅黑字体；其他系统可将 fontpath 修改为本机支持中文的字体路径。修改脚本中的 q 数组即可换姿态。Python 渲染依赖 MuJoCo 和 OpenGL；直接打开交互网页不需要这些依赖。

模型来源：https://github.com/google-deepmind/mujoco_menagerie/tree/main/universal_robots_ur5e

实体网格许可证：`mujoco/model/LICENSE`，BSD-3-Clause，2018 ROS Industrial Consortium。

独立核验：200 组随机关节角，MuJoCo 调整版所有body与DH坐标转换、末端attachment_site与T06相符，最大矩阵误差1.33e-15。该数值验证运动学，不代表简化网格的表面误差。

`mujoco/prepare_mesh.py` 保留网格转换和压缩代码；它另外需要 `trimesh` 与 `fast-simplification`。运行后输出 `src/mesh-data.json` 和 gzip/base64 数据。网页的内嵌数据已生成，普通使用无需运行此脚本。交互版保留77,495个三角面，外壳不透明，页面没有外部网络请求。
<img width="2560" height="1346" alt="image" src="https://github.com/user-attachments/assets/3c53d097-9236-443e-a19d-957717796374" />


