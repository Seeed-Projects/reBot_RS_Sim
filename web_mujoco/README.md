# Web MuJoCo

纯网页 MuJoCo WASM 演示：用官方 `@mujoco/mujoco` 在浏览器里提供四种产品配置：

- 标准机械臂（默认）：`rs_grasp_scene.xml`
- 双臂家居桌面：`rs_dual_tabletop_scene.xml`，两只 B601-RS、完整餐厨角、六件桌面物品，支持杯勺协作和分类整理。
- AGV 底盘款：`rs_agv_scene.xml`
- AGV 园区街区：`rs_agv_campus_scene.xml`，三栋居民楼、十字路口和充电 / 配送区域。

页面顶部可直接切换配置，也可用 `?variant=arm` 或 `?variant=agv` 指定。不启动 ROS、不连真机。

双臂场景使用 [`?variant=dual`](http://127.0.0.1:5173/?variant=dual)。默认“杯勺协作”：
左臂夹住杯柄抬起杯子，右臂从上方垂直夹住两个支撑块之间的勺柄中段，
夹指下探至勺柄两侧，确认两侧接触、闭合停稳后再抬起，对准杯口送入，随后松勺、放杯、退臂。
协作使用完整位姿 IK、关节 PD 和可释放的辅助夹持 weld；松开后依靠杯底、杯壁与勺子的接触支撑。
空心杯与镂空杯耳有独立细节几何，勺头具有凹面；模型没有逐帧传送物品。
任务卡片右上角可收起 / 展开，收起时仍保留任务状态和进度。

切换“分类整理”后点击“开始整理”，
左臂依次把鸡蛋、线轴放入托盘，右臂依次把苹果、青柠放入收纳篮；也可以单选物品。
两侧独立运行关节 PD 与位姿 IK，在同一个 MuJoCo 模型内步进，物品由夹爪摩擦接触搬运，
分类整理没有使用吸附约束。容器落点及支撑接触确认后才计为完成。
支持暂停 / 继续（空格）、复位（R）、1× / 2× / 4× 速度、桌面 / 协作特写 / 房间全景 / 俯视和桌面 RGB 相机。
右侧同时显示桌面俯视、左腕、右腕三个独立实时画面；腕部画面使用模型中的 `left_wrist_rgb` / `right_wrist_rgb` 相机位姿随手臂运动，手机上以三栏显示。
这是预设整理任务，目前没有接入自然语言模型或真机控制。双臂页面当前使用中文界面。

场景 XML 已提交，日常运行不需要 Python。修改基础机械臂模型或桌面布局后，从仓库根目录运行：

```bash
python tools/build_dual_tabletop.py
cd web_mujoco
npm run test:dual
npm run test:cooperation
```

两只机械臂共享网格与材质资源，只有 body、关节和作动器使用左右前缀。
房间增加餐边柜、层架、书籍、窗帘、座椅、落地灯和厨房小物；物品增加釉面、金属凹面、果皮与线轴细节。
桌腿接至桌板底面，四边围裙、下横撑与桌板连接件形成完整支撑。
房间、物品和编织篮使用本地几何和程序纹理，房间细节按材质合并。
原生 MuJoCo 打开 XML 时显示简化外观；物理模型保留杯口与杯耳的空隙。

实验室网页将原有的蓝色储物架占位块替换为金属层架、收纳箱和设备盒，
棕色动态障碍块替换为木托盘、双层周转箱和绑带。细节几何按材质合并，
两件道具合计 4 次绘制，无额外纹理下载。外观完全位于原有碰撞包围盒内，
托盘外观每帧跟随 MuJoCo 几何位姿；导航仍使用完整占用轮廓，货架层间不可穿行。
这些细节仅用于网页渲染，原生 MuJoCo / Gazebo 的模型外观保持简化。
箱盖单独占据顶部厚度，避免与箱体顶面共面闪烁；AGV 主视角采用适合米制场景的
近裁剪距离。园区道路、人行道和底色由同一张地面的互不重叠分区绘制；
停靠区、标线和文字在其上分层显示，避免地面深度冲突。

园区模式使用 `?variant=agv-campus`，例如
[本地园区预览](http://127.0.0.1:5173/?variant=agv-campus)。在“产品配置”中选择
“AGV · 园区街区”，可用 WASD 手动驾驶、点击地面导航，或点击“前往充电区”、
“前往配送区”、“前往抓取区”。充电区和配送区目前用于导航停靠演示，不模拟充电电量或物流任务。

园区按米制搭建，占地 30 × 24 m；小车、机械臂和抓取工位保持原有尺寸。
`src/campus-layout.js` 统一定义建筑 / 花箱 / 设施的轮廓、导航边界和停靠位；
`scripts/build-campus-scene.mjs` 据此生成 MuJoCo 碰撞几何，`npm run dev` 和
`npm run build` 会自动更新 XML。建筑碰撞采用覆盖外观的保守包围盒，室内暂不可进入。
原有实验室和标准机械臂配置继续保留。

详细建筑只在网页中加载；原生 MuJoCo 打开园区 XML 时显示简化建筑轮廓。
该模式没有接入 Gazebo / Nav2 的启动配置或重新生成 ROS 占据地图。

建筑来自 MIT 许可的
[ProceduralBuildingsThreeJS](https://github.com/achrefelouafi/ProceduralBuildingsThreeJS)，
采用其中国居民楼 LOD1 几何，提前生成一份 GLB 并由三个楼栋共享。
纹理内嵌、材质转换为标准 glTF PBR，效果与上游完整 Blender 着色器有所不同。
视窗左上角可在“街区视角”和“跟随小车”之间切换，跟随模式仍可拖动旋转、滚轮缩放。
来源、固定提交与许可见 `public/environments/campus/README.md`。

重新生成建筑素材（需要上游源码；日常启动无需 Blender 或上游仓库）：

```bash
node scripts/import-campus-building.mjs /path/to/ProceduralBuildingsThreeJS
node scripts/build-campus-scene.mjs
npm run test:campus
```

网页源码来自 `/home/robot/reBot_Arm_Mujoco-RS/web_mujoco`，模型统一读取当前仓库的
`rebotarm_mujoco_rs/models/`；源项目保持不变。

## 环境

- Linux 或 macOS，Node.js 20.19+
- Chrome 或 Edge

## 启动

```bash
cd web_mujoco
npm install
npm run dev
```

打开 [http://127.0.0.1:5173](http://127.0.0.1:5173)。AGV 款在观察画面左下角调整
行驶和转向速度，使用键盘方向键/WASD 驾驶，松开即停止，空格键立即停车。点击
“设置导航点”后在地面选点，页面会显示避障路径；也可直接点击“前往抓取区”。

本地校验物理（不需要浏览器）：

```bash
npm run smoke
npm run test:loading
npm run test:variants
npm run test:campus
npm run test:timing
npm run test:surfaces
npm run test:explode
```

## Cloudflare Web Analytics

在 Cloudflare 的 Web Analytics 中添加站点，主机名填写 `seeed-projects.github.io`，
复制安装脚本中 `data-cf-beacon` 的 `token` 值。

在实际发布网站的 GitHub 仓库中打开 Settings → Secrets and variables → Actions →
Variables，创建 `CLOUDFLARE_WEB_ANALYTICS_TOKEN`，值设为该站点 Token，
然后重新运行 GitHub Pages 发布工作流。此 Token 是网页中公开的站点标识，
不需要 Cloudflare API 密钥。

生产构建在设置 Token 后会加入 Cloudflare 的延迟加载脚本。未设置时省略统计脚本，
本地 `npm run dev` 也不会加载统计。要验证本地生产构建，可参考 `.env.example`
将 Token 配置在 `.env.local` 中后执行 `npm run build`。
访问线上网站后，在 Cloudflare 的该站点分析面板查看数据；首次数据可能需要几分钟。
仅向自己的 fork 推送，不会更新 `seeed-projects.github.io`，配置需要部署到实际发布仓库。

## 交互功能

- 产品配置：默认保持标准机械臂，可切换到包含四轮底盘、激光雷达、IMU 和实验室场景的 AGV 底盘款。
- AGV 驾驶：观察画面左下角可调整行驶速度和转向速度；默认 0.65 m/s、1.40 rad/s，限幅与 ROS 2 MuJoCo 桥保持一致。
- AGV 仿真按真实经过时间累计 1 ms 物理步，避免街景渲染掉帧时连带降低车速；左下角显示实测仿真倍率与 FPS，正常实时运行约为 `1.00×`。每帧最多补算 250 ms，长时间卡顿会丢弃额外积压时间；切到后台和复位会重置计时，回到页面不会补跑后台时间。抓取控制仍按不超过 12 ms 的仿真间隔更新。
- 网页导航：10 cm 网格 A* 会避开实验室隔墙、货架和实时位置的动态托盘，并以障碍物净空代价让路线趋向通道中间；这套网页规划不会修改 ROS/Nav2 的膨胀系数。
- 移动抓取：点击“前往抓取区”后 AGV 自动规划、行驶并物理锁定停靠位；随后可像标准机械臂款一样选择物体放入收纳区或启动叠叠乐。AGV 工位整体抬高 136 mm，以保持机械臂与桌面的标准相对几何并避免底盘穿桌。
- 爆炸时间线：先播放整机展示动作，再按夹爪、J6 → J1、基座分阶段拆解；支持拖动进度、暂停和反向还原。
- 防重叠布局：最终位置通过 AABB 分离计算保留 14 mm 间距，并保持在地面上方。
- 零件探索：可直接点击模型或从列表选择零件，查看名称、所属组件、关节、材质和用途说明；支持高亮、隔离与清除选择。
- 双相机画面：侧栏显示全局俯视与 RealSense D405 腕部视角，可切换横排或纵列；纵列中的每一路均可折叠，总开关会同时停止渲染并收起画面。
- 干净相机输出：TCP 球体、拖拽目标、误差线、选中框和 HTML 准星仅用于主界面，不进入相机画面。
- 腕部模型：加入经 B601-RS 实机验证的 D405 30° 向下支架、9 mm 光心对齐偏移和相机外壳代理，并提供独立的模型显示开关。
- 爆炸视图和零件选择只改变 Three.js 显示，不修改 MuJoCo 物理状态。

仓库已配置 GitHub Actions workflow。在仓库的 Settings → Pages 中选择 GitHub Actions
作为发布来源后，推送 `main` 分支或手动运行部署 workflow，即可发布。

- 个人仓库 `Yang-Ci/reBot_RS_Sim`：
  [https://yang-ci.github.io/reBot_RS_Sim/](https://yang-ci.github.io/reBot_RS_Sim/)
- 组织仓库同步上游代码后，还需由管理员单独重命名为 `Seeed-Projects/reBot_RS_Sim`，
  才能使用目标网址：
  [https://seeed-projects.github.io/reBot_RS_Sim/](https://seeed-projects.github.io/reBot_RS_Sim/)

workflow 自动使用当前仓库名设置 `GITHUB_PAGES_BASE`，Vite 据此生成子路径部署包；
仓库名为 `reBot_RS_Sim` 时，路径为 `/reBot_RS_Sim/`，改名前和其他 fork 也会使用各自的仓库名。
Sync fork 只同步代码，不会同步仓库名称或 Pages 设置；无需购买域名或设置 Custom domain。


首次打开会下载所选场景的全部网格，请用 Chrome 或 Edge。
网格通过 gzip 和 6 路并发下载；WASM 初始化期间允许最多 16 MiB
加在途请求的网格缓冲，避免首批下载等待 WASM。生产缓存按文件内容 hash 更新，
只修改 XML 时不重新下载未变化的网格。
生产环境的 `mujoco.wasm` 默认从 jsDelivr CDN 预加载并流式编译，
CDN 请求失败或下载超过 8 秒时回退站内文件；
如需强制使用本地文件，可在构建时设置 `VITE_MUJOCO_WASM_CDN=local`。

本地预览生产包：

```bash
cd web_mujoco
GITHUB_PAGES_BASE=/reBot_RS_Sim/ npm run build
npx vite preview
```

## 里程碑（M1–M4）

| 里程碑 | 内容 | 状态 |
|---|---|---|
| **M1** | RS 臂运动学 | 已完成 |
| **M2** | 物理步进 | 已完成 |
| **M3** | 页内抓取 | 已完成 |
| **M4** | 全局俯视与 D405 腕部相机 | 已完成 |

M0（空场景验证 WASM）已跳过，直接加载 RS 抓取场景。

### M1 运动学（已完成）

加载 `rs_grasp_scene.xml`，滑块直接写 `qpos`，再调用 `mj_forward`。没有 `mj_step`。

验收：

- 手臂、桌面、红/蓝/黄物体可见
- J1–J6 连续转动，模型不瞬移（运动学直连）
- 夹爪滑块同时驱动 `joint7` / `joint_left` / `joint_right`
- 复位回到零位

### M2 物理（已完成）

接上与 `mujoco_sync.py` 对齐的 PD：滑块只改目标，`mj_step` 驱动 `<motor>`。物体可被推倒、掉落。提供 Reset。

标准机械臂模式沿用每帧 4 步（`timestep=0.001`）；AGV 模式按真实经过时间计算步数，以 1 倍速推进仿真。

验收：

- 拖 joint2 时手臂有惯性，不瞬移
- 色块受重力，可被碰到
- Reset 回到初始姿态与物体位置

这步之后才算浏览器里代替了原生 MuJoCo 仿真本体。

### M3 页内抓取

读 `red_cube` / `blue_block` / `yellow_cylinder` 的 `xpos`，先将 J2–J6 收拢到安全高位，再按机械臂基座坐标计算目标方位并单独旋转 J1（使用关节范围内的最短转角），最后进入简化笛卡尔接近：开爪 → 到上方 → 下探 → 闭合 → 抬升。目标会放回桌面上的对应颜色收纳区；叠叠乐模式按蓝、红、黄的顺序堆叠。放置阶段使用接触判断，碰到收纳区或已有堆叠层即松开。

叠叠乐每放好一层都会保持撤离高位，先重新规划到非零的高位抓取准备姿态，再规划到下一个物体，不再中途回零。层间准备姿态采用 J1=0、J2=31.6°、J3=15.4°、J4=18.5°、J5=2.7°、J6=0°。成功或失败时，机械臂会读取浏览器主视角的实时方位，用 J1 面向当前用户视角并将 J4 调整到约 45.8°，使腕部相机保持竖直；随后成功动作由 J6 左右扭头、J5 歪头并将夹爪闭合再张开，失败动作执行低头害羞，最后均停在面向用户的高位中姿态。

全局俯视相机使用模型中定义的 70° 垂直视场角，可同时覆盖完整工作台、收纳区和叠叠乐塔顶；网页预览会直接读取 MJCF 相机 FOV，避免渲染参数与模型配置不一致。

在“一键抓取演示”中选择红、蓝、黄目标后，可直接用机械臂自身朝向定义的 WASD
（W 前、S 后、A 左、D 右）或观察画面中的左键拖动
在桌面内移动；选中物体会呼吸发光，右键拖动（或 Shift + 左键拖动）使用轨迹球方式
调整三维姿态。操作提示固定在观察画面左下角，不再单独占用侧栏；控制器会根据旋转后
的包围范围贴合桌面并限制桌面边界和物体重叠，自动抓取运行时会暂时锁定编辑。

验收：

- 点击抓取后物体 `z` 升高
- 不依赖 ROS 视觉话题；成功判据来自 MuJoCo body 位姿

### M4 全局与腕部相机（已完成）

将场景里的 `overhead_rgb` 和安装在 `gripper_end` 上的 `wrist_rgb` 分别离屏渲染到侧栏，两路画面并排显示并由同一个开关控制。颜色检测仍可直接读取 body 位姿，不必依赖 OpenCV。
