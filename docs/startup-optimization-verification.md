# 网页进入等待优化

用户从双臂页面进入，经模型下载、解压、原生模型编译、Three.js 建图，到主画面和三路相机出图。
本次优化集中在这条加载链路。该阶段模型、碰撞及控制参数保持原内容；测量时尚未推送或部署。
本文件为批量文件写入阶段的历史对照；后续针对线上首访的模型传输压缩与 CDN 修复，
以及真实线上和受限网络审计见 [线上首访验证](deployment-performance-verification.md)。

## 瓶颈与改动

双臂场景加载的 XML / 网格解压后为 65,267,921 字节，压缩资源响应合计 21,040,158 字节。
旧接口 `MjVFS.addBuffer` 在 JS / WASM 绑定中逐字节转换，生产对照中仅文件写入就占用 16.90–17.27 秒。
改为已导出的 `FS.writeFile` 批量写入类型数组，仍由原生 `MjModel.from_xml_path` 解析原始文件，
同一批资源写入耗时降为 19.6–19.7 毫秒。

`model-file-store.js` 为每次加载建立独立内存目录，支持相对 include / 网格路径；
成功和失败后都清理源文件，独立加载互不删除资源。没有批量文件接口的运行时自动回退 VFS。
`modelFiles=vfs` 查询参数用于同包对照；默认使用批量接口。
原有 6 路下载、16 MiB 加在途请求的缓冲上限、gzip、文件内容 hash 均保留。

双臂生产页面原先没有注册 Service Worker，现在复用同一注册与按文件缓存逻辑。
缓存注册失败不会让页面等待一个永远不就绪的 Worker。
首轮 XML 请求可能早于 Worker 接管：本次首轮缓存含 41 个网格资源，再次打开后补全为 42 个模型文件。
因此这里保证的是已缓存资源的复用，不宣称首次访问即全站离线可用。

## 同一生产包的实际对照

环境：Windows、本机 Chrome、AMD Radeon 780M / ANGLE D3D11、1100 × 850、启用三路相机，
本机 HTTP 预览，无带宽模拟。每种接口单独创建浏览器上下文，先首次进入，再在同一上下文再次进入。
两种接口均启用 Service Worker，第二轮使用已缓存的资源；未强制 SwiftShader。

| 指标 | 旧 VFS 首次 | 批量写入首次 | 旧 VFS 再次 | 批量写入再次 |
| --- | ---: | ---: | ---: | ---: |
| 导航至确认主视图首绘完成 | 28.48 s | 14.35 s | 20.71 s | 3.86 s |
| 导航至三路相机均有有效像素 | 28.68 s | 14.51 s | 20.94 s | 4.02 s |
| 模型源文件写入累计 | 16.905 s | 0.0196 s | 17.267 s | 0.0197 s |
| 原生 XML 编译 | 2.183 s | 2.187 s | 2.047 s | 2.286 s |

三路画面就绪的等待在这次对照中首次减少约 49.4%，再次减少约 80.8%。
模型解压和响应字节数两组完全相同；`transferredBytes` 是响应载荷合计，缓存命中时也计入，
不能拿它或 Service Worker 下的 `ResourceTiming.transferSize=0` 直接当作实际网络流量。
两个冷启动的 WASM 就绪时间分别为 2.42 / 4.69 秒，包含 CDN 波动，因此只作为本机短测结果。

“首绘完成”从导航开始计时，由浏览器确认 `startupMetrics().firstFrameMs` 已产生；
它包含导航及主线程排队，不等同浏览器标准 FCP / LCP。
`firstFrameMs` 本身从双臂 `main()` 开始，到首次 `view.render()` 返回，分别为
25.57 / 11.26 秒（首次）和 20.54 / 3.60 秒（再次）。
控件启用早于画面绘制，不能用启用时间替代可见场景就绪时间。

两组页面异常数均为 0，真实 GPU 信息和原始资源记录保存在
`web_mujoco/.reports/startup-production-vfs-control.json`、`startup-production-memfs.json`。
报告目录不入库，以上表格保留本次测量值，命令可重新生成原始报告。

## 正确性与生产验证

`npm run test:loading` 已通过：

- 原生单臂、AGV、双臂场景分别有 66 / 92 / 231 个几何体。
- 双臂经两种接口加载后，顶点 / 三角面、碰撞与惯性 / 关节参数摘要完全一致；
  顺序执行 100 步原生物理后，`qpos` / `qvel` / 时间完全一致。
- 两个独立目录可以同时解析相对 include；成功、编译失败和部分写入失败都清理资源。
- 拒绝越出目录的路径；接口缺失时回退 VFS；WASM 流式 / MIME 回退、超时、
  下载背压及逐文件缓存更新测试通过。

根路径和 `/reBot_RS_Sim/` 子路径生产构建均已通过；子路径入口、模型、视觉 Worker 和
Service Worker 的资源前缀检查通过，测试 harness 没有进入部署包。
最终根路径生产包的实际 Chrome 测试 `test-visual-ui.mjs --hardware` 已通过：
两项纯接触 Worker RGB-D 任务均完成，三路画面独立且随动作变化，
暂停 / 继续、折叠、390 px 手机布局、复位和模式切换均通过，页面异常数为 0。
证据为 `web_mujoco/.reports/visual-ui-production.json` 及桌面 / 手机截图。
已恢复根路径构建，供本机预览；没有以根路径 UI 测试宣称线上部署已验证。

## 复现

在 `web_mujoco` 中执行：

```bash
npm run test:loading
npm run build
node node_modules/vite/bin/vite.js preview --host 127.0.0.1 --port 4173 --strictPort
```

另开终端，依次执行，避免同时运行多个大模型测试：

```bash
node scripts/benchmark-startup.mjs --label=production-memfs --url="http://127.0.0.1:4173/?variant=dual"
node scripts/benchmark-startup.mjs --label=production-vfs-control --url="http://127.0.0.1:4173/?variant=dual&modelFiles=vfs"
node scripts/test-visual-ui.mjs --hardware --url="http://127.0.0.1:4173/?variant=dual"
```

首次传输仍需要约 21 MB 模型资源及 WASM / 页面资产，慢网络上还会有带宽等待；
XML 编译和首次 GPU 着色器编译也仍有开销。下一步可分别测真实部署端的带宽 / CDN，
以及低配置设备上的首帧编译，再决定是否进一步拆分资源。

没有采用整场景预编译 MJB：本场景生成的二进制为 236.50 MB，gzip 后仍为 111.19 MB，
约为当前模型响应总量的 5.3 倍，虽能缩短原生编译，却加重首次下载。
诊断脚本为 `scripts/diagnose-startup.mjs`，其产物未作为运行资产引入。
