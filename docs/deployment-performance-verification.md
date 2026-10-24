# 线上首访与资源传输验证

目标是国内外新访客第一次访问部署站点时少下载、少等待。
本轮修复已在本地生产构建验证，并于 2026-10-24 获得推送授权；线上更新状态以 GitHub Actions 的构建与部署结果为准。
上一轮本机加载处理的测量保留在 `startup-optimization-verification.md`，不能将其当作全球线上速度。

## 真实线上基线

2026-10-24 从本机 Chrome 首次访问 `https://yang-ci.github.io/reBot_RS_Sim/?variant=dual`，
关闭浏览器缓存并禁止 Service Worker，避免用已有缓存掩盖首次下载。
一次完整审计记录为页面文档 1.67 s、控件就绪 24.61 s、三路画布就绪 30.88 s，
CDP 汇总传输 24,959,792 字节。已部署旧版相机是 WebGL 画布，另保存截图检查三路画面。
这些是单机、单条网络和旧版本的基线，没有用它代表国内外各地速度。

线上日志暴露了重复 WASM 下载：CDN 模块已下载，却因同步初始化超过 Chrome 主线程限制而报错，
随后又下载站内副本。日志原文包含 `WebAssembly.Instance is disallowed on the main thread`，
CDN 和站内 WASM 的传输分别约 2.40 MB / 2.53 MB。
该失败还留下等待初始化的日志，不能把“最后能打开页面”当作正常 CDN 加载。

## 已实现的改动

1. 用异步 `WebAssembly.instantiate` 接入 Emscripten 初始化回调，传播异步失败。
   [Emscripten 的接口说明](https://emscripten.org/docs/api_reference/module.html#Module.instantiateWasm)
   支持异步完成后调用 success callback，等待期间返回空对象。
2. CDN 超时按连接和数据停止更新计时：连续 8 秒没有新数据才回退，
   持续接收数据的慢连接不会因总时长超过 8 秒重新下载。
3. 二进制 STL 在构建时无损打包：重复顶点 / 法线按原始浮点字节去重，
   面索引差分编码，保留原始 80 字节头、面顺序及属性，再用 gzip 压缩。
   浏览器还原成逐字节相同的 STL 后交给原加载器；没有减少三角面或修改物理。
4. 新格式使用独立的 `.stlp1.gzbin` 地址与源文件 hash。
   不支持 gzip 流的浏览器读取原文件；紧凑资源失败依次回退原 gzip、原文件。
   Service Worker 按源文件清理旧表示，保留其他网格缓存。

41 个机械臂 STL 的 gzip 总量由 **20,818,881 → 13,609,354 字节（减少 34.63%）**。
全部源文件逐字节还原测试通过；本机 Node 还原总耗时约 243 ms，该时间不代表手机端。
原始网格为 65,046,644 字节；双臂含 XML 的解压加载总量仍为 65,267,921 字节。
减少下载量会惠及首次访问，缓存复用则是后续访问的额外收益。

## 相同生产包、无缓存的网络对照

在本地生产 HTTP 服务上，用 Chrome CDP 模拟 **20 Mbps / 80 ms**，
关闭浏览器缓存、禁止 Service Worker、每次新建浏览器上下文。
两组都包含上一轮批量文件写入和本轮异步 WASM 修复，仅切换模型传输格式。

| 指标 | 原 gzip 传输 | 无损紧凑传输 |
| --- | ---: | ---: |
| 总传输（CDP encodedDataLength） | 22.676 MB | 16.304 MB |
| 控件就绪 | 12.69 s | 10.21 s |
| 三路画面有效像素 | 18.72 s | 16.23 s |
| WASM 请求 | 1 个 CDN、0 个站内 | 1 个 CDN、0 个站内 |

这组总传输减少约 28.1%，三路画面就绪减少约 13.3%。
实际传输含 HTTP 压缩 / 头部，和上面的离线 gzip 文件之和不是同一口径。
以上是传输格式的受控比较，**不是新版本已在线上全球验证的结果**。
旧线上 30.88 s 和本地模拟 16.23 s 的网络及版本不同，不能直接作为线上前后提升比例。

另测 **5 Mbps / 160 ms**：三路画面约 36.18 s，总传输仍为 16.304 MB。
CDN WASM 下载持续 19.12 s 且有数据更新，没有重复下载站内副本，页面异常数为 0。
这也说明低带宽下仍会等待，不能承诺所有用户数秒进入。

另主动阻断 jsDelivr：新包成功回退站内 WASM，41 个紧凑模型资源均成功，
三路相机出图，页面异常数为 0；被主动阻断的请求和预期回退警告单独保留在报告中。
本地预览该次站内 WASM 以未压缩响应发送，不能用这次总字节数推算 GitHub Pages 的 gzip 传输。

最终根路径生产包的两项纯接触 Worker RGB-D 协作均完成，三路相机、折叠、暂停、复位、
模式切换和 390 px 手机布局通过。根路径和 `/reBot_RS_Sim/` 构建通过，
Pages 入口 / 模型 / Worker 前缀及 41 个紧凑文件检查通过，测试 harness 未进入部署包。
`test:loading` 和真实 Chrome `test:delivery` 通过，新增回归已加入 CI。

## 证据与复现

原始记录位于忽略入库的 `web_mujoco/.reports/`：
`deployment-live-before.json` / PNG、`deployment-gzip-20mbps.json`、
`deployment-packed-20mbps.json`、`deployment-packed-5mbps.json`、
`deployment-cdn-unavailable.json`、`visual-ui-production.json`。
每份报告保留资源 URL、响应压缩 / 缓存头、实际传输字节、控制台及页面异常。
报告命令运行位置为 `web_mujoco`：

```bash
npm run test:loading
npm run test:delivery
node scripts/audit-deployment.mjs --label=live-before --url="https://yang-ci.github.io/reBot_RS_Sim/?variant=dual"
node scripts/audit-deployment.mjs --label=gzip-20mbps --mbps=20 --latency=80 --url="http://127.0.0.1:4173/?variant=dual&meshTransfer=gzip"
node scripts/audit-deployment.mjs --label=packed-20mbps --mbps=20 --latency=80 --url="http://127.0.0.1:4173/?variant=dual"
node scripts/audit-deployment.mjs --label=packed-5mbps --mbps=5 --latency=160 --url="http://127.0.0.1:4173/?variant=dual"
node scripts/audit-deployment.mjs --label=cdn-unavailable --block-cdn --url="http://127.0.0.1:4173/?variant=dual"
```

## 部署后的验收边界

这些改动在发布新构建后才会影响公众网站。发布后需重新对真实站点执行审计，确认：
新 `.stlp1.gzbin` 请求成功、没有模型回退、CDN 正常时只有一次 WASM 下载，
再用国内外实际线路分别测首次加载，统计分布，不能仅凭本机一次访问下结论。
当前代码没有迁移托管商、购买 CDN 或配置国内资源镜像；
GitHub Pages / jsDelivr 的跨境可达性仍是独立于压缩和客户端处理的部署问题。
