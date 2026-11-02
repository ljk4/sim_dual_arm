# 外部仿真资产

这里只记录固定来源，不把上游仓库、网格或二进制提交进 Git。

- `panda-source.json`：唯一维护的上游描述。Panda 来自
  `google-deepmind/mujoco_menagerie` 的固定提交
  `f3b6013629c7b94f8ee93fd977b161fa8350f9bc`，子目录
  `franka_emika_panda`，该子目录使用 Apache-2.0。
- `src/`：`tools/assets/fetch_panda.py` 生成的 sparse checkout，已由根
  `.gitignore` 的 `/third_party/src/` 排除。

```bash
# 下载固定提交到 third_party/src/mujoco_menagerie，并把内容摘要写到
# .build/assets/panda.json（摘要不会写入 checkout）
python3 tools/assets/fetch_panda.py fetch
python3 tools/assets/fetch_panda.py verify
```

已存在目录只有在以下条件全部满足时才会复用：是本工具创建的 Git
checkout、origin 与固定来源一致、HEAD 等于固定提交、子目录没有本地修改。
否则命令以非零退出并保留现有文件，不执行 reset/clean。首次克隆失败时保留
本次 staging 目录与 Git 诊断信息，便于定位。

checkout 保留原始 `scene.xml`、`panda.xml`、网格、`LICENSE` 和来源提交；
不要手写替代机器人。生成的摘要按相对路径、文件大小和分块 SHA-256 记录每个
文件，并给出总摘要，供容器内的只读校验使用。
