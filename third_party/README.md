# 外部源码与仿真资产

只记录固定来源，不把上游仓库、网格或二进制提交进 Git。

- `panda-source.json`：Panda 来源 `google-deepmind/mujoco_menagerie`，固定提交
  `f3b6013629c7b94f8ee93fd977b161fa8350f9bc`，子目录 `franka_emika_panda`，
  使用 Apache-2.0。
- `src/mujoco_menagerie/`：`tools/assets/fetch_panda.py` 生成的 sparse checkout。
- `src/lerobot-act-v0.6.1/`：学习环境使用的固定 LeRobot checkout，来源见
  `deploy/docker/learning-source.json`。

`src/` 已由根 `.gitignore` 排除；上游 README、LICENSE 和原始文件保持不动。

获取与校验：

```bash
python3 tools/assets/fetch_panda.py fetch
python3 tools/assets/fetch_panda.py verify
```

摘要写到 `.build/assets/panda.json`，不写入 checkout。只有来源、HEAD 和子目录
均匹配且无本地修改时才复用已有目录，否则非零退出并保留现场。

目录导航：[上级目录](../README.md)。
