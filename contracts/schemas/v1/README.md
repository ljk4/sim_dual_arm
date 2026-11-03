# Schema v1

包括 `robot_profile`、`observation`、`action`、`runtime_config`、`run_event`、
`run_status` 和 `episode_manifest`。部分是历史单相机 mock 格式，部分配置/日志格式
仍在使用。版本按具体类型分别演进，不能因为机器人 profile 升到 v2 就将所有
runtime 配置或日志也改为 v2；旧记录仍按原字段严格读取。

目录导航：[上级目录](../README.md)。
