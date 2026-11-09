# 后端配方

`backend.yaml` 用于基础 Robot API；`backend_stack_teacher.yaml` 用于程序示教；
`backend_stack_inference.yaml` 和 `backend_stack_act_chunked.yaml` 保留各自已使用的
推理/ACT 时序，并共同引用 `profiles/control/panda_follower_v1.yaml` 的跟随预算。
每份文件通过相对路径引用 scene、control、profile 和 schema。
早期仅用于回归对照的配方见[测试夹具](../../../../tests/fixtures/simulation/README.md)。

目录导航：[上级目录](../README.md)。
