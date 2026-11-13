Robot model assets owned by the simulation end.

- `franka_emika_panda/`: MuJoCo Menagerie model, from
  [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie),
  `franka_emika_panda/` at commit `f3b6013629c7b94f8ee93fd977b161fa8350f9bc`.
  The upstream Apache-2.0 license is included in
  `franka_emika_panda/LICENSE`.
- `marvin/`: original Marvin M6 URDF and STL meshes. Source, import adaptation
  and collision boundaries are documented in `marvin/README.md`.

`physics/assets.py` resolves each model from this directory. Scenario recipes
select `robot.assembly`; they do not carry model file paths.
