"""Normalization statistics preserve small variance around a large position offset."""

import json

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from dual_arm_learning.dataset_manifest import _stable_vector_statistics, _write_vector_statistics


def test_centered_statistics_preserve_float32_quantized_small_variance():
    values = np.asarray([[10000.01], [9999.99]], dtype=np.float32)
    stats = _stable_vector_statistics(values)
    assert stats["mean"] == [10000.0]
    assert stats["std"] == [0.009765625]
    assert stats["count"] == [2]
    assert _stable_vector_statistics(np.full((1000, 1), -1.5708, dtype=np.float32))["std"] == [0.0]


def test_split_and_episode_statistics_share_stored_rows_without_changing_data(tmp_path):
    keys = ("observation.state", "observation.velocity", "action")
    values = [[10000.01], [9999.99], [5000.01], [4999.99]]
    data = pa.table(
        {
            "episode_index": [0, 0, 1, 1],
            **{key: pa.array(values, type=pa.list_(pa.float32())) for key in keys},
        }
    )
    (tmp_path / "data").mkdir()
    data_path = tmp_path / "data/file.parquet"
    pq.write_table(data, data_path)
    stats = {
        key: {name: [0] for name in _stable_vector_statistics(np.asarray(values))} for key in keys
    }
    stats["observation.images.global"] = {"mean": [[[0.5]]], "count": [4]}
    metadata = {"episode_index": [0, 1]}
    for key in keys:
        for name in stats[key]:
            metadata[f"stats/{key}/{name}"] = [[0], [0]] if name == "count" else [[0.0], [0.0]]
    directory = tmp_path / "meta/episodes"
    directory.mkdir(parents=True)
    metadata_path = directory / "file.parquet"
    pq.write_table(pa.table(metadata), metadata_path)
    (tmp_path / "meta/stats.json").write_text(json.dumps(stats))

    _write_vector_statistics(tmp_path)

    assert pq.read_table(data_path).equals(data)
    actual = json.loads((tmp_path / "meta/stats.json").read_text())
    assert actual["action"]["count"] == [4]
    assert actual["observation.images.global"] == stats["observation.images.global"]
    episodes = pq.read_table(metadata_path)
    assert episodes.column("stats/action/count").to_pylist() == [[2], [2]]
    assert episodes.column("stats/action/std").to_pylist() == [[0.009765625], [0.009765625]]
