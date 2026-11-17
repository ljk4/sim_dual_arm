from pathlib import Path

import pytest
from dual_arm_learning import training, training_support
from dual_arm_learning.config import load_training_config

torch = pytest.importorskip("torch")
ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "shared/scenarios/panda_sim/stack"


class _Policy:
    def __init__(self):
        self.training = True
        self.history = []
        self.seen_keys = []

    def eval(self):
        self.history.append("eval")
        self.training = False
        return self

    def train(self, mode=True):
        self.history.append(f"train:{mode}")
        self.training = bool(mode)
        return self

    def predict_action_chunk(self, batch):
        self.seen_keys.append(set(batch))
        return torch.zeros((1, 60, 16), dtype=torch.float32)


def test_validation_restores_mode_and_never_passes_labels_to_model(monkeypatch):
    config = load_training_config(SCENARIO / "configs/learning/act_training.yaml", ROOT)
    batch = {
        "observation.state": torch.zeros((1, 16)),
        "action": torch.zeros((1, 60, 16)),
        "action_is_pad": torch.zeros((1, 60), dtype=torch.bool),
    }
    monkeypatch.setattr(training_support, "loader", lambda *args, **kwargs: [batch])
    monkeypatch.setattr(training_support, "process_batch", lambda pre, item, keys: item)

    class _Dataset:
        def __len__(self):
            return 1

    policy = _Policy()
    result = training._validation(policy, None, _Dataset(), config, max_batches=1)
    assert result["val_l1"] == 0.0
    assert policy.training is True
    assert policy.history == ["eval", "train:True"]
    assert policy.seen_keys == [{"observation.state"}]
