"""Disabling previews must preserve the recorded RGB and its hashes."""

import hashlib

import pytest

rosbag2_py = pytest.importorskip("rosbag2_py")
pytest.importorskip("rclpy")


from dual_arm_runtime.offline.export.export_media import write_images  # noqa: E402
from PIL import Image as PillowImage  # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402
from robot_api_msgs.msg import CameraFrame  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402


@pytest.mark.parametrize("previews", [False, True])
def test_previews_are_optional_but_rgb_is_always_exported(tmp_path, previews):
    recording = tmp_path / "recording"
    raw = bytes([10, 20, 30, 40, 50, 60])
    message = CameraFrame(
        frame_seq=1,
        image=Image(width=2, height=1, encoding="rgb8", step=6, data=raw),
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(recording), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    topic = "/robot/cameras/global/frame"
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name=topic, type="robot_api_msgs/msg/CameraFrame", serialization_format="cdr"
    ))
    writer.write(topic, serialize_message(message), 1)
    del writer
    out = tmp_path / "export"
    mapping = write_images(
        recording, out, ("global",),
        [{"observation_id": 1, "observation_backend_time_ns": 1}],
        {1: (1,)}, {"stable": 1}, {"global": (2, 1)}, previews=previews,
    )
    png = out / mapping[0]["global"]["path"]
    assert PillowImage.open(png).tobytes() == raw
    assert mapping[0]["global"]["rgb_sha256"] == hashlib.sha256(raw).hexdigest()
    assert mapping[0]["global"]["png_sha256"] == hashlib.sha256(png.read_bytes()).hexdigest()
    assert (out / "video/global.gif").exists() is previews
    assert (out / "preview/stable_global.png").exists() is previews
