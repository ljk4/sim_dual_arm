"""Public ROS control probe. Refuses every backend except the test-only mock."""

import json
import time
import uuid
from collections import Counter
from dataclasses import replace

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from robot_api_msgs.action import ExecuteTrajectory
from robot_api_msgs.msg import BackendInfo, CommandEvent, RobotState
from robot_api_msgs.srv import AcquireControl, ReleaseControl, RenewControl, Stop
from robot_contracts.models import Target, Trajectory
from robot_ros_support.conversions import (
    command_message,
    context_from_message,
    context_message,
    duration,
)
from robot_ros_support.qos import EVENT_QOS, INFO_QOS, SENSOR_QOS


def main() -> None:
    rclpy.init()
    node = Node("backend_contract_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    root = "/cell_01/robot"
    received = {"info": None, "state": None}
    events = Counter()
    node.create_subscription(
        BackendInfo, root + "/info", lambda message: received.update(info=message), INFO_QOS
    )
    node.create_subscription(
        RobotState, root + "/state", lambda message: received.update(state=message), SENSOR_QOS
    )
    node.create_subscription(
        CommandEvent,
        root + "/command_events",
        lambda message: events.update([message.command_id]),
        EVENT_QOS,
    )

    def spin(seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)

    def wait(future, timeout: float = 5):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError("Public API probe timed out")
            executor.spin_once(timeout_sec=0.02)
        return future.result()

    clients = {
        name: node.create_client(kind, root + "/" + name)
        for name, kind in (
            ("acquire_control", AcquireControl),
            ("renew_control", RenewControl),
            ("release_control", ReleaseControl),
            ("stop", Stop),
        )
    }

    def call(name, request):
        if not clients[name].wait_for_service(timeout_sec=5):
            raise TimeoutError(name)
        return wait(clients[name].call_async(request))

    action = ActionClient(node, ExecuteTrajectory, root + "/execute_trajectory")
    try:
        deadline = time.monotonic() + 10
        while received["info"] is None or received["state"] is None:
            if time.monotonic() > deadline:
                raise TimeoutError("Backend measurements unavailable")
            executor.spin_once(timeout_sec=0.05)
        info = received["info"]
        assert info.test_only and info.backend_kind == "mock", "Probe requires test-only mock"
        context = replace(
            context_from_message(info.context),
            episode_id="probe-" + uuid.uuid4().hex,
            control_epoch=1,
        )
        rejected = call(
            "acquire_control",
            AcquireControl.Request(
                context=context_message(replace(context, clock_epoch=context.clock_epoch + 1)),
                owner_id="probe",
                duration=duration(5_000_000_000),
            ),
        )
        assert not rejected.success and rejected.reason == "STALE_CONTEXT"
        acquired = call(
            "acquire_control",
            AcquireControl.Request(
                context=context_message(context),
                owner_id="probe",
                duration=duration(5_000_000_000),
            ),
        )
        assert acquired.success
        lease = acquired.lease_id
        busy = call(
            "acquire_control",
            AcquireControl.Request(
                context=context_message(context),
                owner_id="other-runtime",
                duration=duration(5_000_000_000),
            ),
        )
        assert not busy.success and busy.reason == "CONTROL_BUSY"
        initial = tuple(received["state"].joints.position)
        target = (initial[0] + 0.2, *initial[1:])
        first = Trajectory(
            context,
            lease,
            "probe-" + uuid.uuid4().hex,
            1,
            1,
            node.get_clock().now().nanoseconds + 3_000_000_000,
            tuple(info.joint_names),
            (Target(500_000_000, target),),
        )
        assert action.wait_for_server(timeout_sec=5)

        def submit(command):
            handle = wait(
                action.send_goal_async(
                    ExecuteTrajectory.Goal(command=command_message(command)),
                )
            )
            assert handle.accepted
            return handle

        first_handle = submit(first)
        assert wait(first_handle.get_result_async()).result.success
        duplicate = submit(first)
        assert wait(duplicate.get_result_async()).result.success
        spin(0.1)
        assert events[first.command_id] == 3, "Duplicate command produced extra execution events"
        second = replace(
            first,
            command_id="probe-" + uuid.uuid4().hex,
            command_seq=2,
            targets=(Target(500_000_000, initial),),
        )
        second_handle = submit(second)
        spin(0.1)
        cancellation = wait(second_handle.cancel_goal_async())
        assert cancellation.goals_canceling
        result = wait(second_handle.get_result_async()).result
        assert not result.success and result.phase == "CANCELED"
        assert call(
            "stop",
            Stop.Request(
                context=context_message(context),
                lease_id=lease,
                reason="PROBE_STOP",
            ),
        ).success
        spin(0.1)
        held = tuple(received["state"].joints.position)
        spin(0.2)
        assert tuple(received["state"].joints.position) == held
        assert call(
            "release_control",
            ReleaseControl.Request(
                context=context_message(context),
                lease_id=lease,
            ),
        ).success
        expired = call(
            "renew_control",
            RenewControl.Request(
                context=context_message(context),
                lease_id=lease,
                duration=duration(1_000_000_000),
            ),
        )
        assert not expired.success and expired.reason == "LEASE_EXPIRED"
        print(
            json.dumps(
                {
                    "backend_api_probe": "passed",
                    "checks": [
                        "stale_context",
                        "exclusive_lease",
                        "completed_trajectory",
                        "duplicate_idempotence",
                        "action_cancel",
                        "stop_hold",
                        "released_lease_rejected",
                    ],
                }
            ),
            flush=True,
        )
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
