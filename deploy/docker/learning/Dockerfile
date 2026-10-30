FROM python:3.12-slim-bookworm AS protocol
WORKDIR /workspace
COPY deploy/docker/requirements-dev.txt /tmp/requirements-dev.txt
RUN pip install --no-cache-dir -r /tmp/requirements-dev.txt
COPY contracts/policy /workspace/contracts/policy
COPY tools/contracts/generate_policy_sdk.py /workspace/tools/contracts/generate_policy_sdk.py
RUN python tools/contracts/generate_policy_sdk.py --output /tmp/policy-sdk \
    && pip install --no-build-isolation --no-deps /tmp/policy-sdk \
    && python -c "from dual_arm_policy.v1 import policy_pb2, policy_pb2_grpc"
CMD ["python", "-c", "from dual_arm_policy.v1 import policy_pb2; print(policy_pb2.DESCRIPTOR.package)"]
