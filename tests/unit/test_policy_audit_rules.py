import pytest
from dual_arm_runtime.policy_audit_rules import require_completed_chunks
from robot_contracts.models import ContractError


def test_completed_policy_requires_exactly_configured_chunks():
    require_completed_chunks(completed=True, policy_chunks=8, configured_chunks=8)
    with pytest.raises(ContractError, match="7 chunks, configured 8"):
        require_completed_chunks(completed=True, policy_chunks=7, configured_chunks=8)
    with pytest.raises(ContractError, match="9 chunks, configured 8"):
        require_completed_chunks(completed=True, policy_chunks=9, configured_chunks=8)
    # A legal abort may retain fewer accepted blocks.
    require_completed_chunks(completed=False, policy_chunks=3, configured_chunks=8)
