"""Control delivery facts that affect bounded runtime cleanup."""

from robot_contracts.models import ContractError


class FirstFollowerRejected(ContractError):
    """An explicit first negative ACK with no accepted execution to await."""
