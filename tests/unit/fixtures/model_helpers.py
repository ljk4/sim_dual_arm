"""Revalidation-aware replacement for pydantic contract/config models."""

from dataclasses import replace as dataclass_replace

from robot_contracts.validation import StrictContractModel


def replace(value, **changes):
    """Replace model fields and re-run validation, like ``dataclasses.replace``."""
    if isinstance(value, StrictContractModel):
        document = value.model_dump(mode="python")
        document.update(changes)
        return type(value).validate_dict(document)
    return dataclass_replace(value, **changes)
