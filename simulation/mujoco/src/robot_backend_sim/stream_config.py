"""Strict immutable configuration for bounded continuous chunk streams."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

from pydantic import Field, model_validator
from robot_contracts.validation import (
    NonNegativeFloat,
    PositiveFloat,
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    contract_error,
    read_unique_yaml,
    validate_config_model,
)

STREAM_CONFIG_FIELDS = (
    "initial_lead_s",
    "min_lead_s",
    "max_observation_age_s",
    "stream_wall_timeout_s",
    "progress_wall_timeout_s",
    "tracking_timeout_s",
)
_MAX_NS = 2**63 - 1


def _nanoseconds(seconds: float) -> int:
    try:
        return round(seconds * 1_000_000_000)
    except (OverflowError, ValueError):
        return _MAX_NS + 1


StreamConfigError = StrictMappingError

class StreamConfig(StrictMappingModel):

    initial_lead_s: PositiveFloat
    min_lead_s: PositiveFloat
    max_observation_age_s: PositiveFloat
    stream_wall_timeout_s: PositiveFloat
    progress_wall_timeout_s: PositiveFloat
    tracking_timeout_s: PositiveFloat
    contact_unload_timeout_s: Annotated[NonNegativeFloat, Field(le=2.0)] = 0.0

    @model_validator(mode="after")
    def _relationships(self) -> StreamConfig:
        if self.initial_lead_s < self.min_lead_s:
            raise contract_error("initial_lead_s", "must be at least min_lead_s")
        for field in STREAM_CONFIG_FIELDS:
            value = _nanoseconds(getattr(self, field))
            if value <= 0 or value > _MAX_NS:
                raise contract_error(field, "must convert to a positive int64 nanosecond duration")
        return self

    def nanoseconds(self, field: str) -> int:
        if field not in STREAM_CONFIG_FIELDS:
            raise StreamConfigError("<stream config>", field, "unknown field")
        value = _nanoseconds(getattr(self, field))
        if value <= 0 or value > _MAX_NS:
            raise StreamConfigError("<stream config>", field,
                                    "must convert to a positive int64 nanosecond duration")
        return value

    initial_lead_ns = property(lambda self: self.nanoseconds("initial_lead_s"))
    min_lead_ns = property(lambda self: self.nanoseconds("min_lead_s"))
    max_observation_age_ns = property(lambda self: self.nanoseconds("max_observation_age_s"))
    stream_wall_timeout_ns = property(lambda self: self.nanoseconds("stream_wall_timeout_s"))
    progress_wall_timeout_ns = property(
        lambda self: self.nanoseconds("progress_wall_timeout_s"))
    tracking_timeout_ns = property(lambda self: self.nanoseconds("tracking_timeout_s"))

    def to_document(self) -> dict[str, float]:
        return {field: float(getattr(self, field)) for field in STREAM_CONFIG_FIELDS} | (
            {"contact_unload_timeout_s": self.contact_unload_timeout_s}
            if self.contact_unload_timeout_s else {})


def parse_stream_config(
    document: object, source: str | Path = "<stream config>"
) -> StreamConfig:
    return validate_config_model(StreamConfig, document, source, StreamConfigError)


def load_stream_config(path: Path | str) -> StreamConfig:
    source = Path(path)
    try:
        document = read_unique_yaml(source)
    except UniqueKeyError as error:
        raise StreamConfigError(error.source, error.field, error.message) from error
    return parse_stream_config(document, source)
