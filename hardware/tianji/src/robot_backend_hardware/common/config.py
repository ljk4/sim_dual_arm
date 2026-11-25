"""Strict configuration shared by hardware device components."""
from pydantic import BaseModel, ConfigDict


class DeviceSettings(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, str_strip_whitespace=True)
