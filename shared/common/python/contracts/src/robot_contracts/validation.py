from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Annotated, Any, ClassVar, Self

import yaml
from pydantic import BaseModel, BeforeValidator, ConfigDict, Field, ValidationError
from pydantic_core import PydanticCustomError

SHA256_PATTERN = r"^[0-9a-f]{64}$"
SAFE_ID_PATTERN = r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$"


class ContractError(ValueError):
    def __init__(self, reason: str, detail: str = "") -> None:
        self.reason = reason
        super().__init__(f"{reason}: {detail}" if detail else reason)


class StrictMappingError(ValueError):
    def __init__(self, source_or_field: str, field_or_message: str,
                 message: str | None = None) -> None:
        if message is None:
            source, field, detail = "", source_or_field, field_or_message
        else:
            source, field, detail = str(source_or_field), field_or_message, message
        self.source, self.field, self.message = source, field, detail
        super().__init__(f"{source}: {field}: {detail}" if source else f"{field}: {detail}")


class UniqueKeyError(ValueError):
    def __init__(self, source: Path | str, field: str, message: str) -> None:
        self.source, self.field, self.message = str(source), field, message
        super().__init__(f"{self.source}: {field}: {message}")


def _int_to_float(value: object) -> object:
    if isinstance(value, int) and not isinstance(value, bool):
        try:
            return float(value)
        except OverflowError:
            return value
    return value


def as_tuple(value: object) -> object:
    return tuple(value) if isinstance(value, list) else value


StrictFloat = Annotated[float, BeforeValidator(_int_to_float),
                        Field(strict=True, allow_inf_nan=False)]
PositiveFloat = Annotated[StrictFloat, Field(gt=0.0)]
NonNegativeFloat = Annotated[StrictFloat, Field(ge=0.0)]
PositiveInt = Annotated[int, Field(strict=True, gt=0)]
NonNegativeInt = Annotated[int, Field(strict=True, ge=0)]
NonEmptyStr = Annotated[str, Field(strict=True, min_length=1)]
SafeIdStr = Annotated[str, Field(strict=True, min_length=1, max_length=128,
                                 pattern=SAFE_ID_PATTERN)]
Sha256HexStr = Annotated[str, Field(strict=True, pattern=SHA256_PATTERN)]
_STRICT = dict(frozen=True, extra="forbid", strict=True, allow_inf_nan=False,
               validate_default=True, populate_by_name=True)


def contract_error(field: str, message: str, reason: str = "SCHEMA_MISMATCH") -> Exception:
    return PydanticCustomError("contract", "{message}",
                               {"reason": reason, "field": field, "message": message})


def _loc(location: tuple[Any, ...]) -> str:
    return ".".join(f"[{part}]" if isinstance(part, int) else str(part)
                    for part in location).replace(".[", "[")


def _error_fields(error: ValidationError) -> tuple[str, str, str]:
    first, context = error.errors()[0], error.errors()[0].get("ctx") or {}
    field = str(context.get("field") or _loc(first["loc"]) or "<root>")
    return (str(context.get("reason") or ""), field,
            str(context.get("message") or first["msg"]))


class StrictContractModel(BaseModel):
    model_config = ConfigDict(**_STRICT)
    _error_reason: ClassVar[str] = "SCHEMA_MISMATCH"

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        for name, value in zip(type(self).model_fields, args, strict=False):
            kwargs.setdefault(name, value)
        try:
            BaseModel.__init__(self, **kwargs)
        except ValidationError as error:
            raise self._convert_error(error) from error

    @classmethod
    def _convert_error(cls, error: ValidationError) -> Exception:
        first = error.errors()[0]
        nested = (first.get("ctx") or {}).get("error")
        if isinstance(nested, ContractError):
            prefix = _loc(first["loc"])
            detail = str(nested).removeprefix(f"{nested.reason}: ")
            return ContractError(nested.reason, f"{prefix}: {detail}" if prefix else detail)
        custom, field, message = _error_fields(error)
        if field != "<root>" and not message.startswith(f"{field}:"):
            message = f"{field}: {message}"
        reason = custom or cls._error_reason
        if reason == "SCHEMA_MISMATCH" and cls._error_reason != "SCHEMA_MISMATCH":
            reason = cls._error_reason
        return ContractError(reason, message)

    @classmethod
    def validate_dict(cls, document: dict[str, Any], *, source: str = "<contract>") -> Self:
        if isinstance(document, cls):
            return document
        try:
            return cls(**document)
        except (ContractError, ValidationError) as error:
            failure = error if isinstance(error, ContractError) else cls._convert_error(error)
            detail = str(failure).removeprefix(f"{failure.reason}: ")
            raise ContractError(failure.reason, f"{source}: {detail}") from error

    def canonical_dict(self) -> dict[str, Any]:
        return json.loads(json.dumps(self.model_dump(mode="json"), sort_keys=True,
                                     separators=(",", ":"), allow_nan=False))

    def canonical_sha256(self) -> str:
        return hashlib.sha256(json.dumps(
            self.canonical_dict(), sort_keys=True, separators=(",", ":")).encode()).hexdigest()


class StrictMappingModel(StrictContractModel):
    model_config = ConfigDict(**_STRICT, arbitrary_types_allowed=True)
    _config_error: ClassVar[type[ValueError]] = StrictMappingError
    _source_default: ClassVar[str] = "<config>"

    @classmethod
    def _convert_error(cls, error: ValidationError) -> Exception:
        first = error.errors()[0]
        nested = (first.get("ctx") or {}).get("error")
        if hasattr(nested, "field") and hasattr(nested, "message"):
            prefix = _loc(first["loc"])
            field = f"{prefix}.{nested.field}" if prefix else nested.field
            return cls._config_error(cls._source_default, field, nested.message)
        _reason, field, message = _error_fields(error)
        return cls._config_error(cls._source_default, field, message)


def validate_config_model(model_cls: type[StrictMappingModel], document: object,
                          source: Path | str, error_cls: type[ValueError]) -> Any:
    if not isinstance(document, dict):
        raise error_cls(source, "<root>", "must be a mapping")
    try:
        return model_cls(**document)
    except error_cls as error:
        raise error_cls(source, error.field, error.message) from error
    except ValidationError as error:
        _reason, field, message = _error_fields(error)
        raise error_cls(source, field, message) from error


class UniqueKeyYamlLoader(yaml.SafeLoader):
    source_name = "<document>"


def _construct_unique_mapping(loader: UniqueKeyYamlLoader, node: yaml.MappingNode,
                              deep: bool = False) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    mapping: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            if key in mapping:
                raise UniqueKeyError(loader.source_name, str(key), "duplicate key")
        except TypeError as error:
            raise UniqueKeyError(loader.source_name, "<mapping>",
                                 "keys must be hashable") from error
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyYamlLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping)


def parse_unique_yaml(text: str, source: Path | str = "<document>") -> Any:
    loader = UniqueKeyYamlLoader(text)
    loader.source_name = str(source)
    try:
        return loader.get_single_data()
    except yaml.YAMLError as error:
        raise UniqueKeyError(source, "<yaml>", f"invalid YAML: {error}") from error


def read_unique_yaml(path: Path | str) -> Any:
    source = Path(path)
    try:
        return parse_unique_yaml(source.read_text(encoding="utf-8"), source)
    except (OSError, UnicodeError) as error:
        raise UniqueKeyError(source, "<file>", str(error)) from error
