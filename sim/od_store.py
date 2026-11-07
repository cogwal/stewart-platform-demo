"""Typed object-dictionary value store: EDS structure + power-on defaults (EDS, then the manual's
overlay). Holds Python values (int, str, bytes); encodes/decodes with the EDS data types."""
from __future__ import annotations

from typing import Iterator

import canopen
from canopen.objectdictionary import ODVariable
from canopen.objectdictionary import datatypes as dt

Key = tuple[int, int]


class ObjectStore:
    def __init__(self, od: canopen.ObjectDictionary, overlay: dict[Key, int]):
        self.od = od
        self.overlay = overlay
        self._values: dict[Key, object] = {}
        self.reset()

    # --- structure --------------------------------------------------------------------------
    def keys(self) -> Iterator[Key]:
        for index in self.od:
            obj = self.od[index]
            if isinstance(obj, ODVariable):
                yield (index, 0)
            else:
                for sub in obj:
                    yield (index, sub)

    def var(self, index: int, sub: int = 0) -> ODVariable:
        obj = self.od[index]
        return obj if isinstance(obj, ODVariable) else obj[sub]

    def has(self, index: int, sub: int = 0) -> bool:
        try:
            self.var(index, sub)
            return True
        except KeyError:
            return False

    # --- values -----------------------------------------------------------------------------
    def _default(self, key: Key) -> object:
        if key in self.overlay:
            return self.overlay[key]
        var = self.var(*key)
        for candidate in (var.value, var.default):
            if candidate is not None:
                if var.data_type in dt.SIGNED_TYPES and isinstance(candidate, int):
                    bits = len(var)
                    if candidate >= 1 << (bits - 1):  # EDS hex for negatives, e.g. 0xFD
                        candidate -= 1 << bits
                return candidate
        if var.data_type in dt.NUMBER_TYPES:
            return 0
        return b"" if var.data_type == dt.DOMAIN else ""

    def reset(self, comm_only: bool = False) -> None:
        """Power-on (all) or NMT reset communication (1000h..1FFFh only)."""
        for key in self.keys():
            if comm_only and not 0x1000 <= key[0] <= 0x1FFF:
                continue
            self._values[key] = self._default(key)

    def get(self, index: int, sub: int = 0):
        return self._values[(index, sub)]

    def set(self, index: int, sub: int, value) -> None:
        if (index, sub) not in self._values:
            raise KeyError(f"{index:04X}h.{sub} not in the object dictionary")
        self._values[(index, sub)] = value

    def encode(self, index: int, sub: int, value=None) -> bytes:
        var = self.var(index, sub)
        return var.encode_raw(self.get(index, sub) if value is None else value)

    def decode(self, index: int, sub: int, data: bytes):
        return self.var(index, sub).decode_raw(data)

    def bits(self, index: int, sub: int = 0) -> int:
        return len(self.var(index, sub))
