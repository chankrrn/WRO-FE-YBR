"""
Tunables for a round, kept in a config.toml next to that round's task.

Everything the pit crew is likely to want to change between runs - speeds,
lookahead, how much room to leave a pillar - lives in TOML rather than in the
code, so a bad corner can be fixed on the field without editing Python and
without a re-read of the whole control loop to find the number.

    config = TaskConfig.load(Path(__file__).parent / "config.toml")
    speed = config.get("speed.base", 55)

Lookups are dotted and always take a default, so a config file missing a key
(or missing entirely) degrades to the built-in value instead of raising in the
middle of a run.
"""
import tomllib
from pathlib import Path


class TaskConfig:
    """Read-only nested dict with dotted lookup and CLI overrides on top."""

    def __init__(self, data=None, source=None):
        self._data = data or {}
        self.source = source
        self._overrides = {}

    # ========================================================================
    # LOADING
    # ========================================================================

    @classmethod
    def load(cls, path):
        """
        Reads a TOML file. A missing file is not an error - every caller
        passes a default to get(), so an absent config just means "all
        defaults", which is what you want when running a bare checkout.
        """
        path = Path(path)
        if not path.exists():
            print(f"WARNING: no config at {path} - using built-in defaults")
            return cls(source=None)
        with path.open("rb") as handle:
            return cls(tomllib.load(handle), source=path)

    # ========================================================================
    # READING
    # ========================================================================

    def get(self, dotted_key, default=None):
        """
        Value at "section.key", or `default` if any part of the path is
        missing. Overrides set by set() win over the file.
        """
        if dotted_key in self._overrides:
            return self._overrides[dotted_key]

        value = self._data
        for part in dotted_key.split("."):
            if not isinstance(value, dict) or part not in value:
                return default
            value = value[part]
        return value

    def set(self, dotted_key, value):
        """Applies a CLI override. Ignores None so `--speed` unset is a no-op."""
        if value is not None:
            self._overrides[dotted_key] = value
        return self

    def section(self, name):
        """A whole section as a plain dict, for splatting into a constructor."""
        value = self._data.get(name, {})
        return dict(value) if isinstance(value, dict) else {}

    def describe(self, keys):
        """One line naming the config and the keys that matter most."""
        where = self.source.name if self.source else "defaults"
        settings = "  ".join(f"{key.split('.')[-1]}={self.get(key)}" for key in keys)
        return f"[config {where}] {settings}"
