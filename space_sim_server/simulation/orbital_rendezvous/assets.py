"""Bind the imported team assets after moving/cloning the server checkout."""

import json
from pathlib import Path


def bind_team_catalog(catalog, repository, manifest, output):
    """Match repository-relative names AND content hashes, never basename alone.

    Imported UE asset names remain unchanged. Only local source-mesh keys change.
    Original catalogs are not modified; missing or stale imports fail before flight.
    """
    payload = json.loads(Path(catalog).read_text(encoding="utf-8-sig"))
    assets = payload.get("assets", {})
    rebound = {}
    for relative, digest in manifest["meshes"].items():
        suffix = "/" + relative.replace("\\", "/")
        candidates = [entry for key, entry in assets.items()
                      if ("/" + key.replace("\\", "/").lstrip("/")).endswith(suffix)
                      and isinstance(entry, dict) and entry.get("source_sha256") == digest]
        # Reject multiple conflicting entries rather than silently choosing one.
        unique = {json.dumps(entry, sort_keys=True): entry for entry in candidates}
        if len(unique) != 1:
            raise ValueError(f"Missing, stale or ambiguous team render asset: {relative}")
        entry = next(iter(unique.values()))
        if not entry.get("asset_path", "").startswith("/Game/"):
            raise ValueError(f"Invalid UE asset reference: {relative}")
        rebound[str((Path(repository) / relative).resolve())] = entry
    result = {**payload, "assets": rebound,
              "source_mjcf": str((Path(repository) / manifest["source_model"]).resolve())}
    path = Path(output) / "team_render.catalog.json"
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return path
