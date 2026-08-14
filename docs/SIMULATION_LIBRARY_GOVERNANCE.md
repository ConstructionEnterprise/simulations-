# Simulation Library Governance

## Policy

Every imported simulation is a governed enterprise artifact. Governance must preserve source provenance while making the artifact discoverable, attributable, and reviewable.

## Required Per-Simulation Metadata

Every immediate directory beneath `simulations/` contains `SIMULATION_METADATA.yaml`. The file records source lineage, semantic category, lifecycle, related documentation, runtime status, interfaces, and validation evidence.

| Field | Rule |
|---|---|
| `source_lineage` | Must name the archived source repository and the imported directory. |
| `classification` | Must describe semantics without changing the inherited path. |
| `lifecycle_status` | Must be explicit; `reference` means preserved but not currently verified. |
| `verification` | Must state an evidence path or `not-assessed`; absence must never imply validation. |
| `documentation` | Must link to at least CE Forge, CE Executive Summary, and the repository Cross-Reference Map. |
| `interfaces` | Must document known Factory Foundation / Forge interactions or state that they are not yet assessed. |

## Review Cadence

Repository identity is reviewed every 30 days. Per-simulation metadata is reviewed whenever the simulation changes and at least quarterly. A metadata-only review does not certify runtime or physical-system correctness.

## Change Rules

1. Do not relocate simulation directories for taxonomy purposes.
2. Do not delete the migration manifest or source-lineage fields.
3. Add documentation and metadata in the same work item as a material simulation change.
4. Use `deprecated` or `archived` lifecycle status before removing a library artifact.
5. Link an approved successor when replacing an artifact.

## Future Automation

A repository workflow should validate inventory coverage, metadata presence, required source-lineage and documentation fields, valid lifecycle values, and review-date drift. It should report classification changes for review but must not automatically move or rewrite simulation artifacts.
