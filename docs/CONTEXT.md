# Simulation Library Context

## Architectural Position

The ConstructionEnterprise Simulation Library is the governed implementation-side repository for Digital Twin and industrial simulation artifacts. It is linked to the Google Docs knowledge layer through the Cross-Reference Map and related CE / FF documents.

The library has two independent but coordinated structures:

| Structure | Authority | Rule |
|---|---|---|
| Directory layout under `simulations/` | Provenance and history | Preserve each imported source-repository boundary exactly. |
| `SIMULATION_METADATA.yaml` and `simulations/INDEX.md` | Semantic classification and knowledge graph | Classify artifacts by capability, physical system, lifecycle, documentation, and validation state. |

This separation allows the enterprise to add meaning without losing lineage.

## Source of Truth Rules

| Question | Governing System |
|---|---|
| What a simulation is intended to model | Per-simulation metadata and CE Forge / Digital Twin documentation. |
| Where a simulation originated | `MIGRATION_MANIFEST.md`, metadata source lineage, and Git history. |
| Whether it is operationally or physically validated | Per-simulation validation evidence; never infer from directory presence. |
| What application contract it must respect | Factory Foundation implementation and the linked specifications. |
| What belongs in the library | The repository identity contract and approved migration decisions. |

## Explicit Non-Goals

The library is not a new semantic folder hierarchy, an implicit production-validation claim, or a replacement for source history. It is a governed library boundary that keeps source history visible and makes classification navigable.
