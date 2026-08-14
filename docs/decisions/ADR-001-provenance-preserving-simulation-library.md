# ADR-001: Preserve Imported Simulation Boundaries as Provenance

**Status:** Accepted
**Date:** 2026-08-14
**Decision owner:** Construction Enterprises / Knowledge Systems

## Context

Twelve standalone simulation repositories were consolidated into `ConstructionEnterprise/simulations-`. The library needs semantic classification and public engineering records, but reorganizing the imported directories into a new topical hierarchy would conceal source lineage and complicate historical review.

## Decision

The direct children of `simulations/` remain aligned to the imported source-repository boundaries. Semantic classification is added through `SIMULATION_METADATA.yaml`, `simulations/INDEX.md`, and machine-readable manifests rather than a replacement directory structure.

## Consequences

### Positive

- Original repository lineage remains inspectable.
- Git history and migration evidence remain easier to audit.
- Semantic classification can evolve without destructive file moves.
- A future knowledge graph can consume metadata without rewriting source layout.

### Constraints

- Directory names may not be the ideal long-term semantic taxonomy.
- New taxonomy views must be generated from metadata, not imposed by relocating artifacts.
- Any future physical reorganization requires a separate approved migration decision and updated provenance records.
