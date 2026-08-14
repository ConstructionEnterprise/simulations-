# ConstructionEnterprise Simulation Library

> The canonical, history-preserving library for ConstructionEnterprise Digital Twin and industrial-simulation artifacts.

| Field | Value |
|---|---|
| Status | `implemented` |
| Classification | `simulation-library` |
| Enterprise owner | Construction Enterprises |
| Technical owner | Simulation Engineering |
| Knowledge owner | Construction Enterprises / Knowledge Systems |
| Last reviewed | 2026-08-14 |
| Canonical repository | `ConstructionEnterprise/simulations-` |

## Enterprise Context

ConstructionEnterprise uses Digital Twin and industrial-simulation assets to de-risk physical manufacturing deployment. This repository is the library boundary for those assets and connects robot, gantry, rail, jig-frame, module-assembly, and integrated-cell models to the CE Forge and Factory Foundation knowledge / implementation layers.

## Purpose and Provenance

This repository is the single active home for twelve imported simulation directories. Each directory retains its original source-repository boundary and commit history through the consolidation process recorded in [MIGRATION_MANIFEST.md](MIGRATION_MANIFEST.md). The original repositories are archived, not deleted, to retain rollback, issue, release, URL, and lineage evidence.

> **Physical repository structure is provenance. Metadata and the knowledge graph provide semantic classification.** The library must not be reorganized into new semantic folders without an approved migration plan that preserves the source boundaries and updates the manifest.

## Library Inventory

The governed inventory is maintained in [simulations/INDEX.md](simulations/INDEX.md). Every simulation directory must contain a `SIMULATION_METADATA.yaml` identity record. The metadata classifies the artifact without moving or altering its inherited source structure.

## Canonical Documentation

- [CE: Forge](https://docs.google.com/document/d/18sLXbC8O3SypYwFSUUv4oMSs8FjOs07wlSUCB7x_w8c/edit)
- [CE: Executive Summary](https://docs.google.com/document/d/1RtnZwt_dbqLR2FSsb2gNXslKz5k4499IWgqpSfPUIuw/edit)
- [FF: Repository Blueprint](https://docs.google.com/document/d/1w8UruwLZ8HybEaF0emZH9ge2TO3wqjsdGmJx5WrlupA/edit)
- [FF: Architecture Index](https://docs.google.com/document/d/1B_hQCchV9sjWfgw0uo-K3JZjPQPScBYEyJm_PiqxCAg/edit)
- [FF: Engineering Standards](https://docs.google.com/document/d/1YTmPNKLguXr3CIkc8joR8AW17SR0BsjAU_XSFERkmD8/edit)
- [FF: Google Docs ↔ GitHub Cross-Reference Map](https://docs.google.com/document/d/1erFSQrIO1vSJbduxgOCP_0Whu16MVhV-K6-uyBA7amE/edit)

## Repository Layout

```text
.
├── README.md                       # Human-readable repository gateway
├── REPOSITORY_METADATA.yaml        # Machine-readable repository identity
├── MIGRATION_MANIFEST.md           # Source lineage and migration evidence
├── docs/
│   ├── CONTEXT.md                  # Architectural and governance context
│   └── SIMULATION_LIBRARY_GOVERNANCE.md
└── simulations/
    ├── INDEX.md                    # Governed inventory
    └── <source-boundary>/
        └── SIMULATION_METADATA.yaml
```

## Related Systems

| System | Relationship |
|---|---|
| [`CE_Forge`](https://github.com/ConstructionEnterprise/CE_Forge) | Defines enterprise-simulation, scenario, and validation principles. |
| [`Factory_To_Foundation`](https://github.com/ConstructionEnterprise/Factory_To_Foundation) | Provides application and runtime contracts that simulations must respect. |
| [`Construction_Enterprises`](https://github.com/ConstructionEnterprise/Construction_Enterprises) | Contains Digital Twin planning and physical-enterprise context. |

## Contribution and Verification

A change to a simulation is incomplete until it updates the affected `SIMULATION_METADATA.yaml`, identifies the governing Docs links, records its Factory Foundation / Forge contract relationship, and supplies validation evidence or a clear statement of unverified limitations. Local execution alone does not establish physical-system validation.

## Governance

Repository identity is defined in [REPOSITORY_METADATA.yaml](REPOSITORY_METADATA.yaml). The operational policies for the library and per-simulation metadata are defined in [docs/SIMULATION_LIBRARY_GOVERNANCE.md](docs/SIMULATION_LIBRARY_GOVERNANCE.md). Future automation should validate metadata presence, Docs links, lifecycle status, review dates, and inventory coverage without moving simulation directories.
