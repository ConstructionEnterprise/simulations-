# Digital Twin Architecture — Public-Safe Overview

## Purpose

ConstructionEnterprise uses Digital Twin and industrial-simulation artifacts to represent, reason about, and validate aspects of manufacturing systems before physical deployment. The canonical Simulation Library preserves the engineering lineage of simulation artifacts and connects their governed metadata to the wider Factory Foundation and CE Forge knowledge system.

## System Relationship

```text
ConstructionEnterprise physical-enterprise concepts
        ↓
Digital Twin and simulation artifacts
        ↓
CE Forge scenario / validation model
        ↓
Factory Foundation application contracts
        ↓
Governed engineering records and review evidence
```

This repository is a library and provenance boundary. It does not expose live operational infrastructure, facility security details, raw production telemetry, customer data, or confidential configuration.

## Source of Truth

- [CE: Forge](https://docs.google.com/document/d/18sLXbC8O3SypYwFSUUv4oMSs8FjOs07wlSUCB7x_w8c/edit)
- [FF: Repository Blueprint](https://docs.google.com/document/d/1w8UruwLZ8HybEaF0emZH9ge2TO3wqjsdGmJx5WrlupA/edit)
- [FF: Google Docs ↔ GitHub Cross-Reference Map](https://docs.google.com/document/d/1erFSQrIO1vSJbduxgOCP_0Whu16MVhV-K6-uyBA7amE/edit)

## Publication Boundary

Technical claims are published only when their scope, evidence, limitations, reviewer, and date are declared. Until then, artifacts retain `reference` or `not-assessed` status in their metadata.
