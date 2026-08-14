# Simulation Repository Consolidation Manifest

## Purpose

This manifest records the non-destructive consolidation of selected ConstructionEnterprise simulation repositories into the `ConstructionEnterprise/simulations-` repository.

The source repositories were **not modified, archived, renamed, or deleted**. Their contents were imported beneath `simulations/` on the dedicated migration branch `consolidate-simulation-repositories-2026-08-13`.

Each import was performed with `git subtree add` **without squashing history**. The imported directories therefore retain access to their source commit histories through the migration commits.

## Imported Repositories

| Source Repository | Source Branch | Target Directory | Visibility at Inventory | Source Last Push (UTC) |
|---|---|---|---|---|
| `ConstructionEnterprise/CE_Intergrated_Cell_V1_3` | `main` | `simulations/CE_Intergrated_Cell_V1_3/` | Private | 2026-07-02 08:59:19 |
| `ConstructionEnterprise/CR6_6Axis` | `main` | `simulations/CR6_6Axis/` | Private | 2026-06-04 05:24:35 |
| `ConstructionEnterprise/CR6_6axis_Object_Tracking` | `main` | `simulations/CR6_6axis_Object_Tracking/` | Private | 2026-06-06 07:52:51 |
| `ConstructionEnterprise/CR6_V08_Dual_Robot_Cell` | `main` | `simulations/CR6_V08_Dual_Robot_Cell/` | Public | 2026-06-07 04:40:53 |
| `ConstructionEnterprise/Dual_CR6_Cell` | `main` | `simulations/Dual_CR6_Cell/` | Private | 2026-06-04 07:27:42 |
| `ConstructionEnterprise/Dual_Robot_Cell_Jig_Frame` | `main` | `simulations/Dual_Robot_Cell_Jig_Frame/` | Private | 2026-06-06 08:02:01 |
| `ConstructionEnterprise/Dual_Robot_Jig_Frame_V1_1` | `main` | `simulations/Dual_Robot_Jig_Frame_V1_1/` | Public | 2026-06-07 04:42:24 |
| `ConstructionEnterprise/Factory_Rail_v2` | `main` | `simulations/Factory_Rail_v2/` | Public | 2026-06-07 04:46:27 |
| `ConstructionEnterprise/Integrated-_Cell_V2_6-` | `main` | `simulations/Integrated-_Cell_V2_6-/` | Public | 2026-06-07 10:55:14 |
| `ConstructionEnterprise/Module_Assembly_V1` | `main` | `simulations/Module_Assembly_V1/` | Public | 2026-06-07 10:23:43 |
| `ConstructionEnterprise/Overhead_Gantry_V1` | `main` | `simulations/Overhead_Gantry_V1/` | Public | 2026-06-07 10:58:39 |
| `ConstructionEnterprise/Rail_System_V1` | `main` | `simulations/Rail_System_V1/` | Public | 2026-06-07 11:00:55 |

## Validation

The migration branch contains all **12** requested source repositories in separate target directories and includes **12** discrete import commits. The working tree was clean immediately before this manifest was added.

## Follow-Up Guidance

The migration branch is intentionally isolated from `main` for review. After review, merge the branch through the normal repository workflow. Source repositories should remain intact until the consolidated repository has been verified in the intended execution environment and any external references have been updated.
