# Engineering Record Methodology

## What Is an Engineering Record?

An engineering record is a durable, reviewable artifact that explains what changed, why it changed, what evidence supports it, what limitations remain, and which sources govern it. It may be a migration manifest, Architecture Decision Record, structured simulation summary, benchmark report, validation report, release note, or data dictionary.

## Required Record Fields

| Field | Requirement |
|---|---|
| Identifier | Stable record ID or version. |
| Scope | What the record covers and what it does not cover. |
| Source lineage | Repository, path, commit, or governed metadata reference. |
| Status | `reference`, `in-development`, `validated`, `deprecated`, or `archived`. |
| Evidence | Test, review, result, or explicit `not-assessed` statement. |
| Limitations | Known gaps, assumptions, and exclusions. |
| Ownership | Technical and business owner. |
| Review date | When the record was last reviewed. |
| Publication classification | Public engineering record, public summary, internal, or restricted. |

## Claim Discipline

A record must not claim that a simulation is validated, production-ready, safe, accurate, or representative of a physical system unless the validation method, scope, outcome, limitations, reviewer, and date are recorded. A successful local execution is not, by itself, physical-system validation.

## Public-Safe Evidence

Public evidence may include approved methodology, sanitized aggregate results, pass / fail criteria, known limitations, and release notes. It must exclude secrets, personal information, unapproved vendor details, facility security information, raw internal configuration, and proprietary operational data.
