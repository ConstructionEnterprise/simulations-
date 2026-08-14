# Dataset Boundary and Data Dictionary Policy

This directory is reserved for approved, structured engineering datasets and their data dictionaries. It intentionally contains no operational datasets at this stage.

## Admission Rules

A dataset may be added only when it has:

1. A declared owner and source lineage.
2. A data dictionary and schema version.
3. A publication classification.
4. A retention / review date.
5. Confirmation that it does not contain secrets, personal data, customer data, active facility-security information, confidential vendor information, or raw production telemetry.

## Public Dataset Record Template

```yaml
dataset_id: CE-DATA-<identifier>
schema_version: 1
name: <approved public name>
classification: public-engineering-record
source_lineage: <repository or approved source>
fields: <link to data dictionary>
limitations: <known constraints>
last_reviewed: YYYY-MM-DD
owner: <role>
```
