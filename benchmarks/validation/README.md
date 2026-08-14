# Validation Evidence Boundary

This directory is reserved for **approved, public-safe validation summaries**. It does not yet contain simulation performance claims or test results.

Before adding a validation record, include:

```yaml
record_id: CE-VAL-<identifier>
status: reference | validated | superseded
scope: What was tested.
method: How it was tested.
environment: Relevant non-sensitive runtime context.
result: Pass / fail / observation.
limitations: What the result does not prove.
source_lineage: Repository path and commit or release.
reviewed_by: Technical review role.
reviewed_on: YYYY-MM-DD
publication_classification: public-summary
```

Do not publish raw configurations, sensitive facility information, personal data, secrets, unapproved vendor information, or benchmark numbers that have not completed technical and business-owner review.
