# Introduction

The M031 BSP SBOM package consists of two CycloneDX SBOM files:

- Product SBOM
- Test Sample SBOM

The Product SBOM includes components that are included in, linked with, deployed to, or reasonably expected to be integrated into the final product firmware/software.

The Test Sample SBOM includes components used only for samples, demonstrations, validation, or testing and not included in the product runtime unless explicitly integrated by the user.

# Product SBOM

Used for CRA compliance, product vulnerability management, and customer product integration risk assessment.

## Scope

Drivers, device support files, CMSIS components, startup code, boot code, and source components that are compiled, linked, flashed, deployed, or reasonably expected to be integrated into products.

## Evidence

.
└── Library

# Test Sample SBOM

Used for transparent disclosure and internal/customer evaluation, but labeled as non-product runtime dependency.

## Scope

Sample code, demo project, host-side test tools, utility programs, third-party libraries, evaluation projects, and engineering tools.

## Evidence

.
├── SampleCode
├── ThirdParty
└── Tool

# Verification Evidence

The current SBOMs were generated from clean source commit
`a4e2a8c8f65c6214cf388e8a65b1e1079d7f5cd0`
(`V3.07.000-70-ga4e2a8c8`). The previous formal package
`V3.07.000-60-g03e4dea8` was audited read-only as the historical baseline.

## Vulnerability Scan Findings

| Evidence | Scope | Findings | Severity | CVE / component |
|---|---|---:|---|---|
| Previous formal package | Product | 0 | None in `matches[]` | No matched entries |
| Previous formal package | Test Sample | 0 | None in `matches[]` | No matched entries |
| Current source snapshot | Product | 0 | None in `matches[]` | No matched entries |
| Current source snapshot | Test Sample | 0 | None in `matches[]` | No matched entries |

All four reports use native Grype JSON with top-level fields
`matches`, `source`, `distro`, and `descriptor`. Grype is version `0.117.0`.
The database status is schema `v6.1.9`, built
`2026-08-31T06:37:31Z`, and `valid: true`. Each source is type
`sbom-file` and names the corresponding repository-relative Product or Test
Sample SBOM target.

Zero matches is not a claim that either SBOM or the BSP is clean or free of
vulnerabilities. CycloneDX input can omit identifiers needed for matching, and
offline or stale databases further limit coverage. No Product Security or
PSIRT disposition, approval, VEX, or `not_affected` assertion is made. Any
future finding remains pending until supported by the required review evidence.

## Structured Verification

| Check | Result |
|---|---|
| CycloneDX JSON 1.6 validation | PASS, Product and Test Sample |
| Strict `sbom-checker` | PASS, 0 errors and 0 warnings for each SBOM |
| Binary physical path and SHA-256 closure | PASS, 44/44 Git-tracked `.a`, `.bin`, `.dll`, `.exe`, and `.lib` files |
| Manual component and license evidence closure | PASS |
| Manifest references, summaries, and SBOM hashes | PASS |
| Source and artifact provenance | PASS |
| Developer absolute paths | PASS, none present |
| Scratch formal package checksum closure | PASS, 67/67 payload files |
| Structured aggregate audit | PASS, `gaps: []` |
