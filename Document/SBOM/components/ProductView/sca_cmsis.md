# CMSIS SCA Evidence

## Component Summary

- **Component:** CMSIS
- **Type:** Library
- **Supplier:** Arm Limited
- **Origin:** Third-party
- **SBOM view:** ProductView
- **Integration:** Source
- **Runtime:** Embedded firmware
- **Included in product firmware:** Yes

## Repository Baseline

- **Repository:** `M031/bsp`
- **Branch:** `master`
- **Revision:** `51ffb9b7e71511e58b705178b2d7acf34de99de4`
- **Origin master:** `51ffb9b7e71511e58b705178b2d7acf34de99de4`
- **Git describe:** `V3.07.000-46-g51ffb9b`
- **Commit date:** `2026-08-18T17:34:09+08:00`
- **Revision status:** `merged`

## Source Scope

- `Library/CMSIS/`

Top-level directories:

- `Library/CMSIS/Core/`
- `Library/CMSIS/Documentation/`
- `Library/CMSIS/Driver/`
- `Library/CMSIS/RTOS2/`

- **Total files:** 2188

## Version Evidence

- `Library/CMSIS/Core/Include/cmsis_version.h`

Confirmed values:

- `__CM_CMSIS_VERSION_MAIN = 6`
- `__CM_CMSIS_VERSION_SUB = 1`
- `__CA_CMSIS_VERSION_MAIN = 6`
- `__CA_CMSIS_VERSION_SUB = 1`

- **CMSIS-Core version:** `6.1`
- **Status:** Confirmed

Version 6.1 is confirmed for CMSIS-Core. Independent aggregate
versions for CMSIS-Driver and CMSIS-RTOS2 were not identified.

## License Evidence

- **SPDX expression:** `Apache-2.0`
- **Evidence:** Source-file SPDX headers
- **Standalone LICENSE under Library/CMSIS:** Not found

Example evidence paths:

- `Library/CMSIS/Core/Include/cmsis_version.h`
- `Library/CMSIS/Core/Include/cmsis_compiler.h`
- `Library/CMSIS/Driver/Include/Driver_Common.h`

## File Manifest

- **Generated path:** `/tmp/m031_cmsis_file_manifest.txt`
- **Record format:** `sha256|size|repository-relative-path`
- **Record count:** 2188
- **Manifest SHA-256:** `2552db62954f51ee0786c2d08aa0efcd7648cedbfb200304b3138632a258f663`
- **Validation:** Passed
- **Tracked by Git:** No

The manifest must be regenerated when the formal BSP revision or
the Library/CMSIS content changes.

## ProductView Decision

CMSIS is assigned to ProductView because CMSIS-Core and related
interfaces are product-level dependencies of the M031 BSP.

The following directories remain part of the top-level M031 BSP:

- `Library/Device/Nuvoton/M031/`
- `Library/StdDriver/`

No separate sca_m031_driver component is created.

## Analysis Status

- **SCA evidence collection:** Completed
- **CMSIS-Core version identification:** Completed
- **License identification:** Completed
- **SAST:** Not performed in this evidence step
- **Binary analysis:** Not required
- **Vulnerability analysis:** Pending

## Unresolved Items

1. Determine whether CMSIS-Driver requires a separate SCA record.
2. Determine whether CMSIS-RTOS2 requires a separate SCA record.
3. Perform vulnerability and support-status analysis.
4. Regenerate the manifest if the formal revision changes.

## Final SBOM Modeling Decision

- **Target:** `M031BSP_Product_SBOM_cdx.json`
- **Name:** CMSIS
- **Supplier:** Arm Limited
- **Version:** 6.1
- **Version scope:** CMSIS-Core
- **License:** Apache-2.0
- **Scope:** `Library/CMSIS/`
