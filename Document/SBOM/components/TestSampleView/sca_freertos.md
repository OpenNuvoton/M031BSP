# FreeRTOS SCA Evidence

## Component Summary

- **Component:** FreeRTOS-Kernel
- **Type:** Operating system
- **Supplier:** Amazon Web Services
- **Origin:** Third-party
- **SBOM view:** TestSampleView
- **Integration:** Source
- **Runtime:** Embedded firmware
- **Included in ProductView:** No
- **Included in TestSampleView:** Yes

## Repository Baseline

- **Repository:** `M031/bsp`
- **Branch:** `master`
- **Revision:** `51ffb9b7e71511e58b705178b2d7acf34de99de4`
- **Origin master:** `51ffb9b7e71511e58b705178b2d7acf34de99de4`
- **Git describe:** `V3.07.000-46-g51ffb9b`
- **Commit date:** `2026-08-18T17:34:09+08:00`
- **Revision status:** `merged`

## Source Scope

FreeRTOS component source:

- `ThirdParty/FreeRTOS/`

M031 sample integration:

- `SampleCode/FreeRTOS/`

- **Third-party files:** 171
- **Sample integration files:** 16
- **Combined scope files:** 187

FreeRTOS top-level directories:

- `ThirdParty/FreeRTOS/Demo/`
- `ThirdParty/FreeRTOS/Source/`

## Version Evidence

Primary version evidence:

- `ThirdParty/FreeRTOS/Source/include/task.h`

Confirmed version fields:

- `tskKERNEL_VERSION_NUMBER = V10.4.3`
- `tskKERNEL_VERSION_MAJOR = 10`
- `tskKERNEL_VERSION_MINOR = 4`
- `tskKERNEL_VERSION_BUILD = 3`

- **Confirmed version:** `10.4.3`
- **Version status:** Confirmed
- **Evidence type:** Source macro

## Supplier Evidence

Source-file copyright headers identify:

- **Supplier:** Amazon Web Services
- **Copyright holder:** Amazon.com, Inc. or its affiliates

## License Evidence

- **SPDX expression:** `MIT`
- **Status:** Identified
- **Evidence type:** Source-file MIT license headers
- **Standalone license file:** Not found

Example license evidence paths:

- `ThirdParty/FreeRTOS/Source/include/FreeRTOS.h`
- `ThirdParty/FreeRTOS/Source/include/task.h`
- `ThirdParty/FreeRTOS/Source/tasks.c`

The inspected source files contain the MIT permission grant,
including permission to use, copy, modify, merge, publish,
distribute, sublicense, and sell copies of the software.

The absence of a standalone license file under
`ThirdParty/FreeRTOS/` is retained as an evidence condition.

## Sample Integration Evidence

`SampleCode/FreeRTOS/main.c` references:

- `FreeRTOS.h`
- `task.h`
- `timers.h`
- `semphr.h`

`SampleCode/FreeRTOS/ParTest.c` references:

- `FreeRTOS.h`
- `task.h`

These references demonstrate that the M031 FreeRTOS sample
integrates the FreeRTOS kernel source.

## File Manifest

- **Generated path:** `/tmp/m031_freertos_file_manifest.txt`
- **Record format:** `sha256|size|repository-relative-path`
- **Record count:** 187
- **Manifest SHA-256:** `6863013835d0083896409f995d400e8d09e052ee832b0c4d1c273265c8ea0a2f`
- **Validation:** `passed`
- **Tracked by Git:** No

The manifest combines files from:

- `ThirdParty/FreeRTOS/`
- `SampleCode/FreeRTOS/`

The manifest must be regenerated if the formal repository
revision or either FreeRTOS scope changes.

## TestSampleView Decision

FreeRTOS is assigned to TestSampleView because the kernel is
integrated by the M031 FreeRTOS sample but is not a mandatory
runtime dependency of every M031 BSP firmware application.

The FreeRTOS source remains available to BSP users, but its
presence in the repository alone does not mean that every
product firmware includes the FreeRTOS kernel.

## Analysis Status

- **SCA evidence collection:** Completed
- **Version identification:** Completed
- **License identification:** Completed
- **SAST:** Not performed in this evidence step
- **Binary analysis:** Not required
- **Vulnerability analysis:** Pending

## Unresolved Items

1. Perform vulnerability analysis for FreeRTOS-Kernel 10.4.3.
2. Review the upstream support status of version 10.4.3.
3. Confirm the portable layer selected by each project.
4. Confirm the heap implementation selected by each project.
5. Regenerate the manifest if the formal revision changes.

## Final SBOM Modeling Decision

- **Target:** `M031BSP_TestSample_SBOM_cdx.json`
- **Name:** FreeRTOS-Kernel
- **Supplier:** Amazon Web Services
- **Version:** 10.4.3
- **License:** MIT
- **Component source:** `ThirdParty/FreeRTOS/`
- **Sample integration:** `SampleCode/FreeRTOS/`
