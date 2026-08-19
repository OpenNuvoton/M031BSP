# HIDTransferTest SCA Evidence

## Component Summary

- **Component:** HIDTransferTest
- **Type:** Windows host-side application
- **Supplier:** Nuvoton Technology Corporation
- **Origin:** First-party
- **SBOM view:** TestSampleView
- **Included in device firmware:** No
- **Version:** `UNKNOWN`
- **Version status:** Not present in PE metadata

## Repository Baseline

- **Repository:** `M031/bsp`
- **Branch:** `master`
- **Revision:** `51ffb9b7e71511e58b705178b2d7acf34de99de4`
- **Git describe:** `V3.07.000-46-g51ffb9b`
- **Revision status:** `merged`

## Tool Scope

- `Tool/HIDTransferTest/`
- **Tool files:** 16

## License Evidence

- **Directory license:** `Apache-2.0`
- **Evidence:** `Tool/HIDTransferTest/LICENSE.md`
- **License SHA-256:** `ef189a94f4a15d5d88ff94bec498d26ef7935dfbb87efb43b33c7195b863f5d4`

Third-party material remains subject to its own notices.
The identity and license of `hid.lib` remain unresolved.

## Executable Evidence

- **Path:** `Tool/HIDTransferTest/HIDTransferTest.exe`
- **Size:** 5579776 bytes
- **SHA-256:** `24bdd0baadd8ee6632b40757d2f599f391f9b87b87e23f8fb30324c8c873e747`
- **Format:** Windows PE32
- **Architecture:** x86
- **Version:** `UNKNOWN`
- **Authenticode:** NotSigned

## Library Evidence

- **Path:** `Tool/HIDTransferTest/HIDTransferTest/hid.lib`
- **Size:** 12384 bytes
- **SHA-256:** `22bcccaed9b092f4ab2aa12e1471959ed1847296f350bb6d7390b0718fdb5ca4`
- **File type:** ar archive / import-library candidate
- **Identity:** UNKNOWN
- **License:** NOASSERTION
- **Archive-member inspection:** Not performed
- **Printable-string inspection:** Not performed
- **Reason:** ar and strings were unavailable

## Project Metadata

- **Solution format:** 12.00
- **IDE:** Visual Studio 2012
- **Platform toolset:** v110
- **Configuration type:** Application
- **Character set:** Unicode
- **Solution encoding:** UTF-8 with BOM
- **Working-tree line ending:** CRLF

## Sample Integration

Included integration paths:

- `SampleCode/StdDriver/USBD_HID_Transfer/`
- `SampleCode/StdDriver/USBD_HID_Transfer_And_Keyboard/`
- `SampleCode/StdDriver/USBD_HID_Transfer_And_MSC/`

`USBD_HID_Transfer_CTRL` is excluded and will be modeled
under the separate HIDCtrlTransferTest component.

Observed HID Transfer features include:

- HID report descriptors
- Interrupt IN and OUT endpoints
- GET_REPORT and SET_REPORT requests
- Erase, read, write, and test commands

## File Manifest

- **Generated path:** `/tmp/m031_hidtransfertest_file_manifest.txt`
- **Record count:** 16
- **Manifest SHA-256:** `17c776c570ca3615276dd746c1a3ac7327eb5561e5eb6c1ed95350b29eed2253`
- **Validation:** Passed
- **Tracked by Git:** No

## Security Findings

1. The distributed executable is unsigned.
2. No authoritative version is present in PE metadata.
3. The identity and license of hid.lib remain unresolved.
4. SAST and binary security scanning remain pending.

## Final SBOM Modeling Decision

- **Target:** `M031BSP_TestSample_SBOM_cdx.json`
- **Name:** HIDTransferTest
- **Supplier:** Nuvoton Technology Corporation
- **Version:** UNKNOWN
- **License:** Apache-2.0 with third-party exceptions
- **Runtime:** Windows host-side
- **Included in firmware:** No
