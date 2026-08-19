# HIDCtrlTransferTest SCA Evidence

## Component Summary

- **Component:** HIDCtrlTransferTest
- **Type:** Windows host-side application
- **Supplier:** Nuvoton Technology Corporation
- **Origin:** First-party
- **SBOM view:** TestSampleView
- **Integration:** Source and prebuilt binary
- **Runtime:** Windows host-side
- **Included in device firmware:** No
- **Included in test-sample scope:** Yes
- **Version:** `1.0.0.1`
- **Version status:** Confirmed from PE metadata

## Repository Baseline

- **Repository:** `M031/bsp`
- **Branch:** `master`
- **Revision:** `51ffb9b7e71511e58b705178b2d7acf34de99de4`
- **Origin master:** `51ffb9b7e71511e58b705178b2d7acf34de99de4`
- **Git describe:** `V3.07.000-46-g51ffb9b`
- **Commit date:** `2026-08-18T17:34:09+08:00`
- **Revision status:** `merged`

## Tool Scope

- `Tool/HIDCtrlTransferTest/`
- **Tool files:** 23

Firmware integration evidence:

- `SampleCode/StdDriver/USBD_HID_Transfer_CTRL/`

The following samples are excluded from this component:

- `SampleCode/StdDriver/USBD_HID_Transfer/`
- `SampleCode/StdDriver/USBD_HID_Transfer_And_Keyboard/`
- `SampleCode/StdDriver/USBD_HID_Transfer_And_MSC/`

The excluded samples are represented by the separate
HIDTransferTest component.

## Version Evidence

- **Evidence path:** `Tool/HIDCtrlTransferTest/HIDCtrlTransferTest.exe`
- **File version:** `1.0.0.1`
- **Product version:** `1.0.0.1`

Version `1.0.0.1` is confirmed from the executable's
PE Version Resource.

## License Evidence

- **Directory license:** `Apache-2.0`
- **Status:** Identified with third-party exceptions
- **Evidence path:** `Tool/HIDCtrlTransferTest/LICENSE.md`
- **License SHA-256:** `ef189a94f4a15d5d88ff94bec498d26ef7935dfbb87efb43b33c7195b863f5d4`

The directory-level Apache-2.0 notice does not establish
the identities or licenses of the four bundled libraries.

Third-party files remain subject to their own applicable
copyright notices and license terms.

## Executable Evidence

- **Path:** `Tool/HIDCtrlTransferTest/HIDCtrlTransferTest.exe`
- **Size:** 1868800 bytes
- **SHA-256:** `9099be6e12479e50b0706e3dfe1828fa4fd62dcbcc0c2a462db1783b35ecac5f`
- **Format:** PE32
- **Subsystem:** GUI
- **Architecture:** x86
- **Version:** `1.0.0.1`
- **Authenticode:** NotSigned
- **Signer certificate:** Not present

## PE Metadata Quality

Authoritative PE fields:

- **FileVersion:** `1.0.0.1`
- **ProductVersion:** `1.0.0.1`
- **OriginalFilename:** `HIDCtrlTransferTest.exe`

Non-authoritative placeholder fields:

- **CompanyName:** `TODO: <Company name>`
- **ProductName:** `TODO: <Product name>`
- **FileDescription:** `TODO: <File description>`
- **LegalCopyright:** `TODO: (c) <Company name>. All rights reserved.`

The placeholder values are retained as evidence but are not
used to establish component identity or supplier.

The supplier is established using the directory license
evidence and the repository context.

## Bundled Library Evidence

### hid.lib

- **Path:** `Tool/HIDCtrlTransferTest/HIDCtrlTransferTest/hid.lib`
- **Size:** 12384 bytes
- **SHA-256:** `3f80f3213b65a99268cc19e38b1378cf381e8cb92730290ed4fce9c3e55dc756`
- **File type:** Archive or import-library candidate
- **Identity:** UNKNOWN
- **Identity status:** Unconfirmed
- **License:** NOASSERTION
- **License status:** Unresolved
- **Archive-member inspection:** Not performed
- **Printable-string inspection:** Not performed
- **Reason:** ar and strings tools were unavailable

### hidclass.lib

- **Path:** `Tool/HIDCtrlTransferTest/HIDCtrlTransferTest/hidclass.lib`
- **Size:** 1766 bytes
- **SHA-256:** `8b8721bf4e19b875082e5c0bbcdd191fc00442ca755f66947e5b21a8862ad3d4`
- **File type:** Archive or import-library candidate
- **Identity:** UNKNOWN
- **Identity status:** Unconfirmed
- **License:** NOASSERTION
- **License status:** Unresolved
- **Archive-member inspection:** Not performed
- **Printable-string inspection:** Not performed
- **Reason:** ar and strings tools were unavailable

### hidparse.lib

- **Path:** `Tool/HIDCtrlTransferTest/HIDCtrlTransferTest/hidparse.lib`
- **Size:** 9212 bytes
- **SHA-256:** `00912ceec31d6c7ae453f0323068ef29919fe2677d9d94bae20e2d6562dc82fb`
- **File type:** Archive or import-library candidate
- **Identity:** UNKNOWN
- **Identity status:** Unconfirmed
- **License:** NOASSERTION
- **License status:** Unresolved
- **Archive-member inspection:** Not performed
- **Printable-string inspection:** Not performed
- **Reason:** ar and strings tools were unavailable

### setupapi.lib

- **Path:** `Tool/HIDCtrlTransferTest/HIDCtrlTransferTest/setupapi.lib`
- **Size:** 141048 bytes
- **SHA-256:** `7bf22e0bc1a833e598e6f9874fd0296e69100467451fe1519a88ab3f358d031a`
- **File type:** Archive or import-library candidate
- **Identity:** UNKNOWN
- **Identity status:** Unconfirmed
- **License:** NOASSERTION
- **License status:** Unresolved
- **Archive-member inspection:** Not performed
- **Printable-string inspection:** Not performed
- **Reason:** ar and strings tools were unavailable

## Project Metadata

- **Solution path:** `Tool/HIDCtrlTransferTest/HIDCtrlTransferTest.sln`
- **Solution format:** 10.00
- **IDE:** Visual Studio 2008
- **Project path:** `Tool/HIDCtrlTransferTest/HIDCtrlTransferTest/HIDCtrlTransferTest.vcproj`
- **Project GUID:** `{80836F19-E76E-4EDA-9EA2-591FBC079CE4}`
- **Root namespace:** `HIDCtrlTransferTest`
- **Configuration type:** Application
- **Configuration type raw value:** `1`
- **Character set:** Unicode
- **Character set raw value:** `1`

## Sample Integration Evidence

- **Integration path:**
  `SampleCode/StdDriver/USBD_HID_Transfer_CTRL/`
- **Transport:** USB HID control transfer
- **Scope status:** Integration evidence

The firmware sample uses HID class control-transfer
operations and is kept separate from the interrupt
IN/OUT HID Transfer samples.

## File Manifest

- **Generated path:** `/tmp/m031_hidctrltransfertest_file_manifest.txt`
- **Record format:** `sha256|size|repository-relative-path`
- **Record count:** 23
- **Manifest SHA-256:** `a5d174596c69b9a16ac7362662b9a24bf5787fb94c265e862a2ea6b4bb75b95e`
- **Validation:** passed
- **Tracked by Git:** No

The manifest must be regenerated if the formal repository
revision or `Tool/HIDCtrlTransferTest/` content changes.

## Analysis Status

- **SCA evidence collection:** Completed
- **Version identification:** Completed
- **License identification:** Completed with limitations
- **SAST:** Not performed
- **Binary analysis:** Partial
- **Malware scanning:** Not performed
- **Authenticode validation:** NotSigned

## Security and Quality Findings

1. The distributed executable is unsigned.
2. PE identity fields contain Visual Studio placeholders.
3. Four bundled library identities remain unresolved.
4. Four bundled library licenses remain NOASSERTION.
5. SAST and binary security scanning remain pending.

## Recommended Actions

1. Replace the PE placeholder fields with approved
   Nuvoton component metadata.
2. Confirm the identity and license of `hid.lib`.
3. Confirm the identity and license of `hidclass.lib`.
4. Confirm the identity and license of `hidparse.lib`.
5. Confirm the identity and license of `setupapi.lib`.
6. Perform SAST on the host-tool source.
7. Perform malware and binary security scanning on the EXE.
8. Consider applying an approved Authenticode signature.

## Final SBOM Modeling Decision

- **Target:** `M031BSP_TestSample_SBOM_cdx.json`
- **Name:** HIDCtrlTransferTest
- **Supplier:** Nuvoton Technology Corporation
- **Version:** 1.0.0.1
- **License:** Apache-2.0 with third-party exceptions
- **Tool scope:** `Tool/HIDCtrlTransferTest/`
- **Integration:** `SampleCode/StdDriver/USBD_HID_Transfer_CTRL/`
- **Runtime:** Windows host-side
- **Included in device firmware:** No

The component structure follows the completed
HIDTransferTest SCA evidence model.
