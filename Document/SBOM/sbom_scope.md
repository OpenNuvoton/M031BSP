# Introduction

The M031 BSP SBOM package consists of two CycloneDX 1.6 SBOM files: a Product SBOM and a Test Sample SBOM.

The Product SBOM covers components included in, linked with, deployed to, or reasonably expected to be integrated into final M031 firmware or software. The Test Sample SBOM covers sample code, demonstrations, validation content, host-side tools, drivers, and third-party components used for evaluation or testing and not treated as product runtime dependencies unless explicitly integrated by the user.

## Release identification

- Product: M031 BSP
- BSP version: V3.07.000-36-g676f919
- Git commit: 676f9193a5726efa60c914f28c7c8a25d6398ff9
- Repository: https://github.com/OpenNuvoton/M031BSP
- SBOM format: CycloneDX JSON 1.6

# Product SBOM

Used for product vulnerability management, customer product integration risk assessment, CRA-related evidence, and internal DevSecOps/SCA controls.

## Scope

Drivers, CMSIS content, device support, startup and system support, and standard peripheral drivers that are compiled, linked, flashed, deployed, or reasonably expected to be integrated into an M031-based product.

## Included paths

```text
.
└── Library
    ├── CMSIS
    ├── Device
    └── StdDriver
```

## Excluded paths

```text
Document/
SampleCode/
ThirdParty/
```

## Component SCA view

Reviewed SCA evidence for Product SBOM components is stored in:

```text
Document/SBOM/components/ProductView/
├── sca_cmsis.json
└── sca_cmsis.md
```

## Formal output

The formal Product SBOM is stored at the BSP repository root:

```text
m031bsp_Product_SBOM_cdx.json
```

# Test Sample SBOM

Used for transparent disclosure and internal or customer evaluation. Components in this scope are not treated as product runtime dependencies unless explicitly integrated into a product by the user.

## Scope

Sample code, demonstration projects, host-side test tools, Windows utilities and drivers, validation tools, test harnesses, and third-party software delivered for sample operation or evaluation. Host-side tools under `SampleCode/` are included through their containing sample paths.

## Included paths

```text
.
├── SampleCode
└── ThirdParty
```

The formal M031 Test Sample SBOM preserves 25 delivery locations as 18 unique binary components and includes one FreeRTOS-Kernel component.

## Excluded paths

```text
Document/
Library/
```

## Component SCA view

Reviewed SCA evidence for Test Sample SBOM components is stored in:

```text
Document/SBOM/components/TestSampleView/
├── sca_freertos.json
└── sca_freertos.md
```

## Formal output

The formal Test Sample SBOM is stored at the BSP repository root:

```text
m031bsp_TestSample_SBOM_cdx.json
```

# Scope and disposition rules

- Component names, versions, suppliers, licenses, PURLs, and CPEs are included only when supported by reviewed evidence.
- Unverified automatically inferred identifiers are excluded from the formal SBOM.
- Unknown component versions are not guessed and must include a documented disposition.
- Placeholder metadata is replaced only when supported by authoritative evidence or documented owner confirmation.
- Duplicate binary files are consolidated by SHA-256 while all delivery locations are preserved.

# Release status

- Internal release status: PASS
- External release status: PASS
- Pending review categories: 0
