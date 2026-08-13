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

## Evidence

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

## Formal output

The formal Product SBOM is stored at the BSP repository root, aligned with the M2L31 BSP layout:

```text
m031bsp_Product_SBOM_cdx.json
```

# Test Sample SBOM

Used for transparent disclosure and internal or customer evaluation, but labeled as a non-product runtime dependency unless explicitly integrated into a product by the user.

## Scope

Sample code, demonstration projects, host-side test tools, Windows utilities and drivers, validation tools, test harnesses, and third-party software delivered for sample operation or evaluation. Host-side tools under `SampleCode/` are included through their containing sample paths. M031 does not have a repository-root `Tool/` directory.

## Evidence

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

## Formal output

The formal Test Sample SBOM is stored at the BSP repository root, aligned with the M2L31 BSP layout:

```text
m031bsp_TestSample_SBOM_cdx.json
```

# Scope and disposition rules

- `Document/SBOM/work/` is a temporary working directory and is not part of the formal release scope.
- `Document/SBOM/output/` contains raw tool output and is not part of the external release scope unless explicitly requested for audit purposes.
- Backup files such as `*.bak` and `*.pre-external-review.bak` are excluded from release.
- Unverified Syft-generated CPE values are excluded from the formal Test Sample SBOM and tracked in the CPE disposition evidence.
- The incorrect NuGet PURL identified during analysis is excluded from the formal Test Sample SBOM and tracked in the PURL disposition evidence.
- Unknown versions are not guessed. `dfu-util` is identified as version `0.8` from tool-specific embedded evidence; remaining HIDTransferTest binaries retain `UNKNOWN` with a documented `unavailable` disposition.
- Placeholder PE metadata is replaced only when supported by PE evidence, file identity, and internal owner confirmation.

# Release status

- Internal release status: PASS
- External release status: PASS
- Pending review categories: 0
