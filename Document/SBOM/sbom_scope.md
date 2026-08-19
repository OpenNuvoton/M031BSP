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
