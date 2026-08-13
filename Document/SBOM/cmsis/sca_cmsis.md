# CMSIS Software Composition Analysis Evidence

## Component Summary

- **Component:** CMSIS
- **Version:** 6.1.0
- **Supplier:** Arm Limited
- **License:** Apache-2.0
- **Integration:** Vendored source
- **Source path:** `Library/CMSIS`
- **Runtime profile:** CMSIS-Core Cortex-M0

## Version and Runtime Profile Evidence

CMSIS-Core version `6.1` is derived from `Library/CMSIS/Core/Include/cmsis_version.h`:

- `__CM_CMSIS_VERSION_MAIN` = `6`
- `__CM_CMSIS_VERSION_SUB` = `1`

The normalized CMSIS component version is recorded as `6.1.0` to align with the M2L31 SBOM component version format.

The M031 runtime profile is `CMSIS-Core Cortex-M0`. `Library/Device/Nuvoton/M031/Include/M031Series.h` includes `core_cm0.h` and does not include `core_cm0plus.h`.

## API and License Evidence

- **CMSIS-RTOS2 API version:** `2.3.0` from `Library/CMSIS/RTOS2/Include/cmsis_os2.h`
- **CMSIS-Driver Common API version:** `2.0` from `Library/CMSIS/Driver/Include/Driver_Common.h`
- **License:** `Apache-2.0` identified by SPDX declarations in the reviewed CMSIS source headers.

The CMSIS source is integrated directly into the M031 BSP repository as vendored source. No package manager is used for this integration.

## Evidence Files

- `Library/CMSIS/Core/Include/cmsis_version.h` - SHA-256: `2604DABF484BB5DF3D76B77BC6DA12094238994DFC03827A6E97EC7FE8CF0328`
- `Library/CMSIS/Core/Include/core_cm0.h` - SHA-256: `7BFACABC98BF7F3C47C77FA9481B29D0923AE0F214D24942174507159F40822E`
- `Library/CMSIS/Driver/Include/Driver_Common.h` - SHA-256: `F7E9515637B8300ECC5F4C137771B84414F66BF77702D159FC1E5BF0213F285C`
- `Library/CMSIS/RTOS2/Include/cmsis_os2.h` - SHA-256: `4480094AA59AC50B6FA2086A25138996673948830A2DA2A15FB15DB912058C2C`

## CycloneDX Mapping

- **Type:** `library`
- **Name:** `CMSIS`
- **Version:** `6.1.0`
- **Scope:** `required`
- **PURL:** `pkg:generic/cmsis@6.1.0`
- **BOM reference:** `pkg:generic/cmsis@6.1.0?source=vendored&path=Library/CMSIS`

## Review Conclusion

CMSIS `6.1.0` is accepted as a required Product SBOM component for the M031 BSP. The component is integrated as vendored source under `Library/CMSIS` and is licensed under `Apache-2.0`.

The M031 runtime dependency is specifically the CMSIS-Core Cortex-M0 profile. Other processor headers, templates, tests, and documentation distributed under `Library/CMSIS` are package contents but are not direct M031 runtime dependencies.

**Review status:** PASS
