## CMSIS Third-Party Component Description (for SCA / SBOM)

### 1) Component Identity

- Component name: CMSIS
- Component type: library
- Supplier: Arm Ltd.
- Version: 6.1.0
- License: Apache-2.0
- Evidence path: Library/CMSIS

### 2) Evidence for Version and License

Version Evidence:

Library/CMSIS/Core/Include/cmsis_version.h

#define __CM_CMSIS_VERSION_MAIN (6U)
#define __CM_CMSIS_VERSION_SUB  (1U)

License Evidence:

Library/CMSIS/Core/Include/cmsis_version.h

SPDX-License-Identifier: Apache-2.0

### 3) Upstream Information

Repository:
https://github.com/ARM-software/CMSIS_6

Version:
6.1.0

### 4) Suggested CycloneDX Field Mapping

type: library

name: CMSIS

version: 6.1.0

license: Apache-2.0

### 5) Compliance Notes

CMSIS is an Arm maintained software framework and is included in Product SBOM scope.

Version identification is based on cmsis_version.h.

License identification is based on SPDX declaration in cmsis_version.h.
