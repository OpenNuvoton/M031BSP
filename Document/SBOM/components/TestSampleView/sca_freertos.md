## FreeRTOS Third-Party Component Description (for SCA / SBOM)

### 1) Component Identity

- Component name: FreeRTOS
- Component type: library
- Supplier: FreeRTOS Project / Amazon
- Version: 10.4.3
- License: MIT
- Evidence path: ThirdParty/FreeRTOS

### 2) Evidence for Version and License

Version Evidence:

ThirdParty/FreeRTOS/Source/include/task.h

#define tskKERNEL_VERSION_NUMBER "V10.4.3"

License Evidence:

ThirdParty/FreeRTOS/LICENSE

MIT License

### 3) Upstream Information

Repository:
https://github.com/FreeRTOS/FreeRTOS-Kernel

Version:
10.4.3

### 4) Suggested CycloneDX Field Mapping

type: library

name: FreeRTOS

version: 10.4.3

license: MIT

### 5) Compliance Notes

FreeRTOS is included in the Test Sample SBOM scope.

Version identification is based on task.h.

License identification is based on the FreeRTOS project MIT license.
