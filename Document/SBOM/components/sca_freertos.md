# FreeRTOS Software Composition Analysis Evidence

## Component Summary

- **Component:** FreeRTOS-Kernel
- **Version:** 10.4.3
- **Supplier:** Amazon.com, Inc.
- **Project:** FreeRTOS
- **License:** MIT
- **Integration:** Vendored source
- **Source path:** `ThirdParty/FreeRTOS`
- **SBOM scope:** Test Sample
- **Target profile:** Cortex-M0

## Version Evidence

FreeRTOS Kernel version `10.4.3` was identified consistently in eight reviewed Kernel source files under `ThirdParty/FreeRTOS/Source`.

The reviewed version evidence includes `FreeRTOS.h`, `tasks.c`, `queue.c`, `list.c`, `timers.c`, `event_groups.c`, `stream_buffer.c`, and `croutine.c`.

## License Evidence

The reviewed Kernel source headers consistently contain the complete MIT license text, including the permission grant and the `THE SOFTWARE IS PROVIDED AS IS` disclaimer.

No separate package-level license file was found in the provided FreeRTOS package root. The consistent MIT license text in the eight reviewed Kernel source headers is used as the authoritative license evidence for this BSP distribution.

- **License:** `MIT`
- **License evidence files:** 8
- **Package-level license file found:** No

## Cortex-M0 Port Inventory

The M031 BSP provides three FreeRTOS Cortex-M0 ports:

- **GCC:** `ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM0` - 2 files
- **IAR:** `ThirdParty/FreeRTOS/Source/portable/IAR/ARM_CM0` - 3 files
- **RVDS/Keil:** `ThirdParty/FreeRTOS/Source/portable/RVDS/ARM_CM0` - 2 files

These ports are distributed as part of the FreeRTOS-Kernel component and are not modeled as separate third-party packages.

## Memory Manager Inventory

The FreeRTOS distribution includes the following standard memory manager implementations:

- `heap_1.c`
- `heap_2.c`
- `heap_3.c`
- `heap_4.c`
- `heap_5.c`

An additional file named `heap2.c` is present with a size of 0 bytes and SHA-256 `E3B0C44298FC1C149AFBF4C8996FB92427AE41E4649B934CA495991B7852B855`. This file is treated as an empty, non-functional placeholder and is excluded from the available memory manager list.

## Evidence Files

- `ThirdParty/FreeRTOS/Source/croutine.c` - SHA-256: `E05642EFB95AE0E3B3C1CE45FF073B3F0C092B5788E64AE57B49AEBAC05170EC`
- `ThirdParty/FreeRTOS/Source/event_groups.c` - SHA-256: `554517DC67E260A4E92DB8203280736AECC2C2B759A9BA77CD18DD60CDC802F6`
- `ThirdParty/FreeRTOS/Source/include/FreeRTOS.h` - SHA-256: `584588108DF86F27E2F70B7FC86790B03DE599EA182E7482ACF28548EB3881B6`
- `ThirdParty/FreeRTOS/Source/list.c` - SHA-256: `C3EBB882623C78519DFE5EA17564A5AD0467894FE8D2ED6CFE611FB4F2BC273C`
- `ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM0/port.c` - SHA-256: `E8AFD146D4C0CA5691EBCBE1E1D7F318562019B55A7232BB29B78D1C31E89315`
- `ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM0/portmacro.h` - SHA-256: `38B292A65C48266B69A63CD465EA6C240252A4ED6D71D5F285F6845DF4406351`
- `ThirdParty/FreeRTOS/Source/portable/IAR/ARM_CM0/port.c` - SHA-256: `421BF7B50806EBC17506B5727327C061BE67B464616D82679462365E6C2CA034`
- `ThirdParty/FreeRTOS/Source/portable/IAR/ARM_CM0/portasm.s` - SHA-256: `30BCD0CC7650215E5BBE206A12EC9EE5CADBA43DDD0CDE0BB13A1615372F32F6`
- `ThirdParty/FreeRTOS/Source/portable/IAR/ARM_CM0/portmacro.h` - SHA-256: `2997E8667EA582A2A9A0A5C4689C21E900AEA53D6BC992890E1BC2ED52C78DDF`
- `ThirdParty/FreeRTOS/Source/portable/MemMang/heap_1.c` - SHA-256: `34A7C2420465263D3C88EC821482121A052A7D9AA55E8007E5F7E92B371F0532`
- `ThirdParty/FreeRTOS/Source/portable/MemMang/heap_2.c` - SHA-256: `9E6F66E25B3DFBD705D11DDA454F2D2686A9DF2258F1A96B01AC646730D0E62B`
- `ThirdParty/FreeRTOS/Source/portable/MemMang/heap_3.c` - SHA-256: `13FFF13A0B0A15F04F2747C1E6A08AD505FEDE9360740E6A1C9C7CC901A78E01`
- `ThirdParty/FreeRTOS/Source/portable/MemMang/heap_4.c` - SHA-256: `1E7D9BE1B65126FF894C81FC3DD70767A67046AC08CD754D23F8E48BCB6CCA27`
- `ThirdParty/FreeRTOS/Source/portable/MemMang/heap_5.c` - SHA-256: `D87D6F83EE822F53A86B0E9946D95AE79AFA1C4CFCCD08B90214E8CB6DE90DCE`
- `ThirdParty/FreeRTOS/Source/portable/RVDS/ARM_CM0/port.c` - SHA-256: `2F572F911EFBE7398E940853C94282892F70BE90377FDDF0D41D0BB285B884A0`
- `ThirdParty/FreeRTOS/Source/portable/RVDS/ARM_CM0/portmacro.h` - SHA-256: `B23F07A28402EEDC63314B11965D965292835BD95D42F93638A35CDF87A2F371`
- `ThirdParty/FreeRTOS/Source/queue.c` - SHA-256: `F8F25C5E814B240712EF636F437C5730B3DDE8FC079CEFDDE85087676E1FF418`
- `ThirdParty/FreeRTOS/Source/stream_buffer.c` - SHA-256: `D1E7838BA4C72FBBB50E8FFBAC3CE5A7CEB765B8AB5128147D3F5ABDBAC8024F`
- `ThirdParty/FreeRTOS/Source/tasks.c` - SHA-256: `4EC8568423271EAD2BBE38872976F855034ED48E45BA68FBB8830BF3AE7F4BE9`
- `ThirdParty/FreeRTOS/Source/timers.c` - SHA-256: `B49BD0F3A6A8A032872ED6A4A6A1FB0AF5DF5529D5A24F4ED07E83CE27C8B4F2`

## CycloneDX Mapping

- **Type:** `library`
- **Name:** `FreeRTOS-Kernel`
- **Version:** `10.4.3`
- **Scope:** `optional`
- **PURL:** `pkg:generic/freertos-kernel@10.4.3`
- **BOM reference:** `pkg:generic/freertos-kernel@10.4.3?source=vendored&path=ThirdParty/FreeRTOS`

## BSP Distribution Scope

FreeRTOS is distributed under `ThirdParty/FreeRTOS` to support M031 examples, demonstrations, and evaluation. For the M031 BSP distribution, FreeRTOS-Kernel is classified in the Test Sample SBOM and is not treated as an M031 product runtime dependency.

If FreeRTOS is compiled, linked, or deployed in a final customer or reference firmware, FreeRTOS-Kernel and the selected Cortex-M0 port and memory manager shall be included in that firmware Product SBOM.

## Review Conclusion

FreeRTOS-Kernel `10.4.3` is accepted as an optional Test Sample SBOM component for the M031 BSP distribution. The component is integrated as vendored source and is licensed under the MIT license.

The reviewed evidence confirms three applicable Cortex-M0 ports, five standard memory manager implementations, and twenty source-file Evidence records with verified SHA-256 values.

**Review status:** PASS
