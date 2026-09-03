# M031 BSP SBOM scope and release readiness

- SBOM sync base: `14039966eeb207be101a34d1de4942fc09864af3`
- Formal source: `4dc3d5af62330f8a4366d2392c3b2f773200b3ab` (`V3.07.000-71-g4dc3d5af`)
- Release state: **blocked**
- Artifact closure: 60/60 Git-tracked `.a`, `.apk`, `.bin`, `.cat`, `.dll`, `.exe`, `.inf`, `.lib`, and `.sys` paths
- FreeRTOS aggregate: 171 files using `sha256-path-nul-content-nul-v1`

Durable exact-path and hash-bound license evidence is stored in `Document/SBOM/evidence/binary-license-database.json`. The reproducible FreeRTOS aggregate inventory is stored in `Document/SBOM/evidence/freertos-inventory.json`.

Release remains blocked pending authoritative decisions for unresolved prebuilt-artifact licenses and Microsoft SDK provenance, a maintained-dictionary CPE for FreeRTOS Kernel 10.4.3, complete vulnerability identifier coverage, and an approved PSIRT/VEX assessment. Local scan, audit, and validation work products are not retained in the BSP repository. No `not_affected` assertion, PSIRT approval, release commit, or distribution commit is asserted.
