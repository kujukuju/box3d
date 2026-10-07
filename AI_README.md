# AI maintenance README — Fat Goblins Box3D fork

Last audited: **2026-10-08**. This is the cumulative record of local native changes, their reasons, and the contracts future AI-assisted changes must preserve. It supplements the API documentation in [docs/simulation.md](docs/simulation.md); it does not describe upstream Box3D features as our work.

> **Review status:** the user explicitly accepts stair-induced ceiling penetration; do not add an upward-clearance sweep or destination-fit veto. The extreme-damping numerical hole is fixed, and the follow-up regressions/validation are recorded in §2.8. Windows/Linux rebuilds and real-player feel remain pending; this allowance is not blanket collision-safety verification.

## Cross-platform rebuild TODO

**AI handoff for the other PC — 2026-10-07. Both tasks are pending.** The callback ABI is now **0.3.0**; only macOS arm64 has been rebuilt. Old Windows/Linux libraries must not be paired with the updated bindings or executables.

- [ ] **Synchronize sources first:** bring over the matching workspace, `box3d`, `JaiBox3D`, `FatGoblins`, and `FatGoblinsServer` changes. They span independent repositories and were uncommitted at this handoff. A pull cannot retrieve unpushed work; commit/push only when the user authorizes it, including updated root gitlinks. This TODO lives in repository documentation, not just local OptMem or ignored `.build/` files.
- [ ] **Windows x64:** with CMake, the Visual Studio C++ x64 toolchain, and Jai available, run from sibling `JaiBox3D`:
  ```bat
  .\build_windows.bat
  .\test_windows.bat
  ```
  Confirm the rebuilt `bin/windows/box3d.dll` and matching `box3d.lib` are staged and the binding smoke test passes.
- [ ] **Linux x86-64:** use an x86-64 Linux host/VM with CMake, Ninja, a C/C++ toolchain, and Jai. Run from sibling `JaiBox3D`:
  ```sh
  ./build_linux.sh
  ./test_linux.sh
  ```
  Confirm `bin/linux/libbox3d.so.0.3.0` and the `.so.0.3` / `.so` symlinks are staged and the binding smoke test passes. Linux scope is the **standalone server**, not a Linux client port.
- [ ] Run the native `WorldTest` / `RecordingTest` and relevant game regressions on each platform. The staging scripts build Release libraries with native unit tests disabled; the smoke scripts do not replace the native suite. Use a separate static native test build as described below.
- [ ] Rebuild the Windows game/embedded server and any Windows standalone server in use; rebuild the Linux standalone server against the matching library. Confirm runtime version **0.3.0**, single precision, architecture, and dynamic-library dependencies. If subsequent fixes change the ABI again, follow the actual current source version rather than forcing 0.3.0.
- [ ] Stage/package native libraries and consumers together. Do not rename an old 0.2.0 library to the new SONAME, bypass the startup compatibility check, or deploy an executable-only Linux update to an old runtime.
- [ ] Record source revisions, platform/toolchain, output artifacts, and actual test results here. Mark each platform complete only after its build and checks succeed. Keep player verification separate in [VERIFY_LATER.md](../VERIFY_LATER.md).

**Build handoff is not player verification:** include the 2026-10-08 numerical fix and regressions in §2.8 when rebuilding. Stair-induced ceiling penetration is explicitly accepted behavior, not a pending request for a clearance veto. Building other platforms does not confirm player feel or authorize unrelated CCD bypasses. Local scratch reproduction files mentioned below will not travel through Git automatically.

## Scope and provenance

- Upstream baseline: **`30c67b5e6d0a3a66f0f506c69ce9e9e0587e3b7c`**, 2026-08-12, from `erincatto/box3d`.
- Original local integration: **`db25c03a0cdf560a9b168ba4e0307252c3f7c2e5`**, 2026-08-24, `updated physics library to match physx modify contact for fat goblins`.
- Subsequent stair/ladder work: **2026-10-07**, the **0.3.0** changes described below. These were uncommitted when this README was written.
- The historical inventory was reconstructed from the actual baseline-to-local-commit diff; the latest inventory was checked against the working diff. Git attributes the historical commit to Kuju, not individual human/AI contributions. This records the local modification set, not an unsupported claim about who typed every line.
- Scope is native source, headers, build metadata, tests, and documentation in this repository. Related game and Jai-binding changes are identified separately below.

To inspect the two implementation layers, from this repository:

```sh
git --no-pager diff 30c67b5 db25c03 -- include src test CMakeLists.txt docs/simulation.md
git --no-pager diff db25c03 -- include src test CMakeLists.txt docs/simulation.md
```

The second command intentionally includes the working tree. After later changes are committed, add their commit IDs and reasons here rather than silently redefining the baseline.

## Maintenance rules for future agents

1. Read this file before changing the native fork. Update it in the same task whenever native behavior, public layout, callback semantics, build compatibility, or related regressions change.
2. For each change, record the observed problem, owning files/functions, actual change, reason, compatibility/lifetime impact, and validation performed. Distinguish tested results from assumptions and pending player checks.
3. Preserve the boundary: **Box3D owns generic contact generation, storage, solving, and callback safety. Fat Goblins owns stair eligibility and other gameplay rules.** Do not put Goblin types, step heights, gameplay impulse markers, or wall classifications into native solver code.
4. Do not replace a per-contact fix with a global physics retune. Keep ordinary/no-op contacts and both scalar and SIMD paths covered by regression tests.
5. Public struct changes require regenerated Jai bindings, matching native binaries, and an explicit version/ABI decision. Do not claim Windows/Linux validation from a macOS build.
6. Preserve borrowed-data lifetimes, current-step freshness, and native storage ownership. Do not cache manifold pointers outside callbacks or introduce a second contact registry.
7. When merging upstream, use the file inventory and regression suites below to preserve each local requirement. Record which local changes become redundant or are replaced upstream.

## 1. Original contact-modification integration — August 2026

### 1.1 Replace the observation/rejection-only callback with mutable discrete contacts

**Problem:** the original pre-solve callback received a point and normal and could reject a collision, but it could not implement the contact edits used by Fat Goblins' PhysX integration. Rejecting an entire pair is not equivalent to changing its support normal, separation, friction, restitution, or individual points.

**Changes:**

- In [include/box3d/types.h](include/box3d/types.h), replaced the old callback parameters `(shapeA, shapeB, point, normal, context)` with `(shapeA, shapeB, b3PreSolveData*, context)`.
- Added `b3PreSolvePhase` and `b3PreSolveData`: discrete manifold array/count, body centers, compound child indices, and continuous candidate point/normal/fraction/triangle indices.
- Added mutable `friction`, `restitution`, `maxNormalImpulse`, and `enabled` to `b3ManifoldPoint`; allowed editing `b3Manifold.normal` while retaining read-only counts.
- Added `b3PreSolve_GetPoint` and `b3PreSolve_SetPoint` in [include/box3d/box3d.h](include/box3d/box3d.h) and [src/contact.c](src/contact.c). The setter derives **both** center-relative anchors from a supplied world-space point.
- Moved discrete callback invocation from the convex-only path into `b3UpdateContact`, after compound-origin and center-of-mass anchor conversion. Mesh/height-field/compound contacts now participate, in the contact's authoritative A/B order rather than a temporary narrow-phase dispatch order.

**Safety/ownership:** `b3InvokeDiscretePreSolve` snapshots originals in existing arena storage, validates edits, compacts disabled points, drops empty manifolds, and leaves allocation/reallocation/free decisions with Box3D. Returning false, or disabling every point, removes the contact for that update. Invalid output asserts and, if execution continues, rejects the contact rather than feeding invalid data to the solver.

A callback may not replace pointers, change manifold/point counts, alter contact feature IDs or cached impulses, mutate the world, or retain the borrowed data. It must be thread-safe. This is targeted PhysX contact-modification support, **not a claim of full PhysX API parity**.

### 1.2 Carry edits through the real scalar and SIMD solvers

**Problem:** editing a public manifold is ineffective if constraint preparation and solving still use only pair-level material properties or retain impulses from incompatible geometry.

**Changes in [src/contact_solver.c](src/contact_solver.c) and [src/contact_solver.h](src/contact_solver.h):**

- Propagated per-point friction, restitution, and normal-impulse caps through mesh/scalar and convex/SIMD constraint preparation.
- Clamped accumulated normal impulses in warm starting, solving, and restitution.
- Used per-point restitution and the sum of per-point friction-weighted normal impulses for central friction/twist limits.
- Selected these properties only when a callback is registered and the contact requests pre-solve. Ordinary contacts retain their existing property selection.
- Existing overflow routines already delegate to the scalar path; no separate overflow solver was introduced.

**Warm-start behavior in `src/contact.c`:** normal/anchor edits clear affected normal impulses; changed geometry or removed points clears manifold friction/rolling/twist caches. No-op callbacks preserve caches. Separation-only edits do not count as geometry changes in this implementation.

Point defaults are the mixed **contact-level** friction/restitution, `FLT_MAX` maximum normal impulse, and `enabled = true`. The hook does not automatically assign a different material coefficient to each mesh triangle.

### 1.3 Make callback enable/disable transitions and recycling correct

**Problem:** contact recycling could skip callbacks or reuse artificial geometry, and enabling pre-solve on a shape after contact creation did not adequately update the live contact state.

**Changes:**

- `b3CollideTask` in [src/physics_world.c](src/physics_world.c) excludes pre-solve-enabled contacts from recycling so updated awake contacts run fresh narrow phase and the callback.
- `b3Shape_EnablePreSolveEvents` in [src/shape.c](src/shape.c) updates existing contacts involving that specific shape, preserving opt-in if either participating shape still requests it.
- Disabling the last opt-in invalidates the relative-transform cache so genuine collision geometry is rebuilt before recycling resumes.

**Limits:** this did not add a wake policy. Sleeping contacts do not acquire callbacks merely because the world callback pointer changes. Also, recycling exclusion uses the contact flag even if the callback pointer is null; the null-callback tests isolate ordinary solver-property behavior, not every possible execution-path difference.

### 1.4 Add safe per-feature material lookup

**Problem:** callback code needs the authored material for the actual contacted mesh/height-field triangle or compound child, not a guessed shape-wide material.

**Changes:** added callback-safe `b3Shape_GetContactMaterialId` and bounds checks around the existing internal material-resolution logic in `src/shape.c`; declared it in `include/box3d/box3d.h`.

This preserves existing compound material indirection. Invalid required child/triangle indices return `0`. It does **not** promise that arbitrary invalid shape handles are safe. Material IDs remain opaque to Box3D; Fat Goblins interprets walkable/wall/ignore bits.

### 1.5 Filter continuous candidates before choosing the nearest accepted hit

**Problem:** rejecting a final nearest time-of-impact result could miss a later valid triangle in the same mesh/compound. A separate path could call a null pre-solve function when shapes opted in but no callback was installed.

**Changes in [src/shape.c](src/shape.c), [src/shape.h](src/shape.h), and [src/solver.c](src/solver.c):**

- Introduced internal `b3TOICandidateFcn` and threaded it through `b3ShapeTimeOfImpact`, mesh, compound, and mesh-fallback candidate processing.
- Invoke the callback **before** accepting a candidate or reducing the search fraction. Rejecting an early triangle still allows a later one to stop the body.
- Install continuous pre-solve filtering only for solid candidates when a callback exists and either shape opts in. This fixes the null-callback path.
- Supply continuous point, normal, fraction, compound child, and triangle identity through `b3PreSolveData`.

**Important limit:** continuous pre-solve still has no mutable solver manifold. `manifolds` is null, `manifoldCount` is zero, and only the callback's boolean return affects CCD. This integration did not turn CCD into the discrete contact solver.

### 1.6 Make the ABI and recording incompatibility explicit

- [CMakeLists.txt](CMakeLists.txt): project/package version `0.1.0` → `0.2.0`.
- [src/CMakeLists.txt](src/CMakeLists.txt): shared-library `SOVERSION` uses `major.minor`; package compatibility uses `SameMinorVersion`, not `SameMajorVersion`.
- [src/recording.h](src/recording.h): recording version `4.4` → `5.0`.
- [src/world_snapshot.c](src/world_snapshot.c): snapshot version `2` → `3`, reflecting the changed raw manifold layout.
- [test/test_recording.c](test/test_recording.c): `RecordingVersionCompatibility` covers the new header, same-major future-minor acceptance, and old-major rejection.

Historical caveat: `b3GetVersion()` in `src/core.c` already returned `0.2.0` in the upstream baseline; the August commit did not change it. Therefore that version number alone did not distinguish the original fork hook ABI from that upstream baseline.

### 1.7 Original regression coverage

Added in [test/test_world.c](test/test_world.c):

| Requirement | Tests |
| --- | --- |
| Reject a contact or disable all points | `TestPreSolveRejectsContact`, `TestPreSolveDisablesAllPoints` |
| Point helpers, mutable properties, partial removal | `TestPreSolveMutableContact`, `TestPreSolvePartialManifoldCompaction` |
| Actual solver consumption | `TestPreSolveMaxNormalImpulse`, `TestPreSolveZeroRestitution`, `TestPreSolveFrictionAffectsSliding` |
| Ordinary/no-op behavior and repeated callbacks | `TestNullPreSolveCallbackUsesOrdinaryContacts`, `TestPreSolveNoOpPreservesWarmStart`, `TestPreSolveRunsEveryStep` |
| Existing-contact flag transitions and geometry reconstruction | `TestPreSolveEnableAfterContact`, `TestPreSolveDisableAfterContact`, `TestPreSolveDisableRebuildsGeometry` |
| Mesh/height-field participation and safe material lookup | `TestPreSolveMeshAndHeightField`, `TestContactMaterialInvalidIndices` |
| CCD rejection and later-triangle search, including compound meshes | `TestContinuousPreSolveRejectsCandidate`, `TestContinuousPreSolveAcceptsLaterTriangle` |

Existing `TestWorldCoverage` was migrated to the new callback and relocated; its noisy diff is not wholesale deletion of coverage. `TestContactEvents` gained explicit A/B ordering checks. `TestContinuousMoveEvent` gained opt-in flags with a null callback to cover that crash path.

## 2. Stair/ladder recovery and CCD handoff — October 2026

### Why the original hook was insufficient

The game could change a riser into upward support, but the native solver still limited penetration recovery with ordinary world tuning: a **3 m/s** speed cap and damping ratio **10** at standard units. Climbing can require much faster upward correction, especially on the actual approximately **74-degree ladder collider**. Lagging recovery eventually made the raised step sweep overlap or fail, restoring an ordinary sloped/riser collision and braking horizontal motion.

Separately, CCD ran afterward against the original geometry. It could truncate horizontal displacement even when the discrete callback had already approved and solved a step. It needed access to that decision without a duplicate gameplay registry or another independent step rule.

The fixes below are native capabilities. Anticipatory step sweeps, correct separation, and deciding which collisions qualify remain game code.

### 2.1 Per-manifold recovery-speed override

Added `b3Manifold.maxPushSpeed`:

- Initialized from `world->contactSpeed` on each contact update.
- Mutable in discrete pre-solve; must be finite and nonnegative.
- `0` disables penetration-recovery bias, **not normal collision response**.
- `FLT_MAX` removes this cap while retaining softness.
- Consumed in both mesh/scalar and convex/SIMD preparation/solving.

**Reason:** allow artificial step support to recover promptly without changing barrel, carryable, ordinary ground, or other contact tuning throughout the world. The override does not directly change speculative-contact bias, friction, restitution, or CCD.

### 2.2 Per-manifold damping override

Added `b3Manifold.contactDampingRatio`:

- Resets to `-1` on each contact update. `-1` uses the existing world softness without recomputing it.
- Other accepted values are finite and nonnegative.
- Reuses the same effective Hertz as the world: `min(world.contactHertz, 0.125 / h)`, with `h` the substep duration.
- Preserves static-contact double Hertz and half damping. A local value of `1` matches setting global damping to `1` for that selected manifold only.
- Scalar softness moved from pair-level prepared storage into the existing per-manifold constraint; SIMD lanes receive the corresponding local softness.
- `b3StepContext.contactHertz` carries the already-computed effective frequency; the solver does not reconstruct it from softness coefficients.

**Reason:** removing the speed cap alone was insufficient on the real steep ladder. This exposes the necessary response tuning locally rather than altering the entire simulation. The native world defaults remain unchanged.

### 2.3 Expose a fresh discrete contact to continuous pre-solve

Added read-only `b3PreSolveData.contactId`:

- Discrete callbacks receive their contact ID, which is validated as read-only metadata.
- Continuous callbacks receive a matching touching contact ID only if a valid, enabled discrete callback completed for that exact shape pair and compound child in the **same advancing step**. Otherwise it is null.
- Native CCD walks the fast body's existing contact adjacency; no new registry, allocation, or public lookup function was added.
- Non-compound contacts use internal child index `0` while continuous candidates use `B3_NULL_INDEX`; child equality is required only for compound shapes.
- During continuous pre-solve, the existing `b3Contact_GetData` can read that contact's solved manifolds. These remain borrowed and read-only. Contact A/B order can differ from continuous candidate order.
- The ID is **not** an automatic CCD exemption. For a mesh, triangle matching and the decision to reject a candidate remain the caller's responsibility.

**Reason:** let the game reuse its already-authoritative discrete step approval rather than have CCD restore the same horizontal block. This does not disable CCD or add mutable continuous manifolds.

### 2.4 Establish current-step freshness, including late wakeups

Added `b3Contact.preSolveStepIndex`, stamped only after successful discrete callback validation and enabled-manifold compaction.

Moved the existing positive-duration `world->stepIndex` increment from `b3Solve` to `b3World_Step` before narrow phase. Discrete callbacks, constraint preparation, and CCD now share one stamp. Zero-duration updates do not advance this counter.

**Why the extra check matters:** a sleeping contact can be moved into the active solver after narrow phase and still contain old modified manifolds. Such a contact must not reuse last step's approval or local recovery tuning.

Consequently:

- Its continuous `contactId` is withheld until a fresh callback completes.
- Both new tuning overrides require the active-callback predicate **and** matching current-step stamp; otherwise the solver uses current world tuning, even if old override values remain visible in storage.
- Existing per-point-property semantics were not changed by this freshness gate.

### 2.5 Version and deployment boundary

Both CMake metadata and `b3GetVersion()` are now **0.3.0**. The public-layout changes require matching native libraries and rebuilt consumers.

Confirmed single-precision, 64-bit Jai/C layouts:

| Type / field | Original local 0.2.0 | Current local 0.3.0 |
| --- | ---: | ---: |
| `b3ManifoldPoint` size | 68 bytes | 68 bytes |
| `b3Manifold` size | 316 bytes | 324 bytes |
| `b3Manifold.maxPushSpeed` offset | absent | 316 |
| `b3Manifold.contactDampingRatio` offset | absent | 320 |
| `b3PreSolveData` size | 88 bytes | 104 bytes |
| `b3PreSolveData.contactId` offset | absent | 88 |

The internal contact layout also changes. Snapshot/recording version constants were not changed again in October; the existing snapshot layout hash includes `sizeof(b3Contact)` and `sizeof(b3Manifold)` and rejects incompatible old snapshot images. Do not infer binary snapshot compatibility merely from unchanged format version constants.

As of this audit, **macOS arm64 is rebuilt**. Checked-in Windows/Linux libraries have **not** been rebuilt for this ABI. Their sources/build staging are prepared, but rebuilding and deploying the native library and game/server together is mandatory. Updated game startup rejects incompatible version/precision unconditionally, including production builds; generated compile-time layout assertions alone cannot validate a loaded old library.

### 2.6 New native regressions

Added in `test/test_world.c`:

- `TestPreSolveMaxPushSpeed`: local/global equivalence for `0`, a binding `6`, and `FLT_MAX`; convex/mesh; independent recovery bounds, ordinary-contact isolation, reset, no-op, and world-tuning changes with recycling. The original nonbinding `30` case was replaced during §2.8.
- `TestPreSolveContactDampingRatio`: local/global equivalence for static convex, static mesh, and dynamic–dynamic contacts at 1/4/8 substeps, damping 0/1/`FLT_MAX`, explicit finite-state checks, default/reset behavior, and ordinary-contact isolation.
- `TestPreSolveDampingManifoldIsolation`: an override remains local within a multi-manifold mesh contact.
- `TestContinuousPreSolveContactId`: current-step solved data, repeated steps/zero-duration update boundaries, later mesh triangle, different shape on the same body, distinct compound children, rejection/all-disabled transitions, contact-slot reuse/generation, and callback-active snapshot restart/continuation.
- `TestContinuousPreSolveStaleContactId`: late-woken contact retains old storage but receives no fresh CCD approval ID.
- `TestPreSolveStaleManifoldTuning`: late-woken convex/mesh contacts with old speed/damping fields solve identically to current-world-tuning references.

Existing CCD tests were strengthened to require a null contact ID where no current discrete approval exists. Recording round trips were rerun against the changed native layouts.

### 2.7 Adversarial design review — 2026-10-07

The user questioned whether the extra fields, CCD handoff, and freshness mechanism were actually correct. This review traced the equations and lifecycle independently and added isolated probes, rather than treating passing tests as proof. **No production physics source was changed during this review.**

#### Per-manifold placement is justified, but not free

A contact with a mesh can have several manifolds with different normals. Artificial support may need different tuning from a wall in the same contact; per-body or per-contact settings would be too broad. The manifold already carries solver impulses and mutable point properties, so these are not the first solver-related fields in a geometry-only object. Separate modifier storage would add association, compaction, and lifetime rules rather than obviously simplify this design.

The cost is eight public bytes per manifold, prepared-constraint storage, and the ABI break. The two controls are independent: the speed field caps penetration bias; damping changes the normal constraint's softness, not just its recovery ceiling. Removing the cap cannot increase recovery when the uncapped bias is already below it. A supplied damping ratio of `1` becomes `0.5` internally against static bodies, following the existing global-tuning convention; it is not universally critical damping.

Scalar, SIMD, and graph-overflow propagation were inspected. Ordinary/recycled contacts use current world tuning, not stale manifold copies. No new normal-range equation error was found.

#### Observed blocked-headroom penetration — accepted by the user on 2026-10-08

The CCD work is a **selective candidate rejection policy**, not general continuous collision against modified geometry. A fresh contact ID proves callback identity/freshness, not that the proposed step is physically achievable or successfully resolved. The game's continuous predicate infers artificial support from its upward normal and exact tuning/impulse-marker values; those values are an implicit gameplay convention, not a native step-approval flag.

An isolated extension of the existing separate-ceiling fixture ran 60 frames at ordinary and fast starting speeds:

- Step height `0.5`; ceiling is only `0.25` above the standing Goblin. A `1.5`-high Goblin cannot fit above the tread in the remaining approximately `1.255` space.
- In the original fast fixture's first frame, the Goblin's X bounds reached approximately `[-0.0115, 0.9885]` inside a step spanning `[0, 3]`, while its bottom remained at `0.2491`: approximately `0.251` vertical overlap. Its top remained below the ceiling, so the old one-frame ceiling assertion still passed.
- At ordinary speed it materially intersected the step for 33 frames, crossed it completely, and its top later reached approximately `0.2865` above the ceiling underside while still beneath the slab's footprint.
- Returning true for Goblin CCD candidates while retaining exactly the same discrete response only delayed entry/crossing by one frame. Therefore the new rejection contributes to immediate entry but is not the sole cause. This comparison does not establish which historical discrete change introduced the underlying failure.
- No ceiling CCD candidate was rejected. Accepting ceiling candidates alone is insufficient to guarantee a valid multi-frame configuration.

`resolve_goblin`'s clearance/support sweeps query only the contacted obstacle, not the separate ceiling. The initial recommendation was a headroom/fit gate, but the user explicitly rejected that restriction and accepted this penetration: intermediate geometry and recoverable side overlaps must not automatically veto a step. **Do not implement that superseded recommendation.** The pre-migration PhysX code at FatGoblins commit `9adcb36c` also used a geometry-pair `PxGeometryQuery_sweep`, not its separate global scene-sweep overload; the old backend's behavior in this exact ceiling fixture has not been run. The downward sweep already supplies the required lift, not a command to move by the full maximum step height.

Local, ignored reproduction files are retained under workspace `.build/stair-review/`: `REPORT.md`, `CeilingReview.jai`, copied test/build sources, `run.log`, and `repeat.log`. From that directory, `python3 run.py rerun.log` reruns the isolated binary with a 30-second bound. Two runs had identical measurements. The observation remains valid, but is no longer a blocker under the user's chosen policy. §2.8 adds checked-in multi-frame finite-state/CCD-filter coverage without requiring nonpenetration.

#### Freshness mechanism is sound within its stated scope

All native `stepIndex` consumers were inspected. Moving the increment preserves once-per-positive-step behavior, including empty/all-sleeping worlds; zero-duration updates do not advance it. Contact allocation clears the stamp. Rejected/empty callbacks cannot authorize CCD. Solver workers finish before CCD reads contact storage, and snapshot restore restores the world counter and contact stamps together. No new lifecycle or counter-relocation defect was found by inspection.

However, the stamp only gates the **new tuning overrides and continuous contact ID**. Late-woken contacts may still solve old modified normals, anchors, separation, and point properties. `TestPreSolveStaleManifoldTuning` deliberately retains artificial support in both its reference and test case; it proves stale tuning is ignored, not that all stale contact edits are removed. That broader limitation predates these new gates. Fixing it would require reviewing the existing late-wakeup/contact-refresh policy, not merely adding another stamp comparison to the new fields.

#### Confirmed extreme-damping API-domain hole — fixed in §2.8

The validator accepts any finite nonnegative local damping. At this review, the `b3MakeSoft` arithmetic in `src/solver.h` could nevertheless overflow for `FLT_MAX`: `a2` becomes infinity and `massScale = a2 * (1 / (1 + a2))` becomes NaN. A compiled probe calling the actual header function at `h = 1/240`, effective world Hertz `30`, reproduced this for both static and dynamic contacts. Damping `1` produced finite coefficients in both cases.

This extreme-value weakness already exists through world damping, but the new override exposes it through another explicitly accepted input. It is not evidence that the game's value `1` is unsafe. The probe established invalid coefficients, not an end-to-end crash. The subsequent fix stabilizes the existing shared calculation without narrowing the accepted damping domain; see §2.8.

#### Validation gaps identified at this review (follow-up in §2.8)

- The native debug `WorldTest` was rebuilt and passed again during this review. That does not contradict the game-level failure above.
- Add a genuinely binding positive local cap: the existing penetration fixture's uncapped initial bias is approximately `12.35`, so its tested `30` cap behaves like the unlimited setting.
- Add override-enabled graph-overflow contacts, loaded warm-start transitions, and explicit finite-state assertions. Equality-only `ENSURE_SMALL` checks can miss NaNs.
- The original freshness/ID fixtures used one worker. Task-order inspection supports read safety, but no new multithreaded readback/race test was run. Snapshot-with-callback restore and contact-slot reuse also deserve direct coverage.
- CCD candidate lookup scans the fast body's adjacency for each candidate. Dense mesh/compound cost has not been benchmarked.

### 2.8 Accepted penetration policy and remaining-contract hardening — 2026-10-08

**User decision:** allow the reproduced stair/ceiling penetration and recoverable side overlaps. Neither an upward-path sweep nor a destination-fit veto was added. This does not mean disabling ceiling collision or granting a body-wide CCD exemption. The game still selects individual fresh step contacts, and unrelated wall/ceiling candidates retain their existing response. Real-player feel is still pending.

**Numerical fix, owner `b3MakeSoft` in [src/solver.h](src/solver.h):** retain the original float calculation when its `a2` intermediate is finite. If that intermediate overflows, recompute the same coefficients using double intermediates and convert the result to float. This avoids `infinity * 0` producing NaN for accepted extreme finite damping, including `FLT_MAX`. No arbitrary damping clamp, global retune, allocation, new abstraction, public field, or ABI/version change was introduced; this remains **0.3.0**. The Mac dylib was rebuilt again for this fix.

**Additional/strengthened tests:**

- [test/test_math.c](test/test_math.c), `TestSoftness`: 990 combinations of timestep, substeps, world frequency, static/dynamic convention, and damping. Explicit finite/range checks, coefficient-sum checks, exceptional-result checks, and bitwise comparisons to the old float equations wherever their intermediates are finite. Reproduced the old failure before applying the fix.
- `TestPreSolveMaxPushSpeed`: a binding positive cap `6` with independent displacement bounds and a demonstrated difference from both world cap `3` and the uncapped response.
- `TestPreSolveContactDampingRatio`: extreme finite damping across scalar/convex and static/dynamic paths, with explicit checks that positions, velocities, manifolds, and cached impulses remain finite rather than relying on equality tests that can miss NaNs.
- `TestPreSolveLoadedTuningTransitions`: genuinely loaded cached contacts before changing/resetting local cap and damping, comparing against equivalent world-tuning references.
- `TestPreSolveOverflowTuning`: populated graph-overflow constraints with actual overrides, warm-start loading, finite-state checks, and local/global/default comparisons.
- `TestContinuousPreSolveContactId`: accepted → rejected → accepted → all-disabled → accepted transitions; actual destroyed/reused contact slot with generation change; public recording/player snapshot restore with callback attached, field readback, fresh/rejected/disabled continuations, and bitwise repeated continuation.
- `TestParallelContinuousPreSolveContactId`: 256 independent pairs on one versus four workers, separately covering non-bullet finalization CCD and bullet CCD. Unique changing markers and solved impulses are read back, rejected/disabled contacts provide no approval, serial/parallel state matches bitwise, and thread-local captures confirm execution on multiple threads. The instrumentation has an explicit MSVC TLS branch, still pending a real Windows build.
- Game `movement_test_approved_step_ccd_guards`: both ordinary- and fast-speed low-ceiling cases now run 60 frames, require finite state and no ceiling CCD exemption, but deliberately do **not** require the character to fit or remain outside the ceiling. Existing unrelated-wall, same-mesh-triangle, and next-frame guards remain.

**Validation on macOS arm64 against the final sources:** full native Debug, Release, and ASan+UBSan suites passed; `WorldTest` passed under ThreadSanitizer without a report. Ten additional ASan+UBSan `WorldTest` runs passed. The rebuilt Mac library passed Jai binding smoke. Full movement and client-physics suites, client + embedded-server build, and standalone-server build passed in isolated scratch outputs. Existing application outputs were untouched; runtime copies were checked against the freshly staged native library. Game validation evidence is retained locally in workspace `.build/stair-final-game/REPORT.md` and logs.

**Remaining scope limits:** Windows/Linux have not been rebuilt or tested here. No player-feel confirmation or dense-mesh performance benchmark is claimed. The inherited late-wakeup behavior described in §2.7 still retains old geometry/point edits; the current-step contract applies specifically to the new tuning and CCD ID. Single-worker late-wakeup and callback-active awake snapshot continuation are covered, but their combination with parallel late wakeup was not exhaustively tested. No new defect in the local-tuning/CCD-ID/step-stamp changes was reproduced by the expanded checks.

## 3. Complete native file inventory

This table covers every tracked implementation/test/build/API-documentation path changed locally relative to `30c67b5` through the October work. It excludes this maintenance README, its agent instructions, and its navigation link.

| File | August integration | October extension |
| --- | --- | --- |
| [CMakeLists.txt](CMakeLists.txt) | Package 0.2.0 | Package 0.3.0 |
| [src/CMakeLists.txt](src/CMakeLists.txt) | Minor-aware SONAME/package compatibility | No additional change |
| [include/box3d/types.h](include/box3d/types.h) | Callback payload/phases and mutable point fields | Local manifold tuning and read-only contact ID |
| [include/box3d/box3d.h](include/box3d/box3d.h) | Point helpers/material query declarations and contracts | Document solved-contact access from CCD |
| [src/contact.c](src/contact.c) | Shared callback dispatch, validation, compaction, anchors, cache handling | Initialize/validate tuning and ID; stamp approval |
| [src/contact.h](src/contact.h) | No change | Contact freshness stamp |
| [src/contact_solver.c](src/contact_solver.c) | Scalar/SIMD point-property consumption | Fresh local speed/damping consumption |
| [src/contact_solver.h](src/contact_solver.h) | Prepared point properties/selection | Prepared manifold cap/softness |
| [src/core.c](src/core.c) | No change | Runtime version 0.3.0 |
| [src/physics_world.c](src/physics_world.c) | Pre-solve recycling exclusion | Earlier step stamp; retained effective Hertz |
| [src/shape.c](src/shape.c) | Live flags/cache invalidation, material bounds/query, TOI candidate filtering | No additional change |
| [src/shape.h](src/shape.h) | Internal TOI candidate callback signature | No additional change |
| [src/solver.c](src/solver.c) | CCD callback adapter, candidate filtering, null-callback fix | Fresh pair/child contact ID; remove old stamp increment |
| [src/solver.h](src/solver.h) | No change | Effective contact Hertz in step context; overflow-safe softness calculation |
| [src/recording.h](src/recording.h) | Recording 5.0 | No additional change |
| [src/world_snapshot.c](src/world_snapshot.c) | Snapshot version 3 | Existing layout hash detects new struct sizes; no file edit |
| [test/test_world.c](test/test_world.c) | Hook/solver/lifecycle/material/CCD regressions | Local tuning, freshness, isolation, overflow, parallel CCD, reuse, and snapshot regressions |
| [test/test_math.c](test/test_math.c) | No change | Finite softness and bitwise ordinary-range regression |
| [test/test_recording.c](test/test_recording.c) | Recording compatibility regression | No additional change |
| [docs/simulation.md](docs/simulation.md) | Replacement callback API and safety contracts | Local tuning and fresh CCD contact access contracts |

No local sample, benchmark, general kinematic-mesh, general overlap-normal, or general wake-policy implementation changes appear in the audited native delta. Upstream already supplied CCD, meshes/compounds/height fields, SIMD, recycling, and recording machinery; we extended their integration points rather than creating those systems.

## 4. Related changes outside this native repository

These are dependencies/consumers, not additional native solver changes:

- [JaiBox3D/generate.jai](../JaiBox3D/generate.jai) and [module.jai](../JaiBox3D/module.jai): regenerated declarations/layout assertions; generator preserves the existing signed 32-bit enum declarations instead of accepting unrelated Clang signedness drift. Edit the generator/header source of truth, not generated layout checks by hand.
- [JaiBox3D/smoke_test.jai](../JaiBox3D/smoke_test.jai): runtime version/precision, new field defaults/consumption, and fresh contact-ID readback coverage.
- [JaiBox3D/build_linux.sh](../JaiBox3D/build_linux.sh): stages `.so.0.3` / `.so.0.3.0`; the Mac dylib is rebuilt in the sibling binding repository.
- [PhysicsCharacter.jai](../FatGoblins/src/shared/physics/PhysicsCharacter.jai): game-owned current/ahead downward sweeps with raised forward clearance, full sweep separation, high-wall preservation, approved-support tuning, and triangle-specific CCD reuse. The native library does not know these gameplay rules.
- [SharedPhysics.jai](../FatGoblins/src/shared/physics/SharedPhysics.jai): unconditional exact-version/single-precision startup check. Global contact tuning remains unchanged.
- [MovementTests.jai](../FatGoblins/tests/MovementTests.jai): per-frame velocity **and displacement** coverage for 72 stair trajectories, moving steps, actual steep ladder geometry, and wall/ceiling/airborne/shared-mesh/next-frame guards. Endpoint-only traversal tests had missed transient braking.
- [Server main_build.jai](../FatGoblinsServer/main_build.jai), server deployment documentation, and the executable-only update script use the new Linux SONAME and require matching deployment libraries.
- [VERIFY_LATER.md](../VERIFY_LATER.md): real-player stair/ladder feel remains pending until explicit user confirmation.

## 5. Validation and reproduction

### Recorded validation of the October implementation

These results were obtained during implementation, **not rerun merely to write this README**:

- Native debug `WorldTest`, `RecordingTest`, and full native unit suite passed.
- Separate macOS arm64 ASan + UBSan + heavy-validation `WorldTest` and `RecordingTest` passed without sanitizer reports.
- Jai binding generation/layout checks and native smoke passed.
- Full movement and client-physics suites, client + embedded-server build, and standalone-server build passed.
- Two final movement runs produced identical logs.
- Loading the saved old 0.2.0 runtime with the updated movement executable exited with the compatibility error rather than proceeding into physics.
- Real player feel and updated Windows/Linux binaries remain unverified.

Existing application binaries/debug outputs were preserved after validation; rebuilding the game is required to activate the source changes. The matching Mac native library remains in the sibling `JaiBox3D/bin/macos/` directory.

### Native commands

A standalone debug test build, run from this repository:

```sh
cmake -S . -B build/ai-tests -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS=OFF -DBOX3D_UNIT_TESTS=ON -DBOX3D_SAMPLES=OFF -DBOX3D_BENCHMARKS=OFF -DBOX3D_DOCS=OFF -DBOX3D_BUILD_SHADERS=OFF -DBOX3D_PROFILE=OFF -DBOX3D_VALIDATE=ON -DBOX3D_SANITIZE=OFF -DBOX3D_DOUBLE_PRECISION=OFF
cmake --build build/ai-tests --target test --parallel 4
./build/ai-tests/bin/test WorldTest
./build/ai-tests/bin/test RecordingTest
./build/ai-tests/bin/test
```

For macOS ASan+UBSan, use a separate build directory and `-DBOX3D_SANITIZE=ON -DBOX3D_SANITIZER_TYPE=address`; the existing Apple CMake branch enables both sanitizers. These are macOS/single-config commands, not a claim of having run the corresponding Windows configuration. The implementation's existing local validation directories were `build/max-push-speed` and `build/abi03-sanitize`.

From sibling `JaiBox3D`, rebuild/stage the Mac library, regenerate bindings after header changes, and test:

```sh
./build_macos.sh
/usr/local/bin/jai generate.jai -quiet
./test_macos.sh
```

Use the sibling platform-specific build/test scripts on supported Windows/Linux hosts. Then run the affected game movement/client-physics suites and client/server builds described in [WORKSPACE.md](../WORKSPACE.md). Check `git diff --check` in each modified repository. Bound test/build runtime when running through an agent; do not use the repository's `build.sh` for an incremental validation directory without checking its destructive cleanup first.
