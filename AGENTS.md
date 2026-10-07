# Box3D fork maintenance

Follow the workspace's parent `AGENTS.md` and applicable skills. These instructions supplement, not replace, those rules.

- Read [AI_README.md](AI_README.md) before changing this fork.
- Keep that cumulative record updated in the same task as any local native source, public API/ABI, build compatibility, callback contract, or regression change. Record the problem, owning files/functions, change, reason, compatibility/lifetime impact, and validation actually performed.
- Preserve historical reasons and the upstream baseline. Distinguish upstream functionality, local changes, and game/Jai-binding changes outside this repository. When upstream replaces a local change, record that rather than silently dropping its rationale.
- Box3D owns generic collision/solver behavior and callback safety; Fat Goblins owns stepping and other gameplay rules. Preserve that boundary.
- Public layout changes require matching bindings/binaries and an explicit version decision. Do not claim another platform was tested or player feel was verified without evidence; keep outstanding player checks in the workspace's `VERIFY_LATER.md`.
