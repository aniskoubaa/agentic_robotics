<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Day 2 — Designing for both an Online VLA (OpenVLA) and a Local VLA (SmolVLA)

> **Short answer: yes — and it's the ideal shape for Day 2.**
> One client interface, two swappable backends, and a **canonical action
> contract**, so the Lab 2.2 executor doesn't care whether the "brain" is a
> 7B model on the GPU server or a 450M model running in-process.
>
> **Policy for RAISE 2026:** the **default backend is the local small model
> (SmolVLA, in-process)**. A **self-hosted OpenVLA** backend (LAN *or* cloud) is
> kept **configurable / opt-in** — there is **no public OpenVLA API** to call,
> so "remote" always means infrastructure *we* stand up.

---

## 1. The design: one interface, two backends

```python
# api_clients/vla_client/base.py
class VLAClient(Protocol):
    def act(self, image, instruction, state=None) -> Action: ...
    #  Action = a CANONICAL contract both backends MUST return,
    #  e.g. {"joints": [6 floats], "gripper": float}  (UR5e command space)
```

```python
# backend A — LOCAL, in-process (SmolVLA, 450M)
class LocalSmolVLA(VLAClient):
    def __init__(self, ckpt="lerobot/smolvla_base", device="cuda"):
        self.policy = SmolVLAPolicy.from_pretrained(ckpt).to(device).eval()
    def act(self, image, instruction, state):
        a = self.policy.select_action({...})   # native LeRobot action
        return to_canonical(a)

# backend B — REMOTE, over HTTP (OpenVLA-7B served on the 4090 box)
class RemoteVLA(VLAClient):
    def __init__(self, url): self.url = url
    def act(self, image, instruction, state):
        r = requests.post(f"{self.url}/act", json=encode(image, instruction))
        return to_canonical(r.json())           # server already normalized it
```

```python
# one switch — env var / CLI flag picks the brain; the executor is unchanged.
# DEFAULT = "local-smolvla". Remote is opt-in (and self-hosted — see below).
def make_vla_client(backend="local-smolvla"):   # | "remote-openvla" | "remote-pi0"
    ...
```

The executor stays **backend-blind**, and **defaults to the local small model**:

```python
client = make_vla_client(os.getenv("VLA_BACKEND", "local-smolvla"))  # default = local
action = client.act(img, instruction, state)
arm.send(action)
```

**Config contract (open by design):**

| Setting | Default | Purpose |
|---|---|---|
| `VLA_BACKEND` | `local-smolvla` | which brain: `local-smolvla` \| `remote-openvla` \| `remote-pi0` |
| `VLA_LOCAL_CKPT` | `lerobot/smolvla_base` | local checkpoint (or a fine-tuned path) |
| `VLA_REMOTE_URL` | *(unset)* | required only when `VLA_BACKEND=remote-*`; e.g. `http://gpu-box.lan:8000` |
| `VLA_DEVICE` | `cuda` if available else `cpu` | local inference device |

If `VLA_BACKEND` is unset, everything runs **fully local** with no server, no
network, no extra setup. Setting `VLA_BACKEND=remote-openvla` + `VLA_REMOTE_URL`
flips to the self-hosted endpoint — nothing else in the executor changes.

- **Remote** is real and easy: OpenVLA ships `vla-scripts/deploy.py` — POST
  image + instruction → action.
- **Local** is just loading `SmolVLAPolicy` in-process (laptop GPU or even CPU).

---

## 2. ⚠️ The one real gotcha: action spaces differ

The *interface* unifies trivially; the *action semantics* do not. This is the
thing that bites people.

| | SmolVLA (local, **fine-tuned on your data**) | OpenVLA (remote, zero-shot) |
|---|---|---|
| Native output | UR5e **joint** actions (your LeRobot dataset) | 7-DoF **EE deltas**, normalized for Bridge/RT data |
| Maps to UR5e? | ✅ directly | ❌ needs un-normalization + a Cartesian/IK adapter; zero-shot accuracy on a novel greenhouse task is poor |

**So the canonical contract needs a per-backend adapter** (`to_canonical()`),
and you decide *where* the action space is anchored.

**Cleanest for the class:** anchor on the **UR5e command space**, and make the
*remote* model also a **fine-tuned** checkpoint (serve a fine-tuned OpenVLA *or*
a fine-tuned SmolVLA on the server) so both speak the same action language.
Keep "raw zero-shot OpenVLA" only as a **demo of why it flails** — a great
teaching moment, not the graded path.

---

## 3. Why this is great pedagogy (the real lesson)

Local-vs-remote *is* the central deployment decision in real robotics, and this
design lets students **measure** it:

| Axis | Local SmolVLA | Remote OpenVLA |
|---|---|---|
| Latency | tens of ms, no network | round-trip + queue (watch the 500 ms budget!) |
| Capability | 450M | 7B |
| Needs a local GPU? | yes | **no** (just an HTTP call) |
| Offline / network drop | keeps working | breaks |
| Update the model | re-deploy locally | swap server checkpoint, all clients get it |

---

## 4. How it maps to the existing lab tracks

The Day-2 track split maps onto the config — but the **default is local**, so
every student has a working brain with zero infrastructure:

- **Default / everyone** → `VLA_BACKEND` unset ⇒ `local-smolvla`: fine-tune
  their own SmolVLA, run it in-process. No server, no network. This is the
  graded path.
- **Optional / advanced / instructor** → `VLA_BACKEND=remote-openvla` +
  `VLA_REMOTE_URL`: point at a **self-hosted** OpenVLA (LAN box or cloud GPU)
  and **A/B against the local model** over the same 5 scenarios — report
  latency + success.

A `--backend` flag on `vla_execute` plus a one-line latency log turns Lab 2.2
into an optional **local-vs-self-hosted benchmark**, without ever blocking the
default local run.

---

## 4b. "Remote" = self-hosted (there is no public OpenVLA API)

OpenVLA ships as **open weights + a self-host server** (`vla-scripts/deploy.py`,
exposing `POST /act {image, instruction, unnorm_key}`). No managed provider
(Replicate / Modal / Baseten / fal / HF Inference) serves it out of the box — it
needs `trust_remote_code` + a custom `predict_action()`, so the remote backend is
always **infrastructure we stand up**:

- **LAN (recommended for the school):** run `deploy.py` on the 4090 box on the
  local network. A network round-trip costs ~50–200 ms; our **action-chunk**
  execution + ≤500 ms/chunk budget absorbs that on a LAN.
- **Cloud (configurable):** deploy the same `deploy.py` on a serverless GPU
  (**Modal** is the easiest, code-first; RunPod / Baseten / Replicate-as-Cog also
  work). Keep it close — a *far* cloud endpoint can blow the latency budget.

Either way it's the same `VLA_REMOTE_URL` knob; π₀ (openpi) and SmolVLA's
`PolicyServer` are self-hosted in exactly the same manner.

---

## 5. Architecture at a glance

```
                        ┌──────────────────────────────┐
   Lab 2.2 executor     │   make_vla_client(backend)    │
   (backend-blind)  ───►│   VLA_BACKEND env / --backend │
                        │   default = local-smolvla     │
                        └───────────────┬───────────────┘
                       ┌────────────────┴─────────────────┐
                       ▼  (default)                        ▼  (opt-in, self-hosted)
            LocalSmolVLA (in-process)            RemoteVLA (HTTP)
            SmolVLAPolicy.select_action          POST url/act
                       │                                   │
                       ▼                                   ▼  (server hosts
                to_canonical(a)                     to_canonical(json)   OpenVLA / π0 /
                       │                                   │              SmolVLA)
                       └──────────────┬────────────────────┘
                                      ▼
                       canonical Action {joints[6], gripper}
                                      │
                                      ▼
                                 arm.send(action)  →  UR5e
```

---

## 6. Files this implies (when we build it)

- `api_clients/vla_client/base.py` — `VLAClient` protocol + canonical `Action`.
- `api_clients/vla_client/local_smolvla.py` — in-process SmolVLA backend.
- `api_clients/vla_client/remote_vla.py` — HTTP backend.
- `api_clients/vla_client/factory.py` — `make_vla_client(backend)`.
- `api_clients/vla_client/server.py` — thin FastAPI server hosting OpenVLA /
  SmolVLA / π0 behind `POST /act` (returns the canonical contract).
- Lab 2.2 `starter/vla_executor.py` gains a `--backend` flag + a latency log.

> Decision context for which model to host lives in
> [`VLA_MODEL_CHOICE.md`](./VLA_MODEL_CHOICE.md); build order and Day-1 linkage
> in [`HOW_TO_BUILD_DAY2_LABS.md`](./HOW_TO_BUILD_DAY2_LABS.md).
