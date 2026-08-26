# Spectral BLDC firmware release guide

How to cut a release of the Spectral BLDC firmware (the `Spectral BLDC
Firmware/` PlatformIO project) so it's consistent with past releases
(V104–V106) and works with the web flasher (motorgui.com), which reads
`firmware.json` from the GitHub Release, not from anything in the repo tree.

This was reconstructed by inspecting the actual V104/V105/V106 commits,
tags, and GitHub Releases (via the GitHub API) for **this** repo — it started
as a copy of the equivalent guide from the STEPFOC stepper-controller repo,
but STEPFOC's values (branch name, asset naming, repo path, tag style)
didn't all match what Spectral actually does, so they were re-derived
against Spectral's own history instead of assumed. If a future release
deviates from this on purpose, update this file in the same commit so it
keeps describing what we actually do.

## The four things a release produces

1. A commit on `Bootloader_main` containing the source change **and** the
   rebuilt `Binaries/firmware.bin`, with `FIRMWARE_VERSION` bumped.
2. An annotated git tag `VNNN` on that commit, pushed to `origin`.
3. A GitHub Release (`Release NNN`) on that tag, with a changelog body.
4. Two files attached to that Release: `spectral-bldc-firmware-VNNN.bin` and
   `firmware.json`.

(3) and (4) are **not** git tags or repo files — they only exist on GitHub.
Pushing the tag alone does not create them; see step 7.

### Why there are two copies of the binary

- `Binaries/firmware.bin` — generic filename, committed to the repo, updated
  every release. Lets you `git log`/`git blame`/diff binary size across
  history and gives tooling a stable in-repo path.
- `spectral-bldc-firmware-VNNN.bin` — versioned filename, uploaded as a
  **Release asset**, never committed to the repo. This is the immutable,
  downloadable artifact that `firmware.json` points at and that the web
  flasher fetches for a specific version. Don't skip either one — they serve
  different consumers.

---

## Step by step

### 1. Make the firmware change, bump the version

Edit `Spectral BLDC Firmware/src/constants.h`:

```c
#define FIRMWARE_VERSION 107   // was 106
```

This is why the bump matters, not just bookkeeping: `#Info` reports this
constant directly so a flashed board's version is trustworthy immediately,
without an EEPROM round-trip. Skipping the bump means two different binaries
both claim to be V106.

### 2. Build

```
cd "Spectral BLDC Firmware"
pio run
```

If `pio` isn't on PATH, it's usually at
`%USERPROFILE%\.platformio\penv\Scripts\pio.exe`. This is the
`genericSTM32F103CB` environment — bootloader-aware
(`VECT_TAB_OFFSET=0x3000`, app base address `0x08003000`), matching what the
bootloader on the boards expects. Output:
`.pio/build/genericSTM32F103CB/firmware.bin`.

Confirm it actually built clean (no errors) and check the size line against
`board_upload.maximum_size` in `platformio.ini` so you're not silently over
flash budget.

### 3. Update the in-repo binary and commit

```
cp "Spectral BLDC Firmware/.pio/build/genericSTM32F103CB/firmware.bin" "Binaries/firmware.bin"
git add "Binaries/firmware.bin" <the changed source files>
git commit -m "<summary> (VNNN)

<changelog body, same content that will go in the GitHub Release>

Bump FIRMWARE_VERSION to NNN.

Co-Authored-By: Claude <model> <noreply@anthropic.com>"
```

Source + binary go in **one commit** (this is what V105/V106 did — don't
split "code change" and "update binaries" into separate commits unless
there's a real reason to). Include a `Co-Authored-By` trailer when the
change was AI-assisted, matching V105/V106.

### 4. Tag and push

```
git tag -a VNNN -m "Release NNN"
git push origin Bootloader_main
git push origin VNNN
```

Tags in this repo are **annotated** (`git tag -a`) as of V105/V106. (V104 and
earlier used lightweight tags — that was the old convention, before the
bootloader branch existed; stay with annotated going forward.)

### 5. Compute the release artifact's hash

```
sha256sum Binaries/firmware.bin
wc -c < Binaries/firmware.bin
```

### 6. Write `firmware.json`

This is the manifest the web flasher reads. Exact schema (confirmed from the
real V105/V106 assets):

```json
{
  "version": "VNNN",
  "firmware": "spectral-bldc-firmware-VNNN.bin",
  "size": <byte size>,
  "sha256": "<sha256 of the binary, lowercase hex>",
  "target": "STM32F103CB",
  "app_base_address": "0x08003000",
  "commit": "<full 40-char commit hash>"
}
```

`commit` is the **full** hash here, unlike the release body text (which uses
the short 7-char form in prose).

### 7. Create the GitHub Release

Prefer the `gh` CLI if it's installed:

```
gh release create VNNN \
  --target <full-commit-hash> \
  --title "Release NNN" \
  --notes-file release_body.md \
  "Binaries/firmware.bin#spectral-bldc-firmware-VNNN.bin" \
  firmware.json
```

Pass `--target` as the full commit hash, not the branch name — `Bootloader_main`
keeps moving, and you want the release pinned to the exact commit that was
built. (Note: the real V105/V106 releases on GitHub actually show
`target_commitish: "main"` — an artifact of `gh` defaulting to the repo's
default branch when `--target` wasn't passed. It's harmless because the tag
itself points at the right commit either way, which is what actually gets
checked out/downloaded, but pass `--target` explicitly going forward so the
release metadata is correct too.)

If `gh` isn't available, the release can be created directly via the GitHub
REST API. This needs a token — reuse the one git's own credential manager
already has for push access rather than creating a new one:

```bash
CRED=$(printf "protocol=https\nhost=github.com\n\n" | git credential fill)
TOKEN=$(printf '%s\n' "$CRED" | sed -n 's/^password=//p')

# 1) create the release (target_commitish = full commit hash)
curl -s -X POST -H "Authorization: token $TOKEN" \
  -H "Accept: application/vnd.github+json" \
  -d @create_payload.json \
  https://api.github.com/repos/Source-Robotics/Spectral-Micro-BLDC-controller/releases

# create_payload.json: {"tag_name":"VNNN","target_commitish":"<full sha>",
#   "name":"Release NNN","body":"<changelog>","draft":false,"prerelease":false}

# 2) upload each asset to the upload_url from the response above
curl -s -X POST -H "Authorization: token $TOKEN" \
  -H "Content-Type: application/json" \
  --data-binary @firmware.json \
  "<upload_url_base>?name=firmware.json"

curl -s -X POST -H "Authorization: token $TOKEN" \
  -H "Content-Type: application/octet-stream" \
  --data-binary @spectral-bldc-firmware-VNNN.bin \
  "<upload_url_base>?name=spectral-bldc-firmware-VNNN.bin"
```

Never print `$TOKEN` — keep it in a shell variable only, and don't pass
`-v`/`--verbose` to curl while it's set (verbose mode echoes headers).

### 8. Write the release body

Structure used by every real release (V105/V106) — reuse it exactly:

```markdown
Spectral BLDC firmware **VNNN** — application image for flashing via the Spectral bootloader.

### <Category> (e.g. CAN bus / Safety / EEPROM / Performance / Usability)
- What changed, in terms of what a user/integrator would notice — not a
  restatement of the commit's code diff.

### Assets
- `spectral-bldc-firmware-VNNN.bin` — application firmware image (flash at `0x08003000`, target STM32F103CB).
- `firmware.json` — manifest with version, size, and SHA-256 for download verification by the web flasher.

### Integrity
- SHA-256: `<hash>`
- Size: <bytes> bytes

Built from commit <short-sha> on `Bootloader_main`.
```

Use only the category headers that actually apply — don't pad with empty
sections. Release name is `Release NNN` (capital R).

### 9. Verify

```
curl -s https://api.github.com/repos/Source-Robotics/Spectral-Micro-BLDC-controller/releases/tags/VNNN
```

Confirm both assets show `"state": "uploaded"`, and that
`https://github.com/.../releases/download/VNNN/firmware.json` actually
serves the manifest (this is what the web flasher fetches — a release with
the right tag but a missing/broken asset looks fine in the GitHub UI but
breaks flashing).

---

## Before you call it done

- **Bench-test before flashing a fleet.** Run any relevant hardware
  verification against the new binary before treating the release as
  trustworthy.
- **Multi-node robots need every node reflashed, not just one.** Mixed
  firmware across a robot's drivers is a failure mode in its own right —
  don't ship a release note that implies a partial update is fine.
- **A flash typically clears calibration** unless the change was
  specifically about EEPROM migration (V106's CRC split was designed to
  survive upgrades from older firmware — see its release notes). Default
  assumption: park → confirm → flash → re-home/recalibrate.
- **Don't touch `main`.** All recent firmware releases (V105+) are cut from
  `Bootloader_main`, which has diverged from `main` (bootloader support
  isn't merged back yet — confirmed: `main`'s tip does not contain the V105
  or V106 commits). Keep releasing from `Bootloader_main` until that merge
  happens, and don't assume `main`'s state matches what's shipping.
