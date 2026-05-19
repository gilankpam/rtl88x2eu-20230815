# RTL8822E 5 MHz monitor-mode TX fix — runtime BW change

**Date:** 2026-05-20
**Branch:** v5.15.0.1
**Chip in scope:** RTL8822EU only (USB ID `0bda:a81a`)

## Problem

In this fork, `iw dev wlan0 set freq <ch> 10MHz` sets the radio to 10 MHz at runtime and produces a usable signal that the same chip on the other end can decode. The same command with `5MHz` succeeds at the cfg80211/PHY level (`iw info` reports `width: 5 MHz`, `/proc/net/rtl88x2eu/wlan0/rf_info` reports `cur_bw=5, oper_bw=5`, and TX bytes increment on `/proc/net/dev`), but a peer also configured for 5 MHz receives zero frames. The fork's own `/proc/.../monitor_chan_override` help documents `bw: 10/20/40/80` and conspicuously omits 5.

Empirical evidence (collected this session on drone `192.168.10.152` + GS `10.18.0.1`):

- Drone at PHY 5 MHz + `wfb_tx -B 20`: `/proc/net/dev wlan0` TX bytes growing, GS wfb-server JSON API (`:8103`) reports `video rx all=[0, ...]` for every interval.
- Same setup at PHY 10 MHz + `wfb_tx -B 20`: `video rx all=[~35, ...]` per interval, `lost=0`. Confirmed working at the wfb-server JSON API.

## Root cause

The driver has a `CONFIG_NARROWBAND_SUPPORTING` code path that's intended for narrow-band operation, but it is gated entirely on the modprobe-time registry parameter `rtw_nb_config == RTW_NB_CONFIG_WIDTH_5|10`. The runtime path triggered by `iw set freq` only re-runs the BB switch in `phydm_hal_api8822e.c:2036-2141`, and that path is structurally correct (DAC/ADC clock dividers, RX DFIR, CFR, all programmed for BW5). What it misses:

1. **HALMAC MAC clock** — `cfg_mac_clk_88xx()` (`halmac/halmac_88xx/halmac_cfg_wmac_88xx.c:722-745`) is only invoked from `cfg_bw_88xx()`, which is only invoked from `halmac_set_hw_value(HALMAC_HW_BANDWIDTH)`, which is only called at modprobe-time poweron (`hal_halmac.c:2744-2796`). Without it:
   - `REG_AFE_CTRL1` MAC clock selector stays at `MAC_CLK_HW_DEF_80M` (the BW20 default), should be `MAC_CLK_HW_DEF_20M_BW_5` for 5 MHz.
   - `REG_USTIME_TSF` / `REG_USTIME_EDCA` stay at `MAC_CLK_SPEED` (80 MHz tick), should be `MAC_CLK_SPEED_BW_5M_10M`.

2. **TBTT prohibit timing** — `hal_com.c:17529-17578` programs `REG_TBTT_PROHIBIT` setup + hold time per BW (`TBTT_PROHIBIT_HOLD_TIME_5M` vs `_10M` vs default). Only reached under `CONFIG_NARROWBAND_SUPPORTING + rtw_nb_config`.

3. **DM (PhyDM) BW info hook** — `hal_dm.c:484-487` hooks `ODM_CMNINFO_BW` with the narrow-band value. Only reached under the same guard.

4. **CCK check register** — `rtl8822e_phy.c:947-953` writes `REG_CCK_CHECK_8822E |= BIT_CHECK_CCK_EN_8822E` before the BB switch, only in the narrow-band path.

**Why 10 MHz works without these and 5 MHz doesn't:** the BB/MAC clock ratio at 10 MHz is off by 8× from the intended 2×, which the chip can tolerate well enough to produce a decodable waveform. At 5 MHz the ratio is off by 16× from the intended 4×, and symbol timing/preamble alignment drift past what the peer RX can lock onto.

## Approach

**Option 1 (selected): targeted runtime-path patch.** Add a helper `rtl8822e_apply_narrowband_state(adapter, target_bw)` that unconditionally programs the four missing pieces above based on the target BW. Call it from `rtl8822e_switch_chnl_and_set_bw()` before the existing `config_phydm_switch_bandwidth_8822e()` call. Idempotent — works for entering narrow-band (5/10), exiting back to 20/40/80, or re-entering the same BW. Leaves the existing `CONFIG_NARROWBAND_SUPPORTING + rtw_nb_config` modprobe-fixed path untouched (writes happen twice when both are active, second wins, same value).

Alternatives considered:

- **Option 2** — refactor away the `CONFIG_NARROWBAND_SUPPORTING` `#ifdef` guards entirely and let `current_channel_bw` drive everything. Cleaner but ~6 files affected and risks regressing the modprobe-fixed flow that other forks/users depend on.
- **Option 3** — new `/proc` toggle (mirroring the fork's `monitor_chan_override` style). Adds a second API the user has to remember alongside `iw set freq`. Worse UX.

## Design

### Location & state model

New helper in `hal/rtl8822e/rtl8822e_phy.c`:

```c
static void rtl8822e_apply_bw_side_effects(_adapter *adapter, u8 target_bw);
```

Called once per BW change from `rtl8822e_switch_chnl_and_set_bw()`. The helper runs for *every* target BW (5, 10, 20, 40, 80) — not just narrow ones — because the same registers need different values per BW and the symmetric "restore to 20" writes are how we cleanly exit narrow-band mode without separate enter/exit code paths. Scope is RTL8822E only; other silicon in this driver tree (8812EU) keeps its current behavior.

State is implicit in the registers themselves — no `current_narrowband_state` field needed in `HAL_DATA_TYPE`. Each invocation writes absolute values for the target BW; re-entering the same BW is a no-op (same writes), and transitioning out (e.g. 5 → 20) restores 20-MHz baseline values for all four registers.

### Side effects

| # | Action | Reference in existing code |
|---|---|---|
| 1 | Map `CHANNEL_WIDTH_5/10/20/40/80` → `HALMAC_BW_5/10/20/40/80` | enum table in `halmac_type.h:802-808` |
| 2 | `odm_cmn_info_hook(pDM_Odm, ODM_CMNINFO_BW, &target_bw)` | `hal_dm.c:484-487` |
| 3 | `REG_CCK_CHECK_8822E (0x0454)`: set `BIT_CHECK_CCK_EN_8822E` (BIT 7) for BW5/BW10, clear for BW≥20 | `rtl8822e_phy.c:947-953` |
| 4 | `REG_TBTT_PROHIBIT` setup byte (offset 0) + 12-bit hold (offsets 1/2): `TBTT_PROHIBIT_HOLD_TIME_5M` / `_10M` / default | `hal_com.c:17529-17548` |
| 5 | `api->halmac_set_hw_value(halmac, HALMAC_HW_BANDWIDTH, &bw_type)` | `hal_halmac.c:2744-2796` |

Step 5 indirectly fixes `REG_AFE_CTRL1` MAC clock selector, `REG_USTIME_TSF`, `REG_USTIME_EDCA` via `cfg_mac_clk_88xx()`.

### Call ordering inside `rtl8822e_switch_chnl_and_set_bw()`

```
existing:  mac_switch_bandwidth(adapter, pri_ch_idx);
NEW:       rtl8822e_apply_bw_side_effects(adapter, hal->current_channel_bw);
existing:  config_phydm_switch_bandwidth_8822e(p_dm_odm, pri_ch_idx, hal->current_channel_bw);
```

Rationale: HALMAC MAC clock must be correct before the BB switch runs, because the BB DAC/ADC clock dividers in `R_0x9b4` derive from the MAC clock. The DM hook must be updated before the BB switch because `config_phydm_switch_bandwidth_8822e` reads phydm state during its run. CCK_CHECK and TBTT order vs. BB is independent — bundled into the helper for locality.

### Coexistence with `CONFIG_NARROWBAND_SUPPORTING`

The existing `#ifdef CONFIG_NARROWBAND_SUPPORTING` block in `rtl8822e_switch_chnl_and_set_bw()` (`rtl8822e_phy.c:945-955`) stays. The new helper runs unconditionally on every BW change. If a user runs with both the modprobe-time registry parameter set *and* the runtime path, the registers get written twice — same values both times, second wins.

## Test plan

All tests on live hardware: drone (`192.168.10.152`, armv7l, kernel 4.9.84) and GS (`10.18.0.1`, aarch64, kernel 6.1.84), channel 132 (5660 MHz), wfb-ng video stream as the link-quality indicator.

| # | Setup | Pass criteria | Verification |
|---|---|---|---|
| 1 | Both 20 MHz, wfb_tx `-B 20` | Baseline: video flows | wfb-server JSON `:8103` reports `video rx all > 0`, `lost = 0` |
| 2 | Both 10 MHz, wfb_tx `-B 20` | Existing 10 MHz still works | same |
| 3 | Both **5 MHz**, wfb_tx `-B 20` | New: video flows | ≥1000 video packets over 30 s with `lost ≤ 1 %` |
| 4 | Transition 20→10→5→10→20 (lockstep, 2 s gap) | Video resumes after each step within 2 s | post-transition `video rx all > 0` |
| 5 | Rapid transitions 5↔20 several cycles | No oops, no DMA stall | `dmesg` clean of new WARNs (existing `phy_chk_ch_setting_consistency` is pre-existing, ignored) |

The existing GS quirk where one adapter drifts back to 40 MHz after wfb-server restart is out of scope here; treated as separate work.

## Out of scope

- 5 MHz on RTL8812EU silicon. User runs RTL8822EU only.
- 2.4 GHz narrow-band channels. Code is band-agnostic but only 5 GHz is tested.
- Full regression on `CONFIG_NARROWBAND_SUPPORTING + rtw_nb_config` modprobe-fixed flow. We're additive, not replacing it; only smoke-test that build still compiles with that define.
- Adaptive-BW controller (auto-switching air-side based on link quality). Separate feature.
- Fixing the wfb_tx `-B 5` "Unsupported HT bandwidth" rejection. Not needed — `-B 20` with PHY at 5 MHz produces correct radiotap (both map to `IEEE80211_RADIOTAP_MCS_BW_20`).

## Risks

- **`halmac_set_hw_value(HALMAC_HW_BANDWIDTH)` at runtime** is not exercised by mainline (only called at poweron). Underlying ops (`cfg_bw_88xx` + `cfg_mac_clk_88xx`) are register writes against `REG_WMAC_TRXPTCL_CTL` bits 7/8 and three clock-config registers. All idempotent, no firmware-handshake side effects. Risk is low but unmodeled. Mitigation: watch dmesg during Test 4/5 for HALMAC error returns or new WARNs.
- **TX rate tables** may have additional BW-keyed entries beyond the four registers identified. If 5 MHz video flows but at unexpected rates, follow up by auditing `hal/phydm/halrf/rtl8822e/halrf_iqk_8822e.c` and any TX power tables keyed on BW.
- **CCK PD threshold** in phydm is set the same for BW5 and BW10 in `phydm_hal_api8822e.c:2136-2137`; if 5 MHz floods the chip with false CCK detections, may need to gate or adjust.

## Post-implementation findings (2026-05-20)

The implementation was carried out as specified and validated on the live drone (`192.168.10.152`) + GS (`10.18.0.1`) pair. **Outcome:**

- ✅ 20 MHz regression — 2279 packets/6 s, 0 lost.
- ✅ 10 MHz regression — 2306 packets/6 s, 0 lost. (Runtime path now correctly programs HALMAC MAC clock for BW10; previously this worked despite the wrong clock because the BB/MAC ratio at 10 MHz was tolerable.)
- ✅ Transition ladder (20→10→5→10→20) — 569/581/0/579/573 packets per 3 s; clean recovery after the 5 MHz step.
- ✅ Rapid 5↔20 stress (5 rounds) — no kernel oops, no DMA stalls, 933 packets in final 5 s window at HT20.
- ❌ **5 MHz video did not flow.** Drone TX bytes increment, all four target registers verified at the correct BW5 values (`REG_AFE_CTRL1 = 0x300000`, `REG_USTIME_TSF = 0x14`, `REG_TBTT_PROHIBIT = 0x0f / hold 0x90`, `REG_CCK_CHECK |= BIT(7)`), but the GS adapter at BW5 receives 0 frames per its kernel RX counter. The chip is producing RF output but not a peer-decodable 802.11 waveform.

### Why 5 MHz doesn't reach the air

We attempted three increasingly aggressive remediations beyond the spec'd patch:

1. **Expand `rtl8822e_apply_bw_side_effects` to also program SLOT / PIFS / EDCA / SIFS / ACK / EIFS / PHY_REQ_DELAY at runtime** (mirroring the remaining BW-keyed branches in `halmac_init_8822e.c`). Result: regressed 10 MHz to 0 packets. The runtime injection of EDCA params or SIFS into a live monitor-mode adapter interferes with TX queue admission. Reverted.

2. **Enable `CONFIG_NARROWBAND_SUPPORTING` at compile time and modprobe with `rtw_nb_config=5`** (the upstream-intended path for BW5). Result: USB device probe failed; `wlan0` never appeared; recovery required normal modprobe. The chip cannot complete its init sequence with `HALMAC_BW_5` set at poweron via the registry path on this fork.

3. **Inspected the IQK and DC-cancellation paths** — both skip BW5 (`phydm.c:3795`, `halrf_iqk_8822e.c:1605`). TX power tables share BW20 entries for BW5/10/20 (`hal_com_phycfg.c:2548`). No additional software toggles found that the runtime path could exercise.

The fork's own `/proc/.../monitor_chan_override` help documents `bw: 10/20/40/80` and conspicuously omits 5. Combined with the above, **5 MHz monitor-mode TX is best characterized as not implemented at the chip/firmware level in this fork**, regardless of how the MAC-side init is sequenced. Making it work would require either chip docs we don't have, vendor firmware updates, or a different chip revision.

### Net value of the merged patch

The minimal patch *does* deliver real value:

- The runtime BW-change path (`iw dev wlan0 set freq <ch> <width>`) now correctly programs HALMAC MAC clock, TBTT timing, CCK_CHECK bit, and PhyDM BW hook for any target width. Previously these were only set at modprobe time under a registry guard that's off in the FPV build profile.
- 10 MHz operation, previously working "by accident" (the BB/MAC clock skew was small enough at 10 MHz for the chip to tolerate), is now backed by the correct register state. Symbol timing is on spec rather than relying on chip tolerance.
- Transitions through and out of BW5 are safe — the helper's idempotency means going from BW5 back to BW10/20 correctly restores the wider-BW register values.
- Cleaner runtime path lays the groundwork for adaptive-BW work if 5 MHz support ever lands at the firmware level.

5 MHz remains documented as the goal in this spec; the patch programs everything *we* can program for it. The remaining gap is in vendor firmware / chip behavior.

## References

- Investigation transcript: this conversation, 2026-05-19 / 2026-05-20.
- Key code locations:
  - `hal/rtl8822e/rtl8822e_phy.c:945-955` (existing narrow-band guard + BB call site)
  - `hal/phydm/rtl8822e/phydm_hal_api8822e.c:2036-2141` (BB switch BW5/BW10/BW20)
  - `hal/halmac/halmac_88xx/halmac_cfg_wmac_88xx.c:582-621, 722-745` (HALMAC bw + MAC clock)
  - `hal/halmac/halmac_88xx/halmac_cfg_wmac_88xx.c:529` (`halmac_set_hw_value` HW_BANDWIDTH case)
  - `hal/hal_halmac.c:2744-2796` (modprobe-time HALMAC narrow-band init)
  - `hal/hal_com.c:17529-17578` (TBTT prohibit timing)
  - `hal/hal_dm.c:484-487` (DM ODM_CMNINFO_BW hook)
  - `include/cmn_info/rtw_sta_info.h:68-77` (`enum channel_width`)
  - `hal/halmac/halmac_type.h:802-808` (`enum halmac_bw`)
