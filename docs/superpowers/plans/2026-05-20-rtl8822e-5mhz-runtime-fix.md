# RTL8822E 5 MHz runtime BW fix — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `iw dev wlan0 set freq <ch> 5MHz` produce a peer-decodable signal on RTL8822EU, matching how 10 MHz already works at runtime — without requiring `CONFIG_NARROWBAND_SUPPORTING + rtw_nb_config` at modprobe time.

**Architecture:** Add a single helper `rtl8822e_apply_bw_side_effects(adapter, target_bw)` in `hal/rtl8822e/rtl8822e_phy.c` that programs the four side effects the runtime path currently misses (HALMAC bandwidth → MAC clock, TBTT prohibit timing, PhyDM BW hook, CCK check register). Call it from the existing driver-side BW switch path between the MAC config and the BB config so the BB switch sees the corrected MAC clock. Idempotent — works for entering 5/10, exiting back to 20/40/80, or re-entering the same BW. Leaves the existing `CONFIG_NARROWBAND_SUPPORTING` modprobe-fixed path untouched.

**Tech Stack:** C, Linux kernel module (out-of-tree), Realtek HALMAC + PhyDM driver framework. Target kernels: 4.9.84 (drone, armv7l) and 6.1.84 (GS, aarch64).

**Spec:** `docs/superpowers/specs/2026-05-20-rtl8822e-5mhz-runtime-fix-design.md`

---

## File Structure

| File | Change | Responsibility |
|---|---|---|
| `hal/rtl8822e/rtl8822e_phy.c` | Modify | Add helper `rtl8822e_apply_bw_side_effects()` + one call site in `switch_chnl_and_set_bw_by_drv()` |

Single-file change. No new headers, no new globals, no Makefile changes. All constants, functions, and macros referenced are already in scope at this file's existing `#include` set.

---

## Task 1: Implement helper and wire it into the BW-change path

**Files:**
- Modify: `hal/rtl8822e/rtl8822e_phy.c` (add helper above `switch_chnl_and_set_bw_by_drv`, add call inside it)

**Background — what each side effect does and why it's there:**

| # | Action | Why missing today | Reference pattern |
|---|---|---|---|
| 1 | `odm_cmn_info_hook(pDM_Odm, ODM_CMNINFO_BW, &target_bw)` | Gated on `rtw_nb_config` in `hal/hal_dm.c:484` | `hal/hal_dm.c:484-487` |
| 2 | `REG_CCK_CHECK_8822E` BIT(7): set for BW5/BW10, clear for BW≥20 | Only set in `#ifdef CONFIG_NARROWBAND_SUPPORTING` block at `rtl8822e_phy.c:947-953` | same file, same function |
| 3 | `REG_TBTT_PROHIBIT` setup byte + 12-bit hold time | Gated on `rtw_nb_config` in `hal/hal_com.c:17529` | `hal/hal_com.c:17529-17548` |
| 4 | `api->halmac_set_hw_value(mac, HALMAC_HW_BANDWIDTH, &bw_type)` — drives `cfg_mac_clk_88xx()` → `REG_AFE_CTRL1`, `REG_USTIME_TSF`, `REG_USTIME_EDCA` | Only called at modprobe-time poweron, `hal/hal_halmac.c:2750-2751` | `hal/hal_halmac.c:2540-2555` (canonical access pattern) |

Constants and helpers already available in this file's scope:
- `CHANNEL_WIDTH_5/10/20/40/80` (from `<include/cmn_info/rtw_sta_info.h>`)
- `HALMAC_BW_5/10/20/40/80`, `HALMAC_HW_BANDWIDTH`, `halmac_set_hw_value` (from HALMAC headers transitively)
- `REG_CCK_CHECK_8822E`, `BIT_CHECK_CCK_EN_8822E` (used at line 947 already)
- `REG_TBTT_PROHIBIT` (0x0540, from `include/hal_com_reg.h`), `TBTT_PROHIBIT_SETUP_TIME` (0x04), `TBTT_PROHIBIT_HOLD_TIME` (0x80), `TBTT_PROHIBIT_HOLD_TIME_10M` (0xc8), `TBTT_PROHIBIT_HOLD_TIME_5M` (0x190) (from `include/hal_com.h:317-325`)
- `ODM_CMNINFO_BW`, `odm_cmn_info_hook()` (from `hal/phydm/phydm.h`)
- `dvobj_to_halmac()`, `HALMAC_GET_API()`, `adapter_to_dvobj()` (HALMAC framework macros, used throughout `hal_halmac.c`)
- `rtw_read8()`, `rtw_write8()` (already used at line 947)

- [ ] **Step 1: Insert the helper function above `switch_chnl_and_set_bw_by_drv`**

In `hal/rtl8822e/rtl8822e_phy.c`, immediately before the line `static void switch_chnl_and_set_bw_by_drv(PADAPTER adapter, u8 switch_band)` (currently line 904), add:

```c
/*
 * Apply per-BW side effects that the modprobe-time CONFIG_NARROWBAND_SUPPORTING
 * path normally programs once at poweron. We re-apply them on every runtime
 * BW change so that "iw set freq <ch> 5MHz/10MHz/20MHz/..." works without
 * requiring rtw_nb_config to be set.
 *
 * Idempotent: writes absolute values keyed on target_bw. Re-entering the same
 * BW is a no-op; transitioning out of narrow-band (e.g. 5->20) restores the
 * 20-MHz baseline values.
 *
 * Must be called BEFORE config_phydm_switch_bandwidth_8822e() so the BB switch
 * sees the corrected MAC clock.
 */
static void rtl8822e_apply_bw_side_effects(PADAPTER adapter, u8 target_bw)
{
	PHAL_DATA_TYPE hal = GET_HAL_DATA(adapter);
	struct dm_struct *p_dm_odm = &hal->odmpriv;
	struct dvobj_priv *dvobj = adapter_to_dvobj(adapter);
	struct halmac_adapter *mac = dvobj_to_halmac(dvobj);
	struct halmac_api *api = HALMAC_GET_API(mac);
	enum halmac_bw bw_type = HALMAC_BW_20;
	u8 cck_check;
	u8 tbtt_setup;
	u16 tbtt_hold;

	/* 1. Map CHANNEL_WIDTH_* -> HALMAC_BW_* */
	switch (target_bw) {
	case CHANNEL_WIDTH_5:	bw_type = HALMAC_BW_5;	break;
	case CHANNEL_WIDTH_10:	bw_type = HALMAC_BW_10;	break;
	case CHANNEL_WIDTH_20:	bw_type = HALMAC_BW_20;	break;
	case CHANNEL_WIDTH_40:	bw_type = HALMAC_BW_40;	break;
	case CHANNEL_WIDTH_80:	bw_type = HALMAC_BW_80;	break;
	default:
		/* Includes CHANNEL_WIDTH_160 and CHANNEL_WIDTH_80_80 -- not used
		 * by this fork's monitor-mode FPV use case; fall through to BW20
		 * baseline rather than failing the BW change. */
		bw_type = HALMAC_BW_20;
		break;
	}

	/* 2. Per-BW values for TBTT and CCK_CHECK */
	if (target_bw == CHANNEL_WIDTH_5) {
		tbtt_setup = 0xf;
		tbtt_hold  = TBTT_PROHIBIT_HOLD_TIME_5M;
	} else if (target_bw == CHANNEL_WIDTH_10) {
		tbtt_setup = 0x8;
		tbtt_hold  = TBTT_PROHIBIT_HOLD_TIME_10M;
	} else {
		tbtt_setup = TBTT_PROHIBIT_SETUP_TIME;
		tbtt_hold  = TBTT_PROHIBIT_HOLD_TIME;
	}

	/* 3. PhyDM BW hook (re-program so phydm reads the right BW during BB switch) */
	odm_cmn_info_hook(p_dm_odm, ODM_CMNINFO_BW, &target_bw);

	/* 4. CCK_CHECK: enable narrow-band CCK check for BW5/BW10, disable otherwise */
	cck_check = rtw_read8(adapter, REG_CCK_CHECK_8822E);
	if (target_bw == CHANNEL_WIDTH_5 || target_bw == CHANNEL_WIDTH_10)
		cck_check |= BIT_CHECK_CCK_EN_8822E;
	else
		cck_check &= ~BIT_CHECK_CCK_EN_8822E;
	rtw_write8(adapter, REG_CCK_CHECK_8822E, cck_check);

	/* 5. TBTT prohibit setup time + 12-bit hold time (offsets 0/1/2 of 0x540) */
	rtw_write8(adapter, REG_TBTT_PROHIBIT, tbtt_setup);
	rtw_write8(adapter, REG_TBTT_PROHIBIT + 1, tbtt_hold & 0xFF);
	rtw_write8(adapter, REG_TBTT_PROHIBIT + 2,
		(rtw_read8(adapter, REG_TBTT_PROHIBIT + 2) & 0xF0) | ((tbtt_hold >> 8) & 0x0F));

	/* 6. HALMAC HW_BANDWIDTH: drives cfg_bw_88xx() + cfg_mac_clk_88xx() which
	 *    programs REG_AFE_CTRL1 MAC clock selector, REG_USTIME_TSF, REG_USTIME_EDCA.
	 *    This is the single most important write -- without it, MAC clock stays
	 *    at the BW20 default (80 MHz) while BB clocks are at 5/10 MHz, and the
	 *    timing skew produces a non-decodable waveform. */
	api->halmac_set_hw_value(mac, HALMAC_HW_BANDWIDTH, &bw_type);
}
```

- [ ] **Step 2: Wire the helper into `switch_chnl_and_set_bw_by_drv`**

In `hal/rtl8822e/rtl8822e_phy.c`, between the existing line 941 (`mac_switch_bandwidth(adapter, pri_ch_idx);`) and the existing line 945 (`#ifdef CONFIG_NARROWBAND_SUPPORTING`), insert one line:

Find this block:
```c
		/* 3.1 set MAC register */
		mac_switch_bandwidth(adapter, pri_ch_idx);

		/* 3.2 set BB/RF registet */

#ifdef CONFIG_NARROWBAND_SUPPORTING
```

Change to:
```c
		/* 3.1 set MAC register */
		mac_switch_bandwidth(adapter, pri_ch_idx);

		/* 3.2 set runtime-narrowband side effects (HALMAC bw, TBTT, DM hook,
		 * CCK_CHECK). Must run before the BB switch so BB clock dividers
		 * derive from the corrected MAC clock. Idempotent for non-narrow BWs. */
		rtl8822e_apply_bw_side_effects(adapter, hal->current_channel_bw);

		/* 3.3 set BB/RF registet */

#ifdef CONFIG_NARROWBAND_SUPPORTING
```

- [ ] **Step 3: Verify the file compiles (build check is in Task 2; this is just a syntax skim)**

Read back the changed region of `hal/rtl8822e/rtl8822e_phy.c` lines ~900–970. Confirm:
- Helper function appears once, statically declared, complete (matching braces, ends in `}`).
- Call site is between `mac_switch_bandwidth(...)` and the `#ifdef CONFIG_NARROWBAND_SUPPORTING` line.
- No duplicate calls (the helper is called exactly once per BW change).
- No leftover diff conflict markers (`<<<<<<<`, `=======`, `>>>>>>>`).

- [ ] **Step 4: Commit**

```bash
cd /home/gilankpam/Projects/drone/rtl88x2eu-20230815
git add hal/rtl8822e/rtl8822e_phy.c
git commit -m "rtl8822e: apply runtime BW side effects so 5/10 MHz work via iw

When the user runs 'iw dev wlan0 set freq <ch> 5MHz' (or 10MHz) at
runtime, the driver previously only re-ran the BB switch and missed
four pieces of state normally only set at modprobe time under
CONFIG_NARROWBAND_SUPPORTING + rtw_nb_config:

 1. HALMAC HW_BANDWIDTH (drives MAC clock + USTIME_TSF/EDCA)
 2. REG_TBTT_PROHIBIT setup/hold time
 3. PhyDM ODM_CMNINFO_BW hook
 4. REG_CCK_CHECK_8822E narrow-band bit

At 10 MHz the MAC/BB clock skew was small enough for the chip to
produce a decodable waveform anyway. At 5 MHz the skew was too large
and the peer could not lock onto the preamble.

Add rtl8822e_apply_bw_side_effects() and call it from
switch_chnl_and_set_bw_by_drv() between the MAC switch and the BB
switch. Idempotent for all widths so transitions out of narrow-band
restore the 20-MHz baseline cleanly. Existing CONFIG_NARROWBAND_SUPPORTING
modprobe-fixed path is untouched.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Cross-compile the module for drone (armv7l, kernel 4.9.84)

**Files:** none modified (build output only).

This task assumes you have the OpenIPC SDK / cross toolchain you've used to build this driver before. If you don't, you'll need that set up before this step; that's outside this plan.

- [ ] **Step 1: Run your existing cross-compile target for the drone**

Use the same build command you used to produce `/usr/bin/8812eu.ko` on the drone (visible in `lsmod` output, kernel 4.9.84 armv7l, `MaxTxBufLen=32` modparam). Typical pattern for this fork:

```bash
cd /home/gilankpam/Projects/drone/rtl88x2eu-20230815
make clean
make ARCH=arm CROSS_COMPILE=<your-arm-prefix>- KSRC=<your-arm-kernel-src> -j$(nproc)
```

Expected: build completes with no errors. A `8812eu.ko` is produced (path depends on your build flow).

- [ ] **Step 2: Verify the new symbol is present**

```bash
<your-arm-objdump> -t 8812eu.ko | grep apply_bw_side_effects
```

Expected: one line, `... l F .text ... rtl8822e_apply_bw_side_effects` (static function, local linkage).

If the symbol is missing, the helper was inlined or stripped — non-fatal but suggests your build optimization is aggressive. Re-grep with `nm 8812eu.ko | grep apply_bw_side_effects`.

---

## Task 3: Cross-compile the module for GS (aarch64, kernel 6.1.84)

**Files:** none modified.

- [ ] **Step 1: Run your existing aarch64 cross-compile target**

Same as Task 2 but for the GS kernel (6.1.84 aarch64). Typical pattern:

```bash
make clean
make ARCH=arm64 CROSS_COMPILE=<your-aarch64-prefix>- KSRC=<your-aarch64-kernel-src> -j$(nproc)
```

Expected: build completes with no errors.

- [ ] **Step 2: Verify the new symbol is present**

```bash
<your-aarch64-objdump> -t 8812eu.ko | grep apply_bw_side_effects
```

Expected: one line. Same as Task 2.

---

## Task 4: Sideload the new modules and verify clean load

**Files:** none modified. Live host changes via SSH.

- [ ] **Step 1: Stop wfb-ng services on both hosts (to allow module unload)**

Drone:
```bash
ssh root@192.168.10.152 'killall wfb_tx wfb_rx wfb_tun msposd mavfwd 2>/dev/null; sleep 1; ps w | grep -E "wfb_|msposd" | grep -v grep || echo "all stopped"'
```
Expected: `all stopped`.

GS:
```bash
ssh root@10.18.0.1 'systemctl stop wifibroadcast@gs.service 2>/dev/null; killall wfb-server wfb_tx wfb_rx 2>/dev/null; sleep 1; pgrep -a wfb || echo "all stopped"'
```
Expected: `all stopped`.

- [ ] **Step 2: Unload + reload module on drone**

```bash
scp <path-to-drone-8812eu.ko> root@192.168.10.152:/lib/modules/4.9.84/extra/8812eu.ko
ssh root@192.168.10.152 'rmmod 8812eu && modprobe 8812eu rtw_tx_pwr_by_rate=0 rtw_tx_pwr_lmt_enable=0 MaxTxBufLen=32 && sleep 2 && lsmod | grep 8812eu && dmesg | tail -20'
```

Expected:
- `lsmod`: shows `8812eu` loaded.
- `dmesg`: standard driver init messages, no oops, no new WARN beyond the pre-existing `phy_chk_ch_setting_consistency` (which is cosmetic).
- `ip link show wlan0`: interface exists.

- [ ] **Step 3: Unload + reload module on GS**

Note: the GS has both `8812eu` and `aic8800_fdrv` loaded. Only reload `8812eu`.

```bash
scp <path-to-gs-8812eu.ko> root@10.18.0.1:/lib/modules/6.1.84/extra/8812eu.ko
ssh root@10.18.0.1 'rmmod 8812eu && modprobe 8812eu && sleep 2 && lsmod | grep 8812eu && dmesg | tail -20'
```

Expected: same as drone.

- [ ] **Step 4: Re-create monitor interfaces and put both adapters at channel 132 / 20 MHz (start state for tests)**

Drone:
```bash
ssh root@192.168.10.152 '
  ip link set wlan0 down
  iw dev wlan0 set type monitor
  ip link set wlan0 up
  iw reg set 00
  iw dev wlan0 set freq 5660 HT20
  iw dev wlan0 info | grep -E "channel|width"
'
```
Expected: `channel 132 (5660 MHz), width: 20 MHz`.

GS:
```bash
ssh root@10.18.0.1 '
  for w in wlx84fc146c36e6 wlx84fc146c36f4; do
    ip link set $w down
    iw dev $w set type monitor
    ip link set $w up
    iw dev $w set freq 5660 HT20
  done
  for w in wlx84fc146c36e6 wlx84fc146c36f4; do
    iw dev $w info | grep -E "channel|width"
  done
'
```
Expected: both adapters report `channel 132 (5660 MHz), width: 20 MHz`.

---

## Task 5: Regression test — 20 MHz baseline

**Files:** none modified.

Goal: confirm we did not break the existing 20 MHz path. This is the most common operating mode and any regression here invalidates everything else.

- [ ] **Step 1: Start a minimal wfb_tx on drone and wfb_rx on GS at 20 MHz**

Drone (TX):
```bash
ssh root@192.168.10.152 '
  iw dev wlan0 set freq 5660 HT20
  wfb_tx -K /etc/drone.key -M 2 -B 20 -k 8 -n 12 -U rtp_local -S 1 -L 1 -i 7669206 -C 8000 -J 10 -E 5000 wlan0 &> /tmp/wfb_tx_video.log &
  sleep 2
  ps w | grep wfb_tx | grep -v grep
  iw dev wlan0 info | grep width
'
```
Expected: one wfb_tx process running, `width: 20 MHz`.

GS (RX):
```bash
ssh root@10.18.0.1 '
  for w in wlx84fc146c36e6 wlx84fc146c36f4; do iw dev $w set freq 5660 HT20; done
  systemctl start wifibroadcast@gs.service
  sleep 3
  ss -tlnp | grep 8103
'
```
Expected: `LISTEN ... 0.0.0.0:8103 ... wfb-server`.

- [ ] **Step 2: Sample video RX for 5 s and confirm packets flow**

```bash
ssh root@10.18.0.1 'python3 << "PY"
import socket, json, time
s=socket.socket(); s.settimeout(6); s.connect(("127.0.0.1",8103))
buf=b""; t0=time.time()
while time.time()-t0 < 5:
    try: d=s.recv(65536)
    except: break
    if not d: break
    buf+=d
n=0; total_all=0; total_lost=0
for line in buf.split(b"\n"):
    line=line.strip()
    if not line: continue
    try: m=json.loads(line)
    except: continue
    if m.get("type")=="rx" and m.get("id")=="video rx":
        p=m.get("packets",{})
        a=p.get("all",[0,0])[0]; l=p.get("lost",[0,0])[0]
        total_all += a; total_lost += l
        n+=1
print(f"intervals={n} delta_packets_total={total_all} delta_lost_total={total_lost}")
PY'
```
Expected pass: `intervals >= 3`, `delta_packets_total >= 100`, `delta_lost_total <= delta_packets_total * 0.01`. If `delta_packets_total == 0`, abort — module load broke 20 MHz, revert and debug.

- [ ] **Step 3: Check dmesg on both ends for new WARNs**

```bash
ssh root@192.168.10.152 'dmesg | tail -40 | grep -iE "WARNING|BUG|oops|stack" | grep -v phy_chk_ch_setting_consistency'
ssh root@10.18.0.1   'dmesg | tail -40 | grep -iE "WARNING|BUG|oops|stack" | grep -v phy_chk_ch_setting_consistency'
```
Expected: no output on either. (Existing `phy_chk_ch_setting_consistency` WARN is filtered out — it's a pre-existing cosmetic issue.)

---

## Task 6: Regression test — 10 MHz

**Files:** none modified.

Goal: confirm the existing 10 MHz path is unaffected.

- [ ] **Step 1: Switch both ends to 10 MHz**

```bash
ssh root@192.168.10.152 'iw dev wlan0 set freq 5660 10MHz; iw dev wlan0 info | grep width'
ssh root@10.18.0.1 'for w in wlx84fc146c36e6 wlx84fc146c36f4; do iw dev $w set freq 5660 10MHz; done; for w in wlx84fc146c36e6 wlx84fc146c36f4; do iw dev $w info | grep width; done'
```
Expected: every reported width is `10 MHz`.

- [ ] **Step 2: Wait 3 s then sample video RX for 5 s**

```bash
sleep 3
ssh root@10.18.0.1 'python3 << "PY"
import socket, json, time
s=socket.socket(); s.settimeout(6); s.connect(("127.0.0.1",8103))
buf=b""; t0=time.time()
while time.time()-t0 < 5:
    try: d=s.recv(65536)
    except: break
    if not d: break
    buf+=d
n=0; total_all=0; total_lost=0
for line in buf.split(b"\n"):
    line=line.strip()
    if not line: continue
    try: m=json.loads(line)
    except: continue
    if m.get("type")=="rx" and m.get("id")=="video rx":
        p=m.get("packets",{})
        a=p.get("all",[0,0])[0]; l=p.get("lost",[0,0])[0]
        total_all += a; total_lost += l
        n+=1
print(f"intervals={n} delta_packets_total={total_all} delta_lost_total={total_lost}")
PY'
```
Expected pass: `delta_packets_total >= 100`, `delta_lost_total <= delta_packets_total * 0.01`.

- [ ] **Step 3: Check dmesg for new WARNs**

```bash
ssh root@192.168.10.152 'dmesg | tail -40 | grep -iE "WARNING|BUG|oops|stack" | grep -v phy_chk_ch_setting_consistency'
ssh root@10.18.0.1   'dmesg | tail -40 | grep -iE "WARNING|BUG|oops|stack" | grep -v phy_chk_ch_setting_consistency'
```
Expected: no output.

---

## Task 7: New capability — 5 MHz works end-to-end

**Files:** none modified.

This is the goal of the entire plan.

- [ ] **Step 1: Switch both ends to 5 MHz**

```bash
ssh root@192.168.10.152 'iw dev wlan0 set freq 5660 5MHz; iw dev wlan0 info | grep width'
ssh root@10.18.0.1 'for w in wlx84fc146c36e6 wlx84fc146c36f4; do iw dev $w set freq 5660 5MHz; done; for w in wlx84fc146c36e6 wlx84fc146c36f4; do iw dev $w info | grep width; done'
```
Expected: every reported width is `5 MHz`.

- [ ] **Step 2: Wait 3 s then sample video RX for 30 s (longer window — narrow BW has lower throughput, need more time to accumulate)**

```bash
sleep 3
ssh root@10.18.0.1 'python3 << "PY"
import socket, json, time
s=socket.socket(); s.settimeout(35); s.connect(("127.0.0.1",8103))
buf=b""; t0=time.time()
while time.time()-t0 < 30:
    try: d=s.recv(65536)
    except: break
    if not d: break
    buf+=d
n=0; total_all=0; total_lost=0
for line in buf.split(b"\n"):
    line=line.strip()
    if not line: continue
    try: m=json.loads(line)
    except: continue
    if m.get("type")=="rx" and m.get("id")=="video rx":
        p=m.get("packets",{})
        a=p.get("all",[0,0])[0]; l=p.get("lost",[0,0])[0]
        total_all += a; total_lost += l
        n+=1
print(f"intervals={n} delta_packets_total={total_all} delta_lost_total={total_lost}")
loss_pct = (total_lost / total_all * 100) if total_all else 0
print(f"loss_pct={loss_pct:.2f}%")
PY'
```
Expected pass (from spec): `delta_packets_total >= 1000` over 30 s, `loss_pct <= 1%`.

If `delta_packets_total == 0`: the fix did not work. Capture state for diagnosis:

```bash
ssh root@192.168.10.152 '
echo "=== rf_info ==="
cat /proc/net/rtl88x2eu/wlan0/rf_info
echo "=== /proc/net/dev ==="
cat /proc/net/dev | grep wlan0
echo "=== TX log tail ==="
tail -15 /tmp/wfb_tx_video.log
echo "=== dmesg tail (all) ==="
dmesg | tail -50
'
```

Likely diagnostic paths:
- TX bytes still growing in `/proc/net/dev` → drone is transmitting; suspect HALMAC clock didn't apply. Verify with `cat /proc/net/rtl88x2eu/wlan0/bw_mode` before vs. after the BW change.
- New `phy_chk_ch_setting_consistency` WARN with different backtrace → consistency check now fails on a different path; capture and report.

- [ ] **Step 3: Check dmesg for new WARNs (5 MHz path is the riskiest — pay attention)**

```bash
ssh root@192.168.10.152 'dmesg | tail -50 | grep -iE "WARNING|BUG|oops|stack" | grep -v phy_chk_ch_setting_consistency'
ssh root@10.18.0.1   'dmesg | tail -50 | grep -iE "WARNING|BUG|oops|stack" | grep -v phy_chk_ch_setting_consistency'
```
Expected: no output. If `halmac` warnings appear (e.g. from `cfg_bw_88xx` or `cfg_mac_clk_88xx`), that's new behavior worth capturing and reporting.

---

## Task 8: Transition test — 20→10→5→10→20

**Files:** none modified.

Goal: confirm idempotency and clean restore across the full ladder.

- [ ] **Step 1: Run the transition sequence with 3 s settle + 3 s sample at each step**

```bash
for bw in HT20 10MHz 5MHz 10MHz HT20; do
  echo "=== switching to $bw ==="
  ssh root@192.168.10.152 "iw dev wlan0 set freq 5660 $bw"
  ssh root@10.18.0.1 "for w in wlx84fc146c36e6 wlx84fc146c36f4; do iw dev \$w set freq 5660 $bw; done"
  sleep 3
  ssh root@10.18.0.1 'python3 << "PY"
import socket, json, time
s=socket.socket(); s.settimeout(4); s.connect(("127.0.0.1",8103))
buf=b""; t0=time.time()
while time.time()-t0 < 3:
    try: d=s.recv(65536)
    except: break
    if not d: break
    buf+=d
total_all=0
for line in buf.split(b"\n"):
    line=line.strip()
    if not line: continue
    try: m=json.loads(line)
    except: continue
    if m.get("type")=="rx" and m.get("id")=="video rx":
        total_all += m.get("packets",{}).get("all",[0,0])[0]
print(f"  packets_in_3s={total_all}")
PY'
done
```

Expected pass: each step reports `packets_in_3s > 0` (link recovered within the 3 s settle window). HT20 baselines should match Task 5 numbers; 5 MHz step should match Task 7 throughput.

- [ ] **Step 2: Check dmesg for new WARNs (only on the drone — GS is RX-only here)**

```bash
ssh root@192.168.10.152 'dmesg | tail -60 | grep -iE "WARNING|BUG|oops|stack" | grep -v phy_chk_ch_setting_consistency'
```
Expected: no output.

---

## Task 9: Stress — rapid 5↔20 transitions

**Files:** none modified.

Goal: catch DMA stalls or HALMAC state corruption that only manifest under rapid reconfig.

- [ ] **Step 1: Run 10 rapid transitions, 1 s apart, no settle window**

```bash
for i in 1 2 3 4 5; do
  ssh root@192.168.10.152 "iw dev wlan0 set freq 5660 5MHz"
  ssh root@10.18.0.1 "for w in wlx84fc146c36e6 wlx84fc146c36f4; do iw dev \$w set freq 5660 5MHz; done"
  sleep 1
  ssh root@192.168.10.152 "iw dev wlan0 set freq 5660 HT20"
  ssh root@10.18.0.1 "for w in wlx84fc146c36e6 wlx84fc146c36f4; do iw dev \$w set freq 5660 HT20; done"
  sleep 1
done
echo "=== final state ==="
ssh root@192.168.10.152 'iw dev wlan0 info | grep -E "channel|width"'
ssh root@10.18.0.1 'for w in wlx84fc146c36e6 wlx84fc146c36f4; do iw dev $w info | grep -E "channel|width"; done'
```
Expected pass: all final widths are `20 MHz`. No SSH hangs (would indicate kernel hang).

- [ ] **Step 2: Sample video for 5 s at the final 20 MHz state and confirm link recovered**

```bash
sleep 3
ssh root@10.18.0.1 'python3 << "PY"
import socket, json, time
s=socket.socket(); s.settimeout(6); s.connect(("127.0.0.1",8103))
buf=b""; t0=time.time()
while time.time()-t0 < 5:
    try: d=s.recv(65536)
    except: break
    if not d: break
    buf+=d
n=0; total_all=0
for line in buf.split(b"\n"):
    line=line.strip()
    if not line: continue
    try: m=json.loads(line)
    except: continue
    if m.get("type")=="rx" and m.get("id")=="video rx":
        n+=1; total_all += m.get("packets",{}).get("all",[0,0])[0]
print(f"intervals={n} delta_packets_total={total_all}")
PY'
```
Expected: `delta_packets_total >= 100`.

- [ ] **Step 3: Check dmesg on both ends — this is the most likely place for new WARNs to appear**

```bash
ssh root@192.168.10.152 'dmesg | tail -80 | grep -iE "WARNING|BUG|oops|stack|stall|halmac.*err" | grep -v phy_chk_ch_setting_consistency'
ssh root@10.18.0.1   'dmesg | tail -80 | grep -iE "WARNING|BUG|oops|stack|stall|halmac.*err" | grep -v phy_chk_ch_setting_consistency'
```
Expected: no output. If output appears, capture full dmesg from each host (`dmesg > /tmp/dmesg-<host>.log`) and stop — investigate before claiming pass.

---

## Task 10: Final cleanup / fixup commit (only if Task 7-9 required code adjustments)

**Files:** depends on what fixups were needed.

Skip this task if Tasks 5–9 all passed first try.

- [ ] **Step 1: If any test failed and required a code fix, the fix should already be a separate commit. Verify git log shows the sequence:**

```bash
git log --oneline -5
```
Expected (if no fixups needed): the Task 1 commit, then earlier commits.
Expected (if fixups needed): Task 1 commit + one or more `fixup:` commits, each scoped to a specific test failure.

- [ ] **Step 2: Restore working state for the user (10 MHz, video flowing, wfb-server running normally)**

```bash
ssh root@192.168.10.152 'iw dev wlan0 set freq 5660 10MHz'
ssh root@10.18.0.1 'for w in wlx84fc146c36e6 wlx84fc146c36f4; do iw dev $w set freq 5660 10MHz; done'
sleep 3
ssh root@10.18.0.1 'python3 -c "
import socket, json, time
s=socket.socket(); s.settimeout(4); s.connect((\"127.0.0.1\",8103))
buf=b\"\"; t0=time.time()
while time.time()-t0<3:
    try: d=s.recv(65536)
    except: break
    if not d: break
    buf+=d
for line in buf.split(b\"\\n\"):
    line=line.strip()
    if not line: continue
    try: m=json.loads(line)
    except: continue
    if m.get(\"type\")==\"rx\" and m.get(\"id\")==\"video rx\":
        p=m.get(\"packets\",{})
        print(\"video rx all=%s lost=%s\" % (p.get(\"all\"),p.get(\"lost\"))); break
"'
```
Expected: `video rx all=[...,...] lost=[0,...]` with non-zero first element of `all`.
