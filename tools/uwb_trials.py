"""Thống kê từng lần thử unlock/relock UWB từ log của Car.

Ghép 2 file do monitor/filter_uwb_raw.py ghi (cùng mốc thời gian):
    logs/uwb_raw-<mốc>.csv   ms,seq,raw_cm,med_cm,filt_cm,alpha,flag   (mọi round)
    logs/uwb_evt-<mốc>.csv   ms,seq,event,dist_m,count,dt_ms          (sự kiện FSM)

Mỗi "lần thử" bắt đầu ở một sự kiện START (sau BLE auth) hoặc RESUME (sau cooldown
relock) và kéo dài tới lần thử kế tiếp.

Cách dùng:
    python tools/uwb_trials.py                          # cặp file mới nhất trong logs/
    python tools/uwb_trials.py logs/uwb_raw-A.csv logs/uwb_raw-B.csv   # gộp nhiều phiên monitor

Kết quả (cạnh file raw đầu tiên, trong logs/):
    uwb_trials-<mốc>-samples.csv   từng mẫu: lần thử, thời gian, mẫu thứ, raw, distance, trạng thái FSM, sự kiện
    uwb_trials-<mốc>-summary.csv   mỗi lần thử 1 dòng (bảng thống kê)
và in bảng tóm tắt + trung bình/min/max ra màn hình.
"""
import csv
import glob
import os
import re
import statistics
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS = os.path.join(ROOT, "logs")


def read_threshold(name, default):
    """Đọc hằng số từ src/car/main.cpp để bảng luôn khớp firmware."""
    try:
        with open(os.path.join(ROOT, "src", "car", "main.cpp"), encoding="utf-8") as f:
            m = re.search(r"constexpr\s+\w+\s+%s\s*=\s*([\d.]+)f?" % name, f.read())
        return float(m.group(1)) if m else default
    except OSError:
        return default


R_UNLOCK = read_threshold("UWB_R_UNLOCK", 1.5)  # m: mẫu < mức này là trong vùng unlock
R_LOCK = read_threshold("UWB_R_LOCK", 3.5)      # m: mẫu >= mức này là chạm ngưỡng relock
N_UNLOCK = int(read_threshold("UWB_N_UNLOCK", 12))  # mở khi >= N trong W mẫu gần nhất < R_UNLOCK
W_UNLOCK = int(read_threshold("UWB_W_UNLOCK", 15))

# Trạng thái FSM sau mỗi sự kiện (áp cho chính mẫu gây ra sự kiện)
STATE_AFTER = {
    "START": "TRACKING",
    "RESUME": "TRACKING",
    "ENTER": "UNLOCK_WINDOW",
    "EXIT": "TRACKING",
    "UNLOCK": "UNLOCKED",
    "RELOCK": "COOLDOWN",
    "FAILSAFE": "COOLDOWN",
}


def evt_path_for(raw_path):
    d, name = os.path.split(raw_path)
    return os.path.join(d, name.replace("uwb_raw-", "uwb_evt-", 1))


def latest_raw():
    for p in sorted(glob.glob(os.path.join(LOGS, "uwb_raw-*.csv")), reverse=True):
        if os.path.exists(evt_path_for(p)):
            return p
    sys.exit("Khong tim thay cap uwb_raw-*.csv / uwb_evt-*.csv trong logs/ "
             "(can firmware Car moi co [UWBEVT] va monitor filter uwb_raw)")


def read_csv(path):
    with open(path, encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def to_int(v):
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def to_float(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load_session(raw_path, session_idx):
    """Đọc 1 cặp file, trả danh sách lần thử (mỗi lần thử: sự kiện + mẫu của nó)."""
    evt_path = evt_path_for(raw_path)
    if not os.path.exists(evt_path):
        sys.exit("Thieu file su kien: %s" % evt_path)

    raws = []
    for r in read_csv(raw_path):
        ms, seq = to_int(r.get("ms")), to_int(r.get("seq"))
        if ms is None or seq is None:
            continue  # dòng định dạng cũ (không có seq) hoặc hỏng
        fail = r.get("raw_cm") == "FAIL"
        raws.append({
            "ms": ms, "seq": seq, "fail": fail,
            "raw_cm": None if fail else to_float(r.get("raw_cm")),
            "filt_cm": None if fail else to_float(r.get("filt_cm")),
            "flag": None if fail else to_int(r.get("flag")),
            "fail_state": r.get("med_cm") if fail else "",
        })

    evts = []
    for e in read_csv(evt_path):
        ms = to_int(e.get("ms"))
        if ms is None:
            continue
        evts.append({
            "ms": ms, "seq": to_int(e.get("seq")) or 0, "event": e.get("event", ""),
            "dist_m": to_float(e.get("dist_m")), "count": to_int(e.get("count")),
            "dt_ms": to_int(e.get("dt_ms")),
        })

    starts = [e for e in evts if e["event"] in ("START", "RESUME")]
    trials = []
    for i, st in enumerate(starts):
        end_ms = starts[i + 1]["ms"] if i + 1 < len(starts) else None
        in_trial = lambda x: x["ms"] >= st["ms"] and (end_ms is None or x["ms"] < end_ms)
        trials.append({
            "session": session_idx,
            "file": os.path.basename(raw_path),
            "start": st,
            "events": [e for e in evts if in_trial(e) and e is not st],
            "samples": [r for r in raws if in_trial(r)],
        })
    return trials


def annotate(trial):
    """Gắn trạng thái FSM + sự kiện vào từng mẫu thành công của lần thử."""
    by_seq = {}
    for e in trial["events"]:
        by_seq.setdefault(e["seq"], []).append(e["event"])
    state = "TRACKING"
    rows = []
    t0 = trial["start"]["ms"]
    for s in trial["samples"]:
        ev = ""
        if not s["fail"]:
            names = by_seq.get(s["seq"], [])
            for n in names:
                state = STATE_AFTER.get(n, state)
            ev = "+".join(n for n in names if n != "FIRST")
        rows.append({
            "t_ms": s["ms"] - t0, "ms": s["ms"], "seq": s["seq"] if not s["fail"] else "",
            "raw_m": "FAIL" if s["fail"] else "%.3f" % (s["raw_cm"] / 100.0),
            "filt_m": "" if s["fail"] else "%.3f" % (s["filt_cm"] / 100.0),
            "fsm": "" if s["fail"] else state,
            "event": ev if not s["fail"] else s["fail_state"],
        })
    return rows


def first(events, name):
    for e in events:
        if e["event"] == name:
            return e
    return None


def summarize(n, trial):
    ev, st, samples = trial["events"], trial["start"], trial["samples"]
    ok = [x for x in samples if not x["fail"]]
    by_seq = {x["seq"]: x for x in ok}
    unlock = first(ev, "UNLOCK")
    relock = first(ev, "RELOCK")
    failsafe = first(ev, "FAILSAFE")
    first_s = first(ev, "FIRST")
    end_lock = relock or failsafe

    def fmt(v, spec="%d"):
        return "" if v is None else spec % v

    # ── Unlock: cửa sổ W mẫu gần nhất kết thúc ở mẫu gây UNLOCK, trong đó >= N mẫu
    # < R_UNLOCK. win = các mẫu TRONG VÙNG của cửa sổ (đúng N mẫu làm mở khoá),
    # out_win = số mẫu nhiễu vọt ra ngoài vùng trong cửa sổ (được bỏ qua).
    # t_UL: từ mẫu gây ENTER cuối cùng trước UNLOCK (chạm ngưỡng) tới mẫu gây UNLOCK.
    win = []
    out_win = None
    t_ul = None
    if unlock:
        enters = [e for e in ev if e["event"] == "ENTER" and e["ms"] <= unlock["ms"]]
        lo = unlock["seq"] - W_UNLOCK + 1
        if enters:
            lo = max(lo, enters[-1]["seq"])
        span = [x for x in ok if lo <= x["seq"] <= unlock["seq"]]
        win = [x for x in span if x["filt_cm"] / 100.0 < R_UNLOCK]
        out_win = len(span) - len(win)
        if enters and by_seq.get(enters[-1]["seq"]) and by_seq.get(unlock["seq"]):
            t_ul = by_seq[unlock["seq"]]["ms"] - by_seq[enters[-1]["seq"]]["ms"]

    # ── Relock: mẫu CUỐI còn < R_UNLOCK sau khi mở -> mẫu ĐẦU chạm >= R_LOCK ──
    t_rl = t_touch_to_rl = None
    last_in = touch = None
    if unlock and relock:
        after = [x for x in ok if unlock["seq"] <= x["seq"] <= relock["seq"]]
        ins = [x for x in after if x["filt_cm"] / 100.0 < R_UNLOCK]
        if ins:
            last_in = ins[-1]
            touch = next((x for x in after if x["seq"] > last_in["seq"]
                          and x["filt_cm"] / 100.0 >= R_LOCK), None)
        if last_in and touch:
            t_rl = touch["ms"] - last_in["ms"]
            rl_sample = by_seq.get(relock["seq"])
            t_touch_to_rl = (rl_sample["ms"] if rl_sample else relock["ms"]) - touch["ms"]

    m = lambda x: x / 100.0
    return {
        "trial": n,
        "file": trial["file"],
        "type": st["event"],
        "connect_to_start_ms": fmt(st["dt_ms"]) if st["event"] == "START" else "",
        "start_to_first_ms": fmt(first_s["dt_ms"]) if first_s else "",
        "unlock": "Y" if unlock else "N",
        # N mẫu trong vùng quyết định mở khoá (khoảng cách đã lọc, m) - mẫu cuối là mẫu gây UNLOCK
        "unlock_win_filt_m": " ".join("%.2f" % m(x["filt_cm"]) for x in win),
        "unlock_win_raw_m": " ".join("%.2f" % m(x["raw_cm"]) for x in win),
        "unlock_win_raw_mean_m": fmt(statistics.mean(m(x["raw_cm"]) for x in win), "%.2f") if win else "",
        "unlock_last_filt_m": fmt(m(win[-1]["filt_cm"]), "%.2f") if win else "",
        "unlock_win_out": fmt(out_win),
        "t_ul_ms": fmt(t_ul),
        "exit_resets": sum(1 for e in ev if e["event"] == "EXIT" and (not unlock or e["ms"] <= unlock["ms"])),
        "lock_by": end_lock["event"] if end_lock else "",
        "last_in_unlock_m": fmt(m(last_in["filt_cm"]), "%.2f") if last_in else "",
        "touch_relock_m": fmt(m(touch["filt_cm"]), "%.2f") if touch else "",
        "t_rl_ms": fmt(t_rl),
        "touch_to_relock_ms": fmt(t_touch_to_rl),
        "relock_dist_m": fmt(relock["dist_m"], "%.2f") if relock else "",
        "samples_ok": len(ok),
        "rounds_fail": sum(1 for x in samples if x["fail"]),
    }


# Bảng in ra terminal: tiêu đề ngắn, số căn phải, để vừa ~100 cột không bị xuống dòng.
# (tiêu đề, cột trong summary, hàm đổi giá trị -> chuỗi). Bảng đầy đủ nằm trong file summary CSV.
def _s(v):  # ms -> giây, 1 chữ số thập phân
    return "%.1f" % (float(v) / 1000.0) if v not in ("", None) else ""


TERM_COLS = [
    ("#",               "trial",                 str),
    ("%d mau unlock (m, da loc)" % N_UNLOCK, "unlock_win_filt_m", str),
    ("Nhieu",           "unlock_win_out",        str),
    ("t UL(ms)",        "t_ul_ms",               str),
    ("Khoa",            "lock_by",               lambda v: {"RELOCK": "RL", "FAILSAFE": "FS"}.get(v, v or "-")),
    ("t RL(s)",         "t_rl_ms",               _s),
    ("Cham->RL(ms)",    "touch_to_relock_ms",    str),
    ("d RL(m)",         "relock_dist_m",         str),
]

LEGEND = """Chu thich (R_UNLOCK = %.2f m, R_LOCK = %.2f m, unlock khi >= %d/%d mau gan nhat < R_UNLOCK;
           doc tu src/car/main.cpp):
  %d mau unlock  khoang cach DA LOC cua cac mau < R_UNLOCK trong cua so %d mau luc mo khoa;
                 mau cuoi = mau lam mo khoa (FSM quyet dinh theo gia tri da loc)
  Nhieu          so mau trong cua so do bi vot ra ngoai R_UNLOCK (duoc bo qua, toi da %d)
  t UL           tu mau dau (cham R_UNLOCK) toi mau cuoi (mo khoa), theo thoi diem do
  Khoa           RL = relock theo khoang cach, FS = fail-safe mat BLE
  t RL           tu mau CUOI con < R_UNLOCK (sau khi mo) toi mau DAU cham >= R_LOCK (thoi gian di ra)
  Cham->RL       tu mau dau cham >= R_LOCK toi luc relock (do tre he thong phia relock)
  d RL           khoang cach (da loc) cua mau lam relock""" % (
    R_UNLOCK, R_LOCK, N_UNLOCK, W_UNLOCK, N_UNLOCK, W_UNLOCK, W_UNLOCK - N_UNLOCK)


def print_table(rows):
    cells = [[fn(r[key]) or "-" for _, key, fn in TERM_COLS] for r in rows]
    heads = [h for h, _, _ in TERM_COLS]
    widths = [max(len(heads[i]), *(len(c[i]) for c in cells)) for i in range(len(heads))]
    left = [key == "unlock_win_filt_m" for _, key, _ in TERM_COLS]
    just = lambda v, w, l: v.ljust(w) if l else v.rjust(w)
    print("  ".join(just(h, w, l) for h, w, l in zip(heads, widths, left)))
    print("  ".join("-" * w for w in widths))
    for c in cells:
        print("  ".join(just(v, w, l) for v, w, l in zip(c, widths, left)))


def main():
    raw_paths = sys.argv[1:] or [latest_raw()]
    trials = []
    for i, p in enumerate(raw_paths, 1):
        trials += load_session(p, i)
    if not trials:
        sys.exit("Khong co lan thu nao (khong thay su kien START/RESUME)")

    stamp = os.path.basename(raw_paths[0]).replace("uwb_raw-", "").replace(".csv", "")
    out_dir = os.path.dirname(os.path.abspath(raw_paths[0]))
    samples_path = os.path.join(out_dir, "uwb_trials-%s-samples.csv" % stamp)
    summary_path = os.path.join(out_dir, "uwb_trials-%s-summary.csv" % stamp)

    summary = []
    with open(samples_path, "w", encoding="utf-8", newline="") as f:
        cols = ["trial", "t_ms", "ms", "seq", "raw_m", "filt_m", "fsm", "event"]
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for n, t in enumerate(trials, 1):
            for r in annotate(t):
                w.writerow(dict(r, trial=n))
            summary.append(summarize(n, t))

    with open(summary_path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)

    print("Nguon: %s" % ", ".join(os.path.basename(p) for p in raw_paths))
    print()
    print_table(summary)
    print()
    print(LEGEND)

    print()
    print("Tong hop (%d lan thu, %d lan unlock):" % (len(summary), sum(r["unlock"] == "Y" for r in summary)))
    for c in ["t_ul_ms", "unlock_last_filt_m", "t_rl_ms",
              "touch_to_relock_ms", "relock_dist_m", "connect_to_start_ms", "start_to_first_ms"]:
        vals = [float(r[c]) for r in summary if r[c] not in ("", None)]
        if vals:
            print("  %-24s TB=%9.2f  min=%9.2f  max=%9.2f  (n=%d)"
                  % (c, statistics.mean(vals), min(vals), max(vals), len(vals)))
    print()
    print("Da ghi: %s" % samples_path)
    print("        %s" % summary_path)


if __name__ == "__main__":
    main()
