"""Bộ lọc `pio device monitor`: tách luồng dữ liệu UWB khỏi log thường.

Firmware Car in hai luồng có thẻ:
    [UWBRAW],<ms>,<seq>,<raw_cm>,<med_cm>,<filt_cm>,<alpha>,<flag>   round thành công
    [UWBRAW],<ms>,<seq>,FAIL,<state>                                 round lỗi
    [UWBEVT],<ms>,<seq>,<event>,<dist_m>,<count>,<dt_ms>             sự kiện FSM (unlock/relock...)
seq = số thứ tự mẫu thành công trong phiên ranging (về 0 mỗi lần START/RESUME).

Bộ lọc này:
  * ghi [UWBRAW] vào  logs/uwb_raw-<ngày>-<giờ>.csv  và [UWBEVT] vào  logs/uwb_evt-<cùng mốc>.csv
    (mở cặp file mới mỗi lần chạy monitor),
  * loại các dòng đó khỏi luồng đi tiếp, nên màn hình và file log2file chỉ còn log đọc được.
Ghép 2 file thành bảng thống kê từng lần thử: python tools/uwb_trials.py

Dùng trong platformio.ini (đặt TRƯỚC log2file):
    monitor_filters = uwb_raw, log2file, default
"""
import os
import time

from platformio.public import DeviceMonitorFilterBase

STREAMS = {
    # thẻ: (tiền tố tên file, header CSV)
    # flag: 0 nhận bình thường | 1 bỏ mẫu (chặn nhảy vọt) | 2 khởi tạo lại | 3 chấp nhận nhảy thật
    # alpha: hệ số EMA đã dùng (0 = mẫu bị bỏ, 1 = khởi tạo lại)
    "[UWBRAW]": ("uwb_raw", "ms,seq,raw_cm,med_cm,filt_cm,alpha,flag\n"),
    "[UWBEVT]": ("uwb_evt", "ms,seq,event,dist_m,count,dt_ms\n"),
}


class UwbRaw(DeviceMonitorFilterBase):
    NAME = "uwb_raw"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._carry = ""
        self._stamp = None
        self._fh = {}

    # -- ghi file -----------------------------------------------------------
    def _file(self, tag):
        fh = self._fh.get(tag)
        if fh is not None:
            return fh
        if self._stamp is None:
            self._stamp = time.strftime("%Y%m%d-%H%M%S")
        prefix, header = STREAMS[tag]
        logs_dir = os.path.join(self.project_dir, "logs")
        os.makedirs(logs_dir, exist_ok=True)
        name = "%s-%s.csv" % (prefix, self._stamp)
        fh = open(os.path.join(logs_dir, name), "w", encoding="utf-8", newline="")
        fh.write(header)
        self._fh[tag] = fh
        return fh

    def _write(self, tag, line):
        fh = self._file(tag)
        # bỏ thẻ "[UWBxxx]," rồi ghi phần CSV
        fh.write(line[len(tag) + 1:] + "\n")
        fh.flush()

    @staticmethod
    def _tag_of(line):
        for tag in STREAMS:
            if line.startswith(tag):
                return tag
        return None

    @staticmethod
    def _maybe_tag_prefix(tail):
        return any(tag.startswith(tail) or tail.startswith(tag) for tag in STREAMS)

    # -- xử lý dữ liệu nhận về ----------------------------------------------
    def rx(self, text):
        data = self._carry + text
        self._carry = ""
        parts = data.split("\n")
        tail = parts.pop()  # phần cuối có thể chưa đủ dòng
        out = []
        for ln in parts:
            s = ln.rstrip("\r")
            tag = self._tag_of(s)
            if tag:
                self._write(tag, s)
            else:
                out.append(ln + "\n")
        # Chỉ giữ lại phần cuối nếu nó có thể là đầu của một dòng có thẻ; dòng khác
        # thì cho đi tiếp ngay để không làm trễ log thường.
        if tail and self._maybe_tag_prefix(tail):
            self._carry = tail
        else:
            out.append(tail)
        return "".join(out)

    def tx(self, text):
        return text
