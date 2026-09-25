"""Bộ lọc `pio device monitor`: tách luồng dữ liệu thô UWB khỏi log thường.

Firmware Car in mỗi round đo thành một dòng có thẻ:
    [UWBRAW],<ms>,<raw_cm>,<med_cm>,<filt_cm>,<alpha>,<flag>   round thành công
    [UWBRAW],<ms>,FAIL,<state>            round lỗi

Bộ lọc này:
  * ghi các dòng đó vào  logs/uwb_raw-<ngày>-<giờ>.csv  (mở file mới mỗi lần chạy monitor),
  * loại chúng khỏi luồng đi tiếp, nên màn hình và file log2file chỉ còn log đọc được.

Dùng trong platformio.ini (đặt TRƯỚC log2file):
    monitor_filters = uwb_raw, log2file, default
"""
import os
import time

from platformio.public import DeviceMonitorFilterBase

TAG = "[UWBRAW]"


class UwbRaw(DeviceMonitorFilterBase):
    NAME = "uwb_raw"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._carry = ""
        self._fh = None

    # -- ghi file -----------------------------------------------------------
    def _open(self):
        if self._fh is not None:
            return
        logs_dir = os.path.join(self.project_dir, "logs")
        os.makedirs(logs_dir, exist_ok=True)
        name = "uwb_raw-%s.csv" % time.strftime("%Y%m%d-%H%M%S")
        self._fh = open(os.path.join(logs_dir, name), "w", encoding="utf-8", newline="")
        # flag: 0 nhận bình thường | 1 bỏ mẫu (chặn nhảy vọt) | 2 khởi tạo lại | 3 chấp nhận nhảy thật
        # alpha: hệ số EMA đã dùng (0 = mẫu bị bỏ, 1 = khởi tạo lại)
        self._fh.write("ms,raw_cm,med_cm,filt_cm,alpha,flag\n")

    def _write(self, line):
        self._open()
        # bỏ thẻ "[UWBRAW]," rồi ghi phần CSV
        self._fh.write(line[len(TAG) + 1:] + "\n")
        self._fh.flush()

    # -- xử lý dữ liệu nhận về ----------------------------------------------
    def rx(self, text):
        data = self._carry + text
        self._carry = ""
        parts = data.split("\n")
        tail = parts.pop()  # phần cuối có thể chưa đủ dòng
        out = []
        for ln in parts:
            s = ln.rstrip("\r")
            if s.startswith(TAG):
                self._write(s)
            else:
                out.append(ln + "\n")
        # Chỉ giữ lại phần cuối nếu nó có thể là đầu của một dòng [UWBRAW]; dòng khác
        # thì cho đi tiếp ngay để không làm trễ log thường.
        if tail and (TAG.startswith(tail) or tail.startswith(TAG)):
            self._carry = tail
        else:
            out.append(tail)
        return "".join(out)

    def tx(self, text):
        return text
