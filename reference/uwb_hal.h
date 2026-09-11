/*
 * uwb_hal.h — HAL tối thiểu cho ranging DS-TWR (vai RX/Anchor).
 *
 * Gói toàn bộ chuỗi state PREPOLL->POLL->RESPOND->FINAL->FINALDATA->IDLE (vốn
 * là switch-case blocking trong loop() của main.cpp gốc) thành 2 hàm blocking
 * để FSM/BLE (chạy cùng board) gọi trực tiếp:
 *
 *   bool  uwb_init();          // gọi 1 lần trong setup()
 *   float uwb_get_distance();  // chạy N chu kỳ đo, lọc median, trả cm (-1 nếu fail)
 *
 * Thuật toán DS-TWR/STS và driver DW3000.cpp/.h không đổi — file này chỉ tái
 * cấu trúc luồng điều khiển đã có trong main.cpp gốc.
 */
#ifndef UWB_HAL_H
#define UWB_HAL_H

#include <Arduino.h>

// Số chu kỳ đo mỗi lần uwb_get_distance() thực hiện để lọc median.
#ifndef UWB_MEASURE_SAMPLES
#define UWB_MEASURE_SAMPLES 5
#endif

// Số lần PREPOLL được phép tự lặp lại (mỗi lần là 1 cửa sổ timeout 5s chờ TX,
// xem case PREPOLL trong uwb_hal.cpp) trước khi 1 lần đo đơn bị coi là thất
// bại. Không đổi giá trị 5s, chỉ chặn số lần lặp lại ở tầng ngoài.
#ifndef UWB_PREPOLL_MAX_RETRY
#define UWB_PREPOLL_MAX_RETRY 1
#endif

// Số lần hardReset được phép thử lại khi chờ chip vào IDLE trong uwb_init()
// trước khi bỏ cuộc và trả về false. Không đổi các mốc thời gian bên trong
// (100ms/lần check, hardReset sau 50 lần, delay 500ms) — chỉ chặn tổng số lần.
#ifndef UWB_INIT_MAX_HARDRESET_RETRY
#define UWB_INIT_MAX_HARDRESET_RETRY 5
#endif

// Khởi tạo DW3000 (SPI, reset, cấu hình, crypto STS...). Gọi 1 lần trong setup().
// Trả về false nếu chip không vào IDLE được sau UWB_INIT_MAX_HARDRESET_RETRY lần
// hardReset (chip lỗi/không kết nối).
bool uwb_init();

// Chạy UWB_MEASURE_SAMPLES chu kỳ ranging DS-TWR đầy đủ (responder), lọc median
// để loại outlier. Blocking. Trả về khoảng cách (cm), hoặc -1 nếu quá nửa số
// mẫu thất bại (timeout / không có TX / lỗi frame).
float uwb_get_distance();

#endif
