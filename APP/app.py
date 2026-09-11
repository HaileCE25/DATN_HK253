import os
import time
from flask import Flask, render_template, request, redirect, flash, jsonify
import firebase_admin
from firebase_admin import credentials, db
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

app = Flask(__name__)
app.secret_key = "smart_car_secret_key"

# Khởi tạo Firebase (Nhớ đảm bảo file serviceAccountKey.json đã chuẩn)
cred = credentials.Certificate("serviceAccountKey.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://smart-car-rental-b2b-default-rtdb.asia-southeast1.firebasedatabase.app'
})


MASTER_KEY = b"MySuperSecretMasterKey32Bytes!!!" 

@app.route('/')
def index():
    vehicles_ref = db.reference('Vehicles')
    nfc_ref = db.reference('NfcWhitelist')
    return render_template(
        'index.html',
        vehicles=vehicles_ref.get() or {},
        nfc_whitelist=nfc_ref.get() or {}
    )

@app.route('/add_vehicle', methods=['POST'])
def add_vehicle():
    try:
        car_id = request.form.get('car_id').strip()
        key_root_plain = request.form.get('key_root').strip() # Key trần nhận từ Web
        
        if not key_root_plain:
            flash("Lỗi: Không nhận được Key root từ giao diện Web.", "error")
            return redirect('/')

        # 1. MÃ HÓA KEY_ROOT BẰNG AES-256-CTR
        # Tạo IV (Initialization Vector) ngẫu nhiên 16 bytes. IV giống như "muối" rắc thêm vào để mỗi lần mã hóa ra một kết quả khác nhau, chống bị hacker đoán ngược.
        iv = os.urandom(16)
        cipher = Cipher(algorithms.AES(MASTER_KEY), modes.CTR(iv))
        encryptor = cipher.encryptor()
        
        # Băm key_root thành các ký tự lộn xộn
        ciphertext = encryptor.update(key_root_plain.encode('utf-8')) + encryptor.finalize()

        # 2. ĐẨY GÓI TIN ĐÃ MÃ HÓA LÊN FIREBASE
        secure_ref = db.reference(f'SecureKeys/{car_id}')
        secure_ref.set({
            'iv': iv.hex(),                       # Gửi IV lên (IV không cần giấu)
            'encrypted_key_root': ciphertext.hex(), # Dữ liệu đã bị mã hóa nát bét
            'status': 'WAITING_ESP32_DECRYPT'
        })
        
        # 3. LƯU THÔNG TIN XE NHƯ BÌNH THƯỜNG
        db.reference(f'Vehicles/{car_id}').update({
            'provisioned_status': 'Ready',
            'car_model': request.form.get('car_model'),
            'license_plate': request.form.get('license_plate'),
            'color': request.form.get('color'),
            'seats': request.form.get('seats'),
            'status': 'AVAILABLE',
            'door_status': 'Locked',
            'engine_status': 'OFF'
        })

        flash(f"Đã mã hóa an toàn và đẩy Key của {car_id} lên Firebase!", "success")
        
    except Exception as e:
        flash(f"Có lỗi xảy ra: {str(e)}", "error")

    return redirect('/')


# ============ QUẢN LÝ WHITELIST NFC (đồng bộ với Car qua Serial riêng) ============
# Các route này CHỈ ghi/xoá trên Firebase - phần ghi/xoá thật trên NVS
# của Car do JS xử lý qua Web Serial (xem index.html), gọi các route
# này SAU KHI Car đã xác nhận SUCCESS, để tránh Firebase và NVS lệch
# nhau nếu 1 trong 2 bước thất bại giữa chừng.

@app.route('/add_nfc_uid', methods=['POST'])
def add_nfc_uid():
    car_id = (request.form.get('car_id') or '').strip()
    uid = (request.form.get('uid') or '').strip().upper()

    if not car_id or not uid:
        return jsonify({"success": False, "error": "Thieu car_id hoac uid"}), 400

    try:
        db.reference(f'NfcWhitelist/{car_id}/{uid}').set({
            'added_at': int(time.time())
        })
        return jsonify({"success": True})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route('/remove_nfc_uid', methods=['POST'])
def remove_nfc_uid():
    car_id = (request.form.get('car_id') or '').strip()
    uid = (request.form.get('uid') or '').strip().upper()

    if not car_id or not uid:
        return jsonify({"success": False, "error": "Thieu car_id hoac uid"}), 400

    try:
        db.reference(f'NfcWhitelist/{car_id}/{uid}').delete()
        return jsonify({"success": True})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


if __name__ == '__main__':
    app.run(debug=True, port=5000)