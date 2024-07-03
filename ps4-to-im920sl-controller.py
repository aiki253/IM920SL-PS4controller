import pygame
import serial
import time

def map_axis_to_range(value, old_min, old_max, new_min, new_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    return int(((value - old_min) * new_range) / old_range + new_min)

# IM920sLとのシリアル通信設定
im920sl_ser = serial.Serial('/dev/ttyUSB0', '19200', timeout=None)

# Pygameの初期化
pygame.init()

# ジョイスティックの初期化
pygame.joystick.init()

# ジョイスティックの数を取得
joystick_count = pygame.joystick.get_count()

if joystick_count > 0:
    # ジョイスティックのインスタンスを取得
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # ジョイスティックの名前を表示
    print(f"Joystick name: {joystick.get_name()}")

    # 前回の送信時間
    last_send_time = time.time()

    while True:
        # イベントを処理する
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        # ボタンの状態を取得
        buttons = sum(joystick.get_button(i) << i for i in range(joystick.get_numbuttons()))
        buttons_hex = f"{buttons:04X}"

        # ジョイスティックの軸の状態を取得し、0から255の範囲に変換
        axes = [map_axis_to_range(joystick.get_axis(i), -1, 1, 0, 255) for i in range(joystick.get_numaxes())]
        axes_hex = "".join(f"{axis:02X}" for axis in axes)

        # 十字キーの状態を取得
        hat = joystick.get_hat(0)
        hats_hex = {
            (-1, -1): "0000", (0, -1): "8000", (1, -1): "FF00",
            (-1,  0): "0080", (0,  0): "8080", (1,  0): "FF80",
            (-1,  1): "00FF", (0,  1): "80FF", (1,  1): "FFFF"
        }.get(hat, "8080")  # デフォルトは中央位置

        # 結合して表示
        controller_data = buttons_hex + axes_hex + hats_hex
        # print(controller_data)

        # 0.052秒経過したらデータを送信
        current_time = time.time()
        if current_time - last_send_time >= 0.052:
            if controller_data is not None:
                im920sl_ser.write(f'TXDA{controller_data}\r'.encode('utf-8'))
                print(controller_data)

            # IM920sLからの応答を受信
            im920sl_response = im920sl_ser.readline().decode('utf-8').rstrip()
            print(f"Response from IM920SL: {im920sl_response}")

            # 最後の入力データをクリア
            controller_data = None

            last_send_time = current_time

else:
    print("No joysticks found.")

# Pygameの終了
pygame.quit()
