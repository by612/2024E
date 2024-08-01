import RPi.GPIO as GPIO
import time
from threading import Event
from control import question2

KEY_PLUS = 17
KEY_SUB = 27
KEY_ENTER = 22
BOUNCE_TIME = 100  # 按键消抖时间，单位为毫秒

GPIO.setmode(GPIO.BCM)
GPIO.setup(KEY_PLUS, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY_SUB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY_ENTER, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def select_grid():
    # 设置GPIO模式
    current_grid = 0
    grid_selected = Event()

    def plus_callback(channel):
        nonlocal current_grid
        if current_grid < 9:
            current_grid += 1
        print(f"当前选择的棋格: {current_grid}")

    def sub_callback(channel):
        nonlocal current_grid
        if current_grid > 1:
            current_grid -= 1
        print(f"当前选择的棋格: {current_grid}")

    def enter_callback(channel):
        nonlocal current_grid
        print(f"选择的棋格: {current_grid}")
        # question2(current_grid)  # 调用对应的机械臂操作函数
        grid_selected.set()

    # 清理旧的事件检测设置
    GPIO.remove_event_detect(KEY_PLUS)
    GPIO.remove_event_detect(KEY_SUB)
    GPIO.remove_event_detect(KEY_ENTER)

    # 设置按键事件检测，并添加消抖时间
    GPIO.add_event_detect(KEY_PLUS, GPIO.FALLING, callback=plus_callback, bouncetime=BOUNCE_TIME)
    GPIO.add_event_detect(KEY_SUB, GPIO.FALLING, callback=sub_callback, bouncetime=BOUNCE_TIME)
    GPIO.add_event_detect(KEY_ENTER, GPIO.FALLING, callback=enter_callback, bouncetime=BOUNCE_TIME)

    print("按下按键二和按键三选择1到9之间的棋格，按按键一完成选择")

    try:
        grid_selected.wait()  # 等待用户按下KEY_ENTER键
        return current_grid
    except KeyboardInterrupt:
        print("\n被用户中断的程序")

