Import("env")

board_config = env.BoardConfig()
board_config.update("build.hwids", [
    ["0x9898", #VID
     "0x8989" #PID
    ]
])
board_config.update("build.usb_product", "Nivek's SUPA DUPA Controller")