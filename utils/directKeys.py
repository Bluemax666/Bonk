# direct inputs
# source to this solution and code:
# http://stackoverflow.com/questions/14489013/simulate-python-keypresses-for-controlling-a-game
# http://www.gamespp.com/directx/directInputKeyboardScanCodes.html
#https://arma.fandom.com/zh/wiki/DIK_KeyCodes

#CODE FOR AZERTY KEYBOARD

import ctypes
import time

SendInput = ctypes.windll.user32.SendInput

D_ESC = 0x01

D_A = 0x10
D_Z = 0x11 
D_E = 0x12
D_R = 0x13
D_T = 0x14
D_Y = 0x15
D_U = 0x16


D_Q = 0x1E
D_S = 0x1F
D_D = 0x20
D_F = 0x21

D_LSHIFT =0x2A
D_W = 0x2C
D_X = 0x2D
D_C = 0x2E
D_V = 0X2F

D_CTRL = 0x36
D_SPACE = 0x39

def kn(hex_key):
    if hex_key == D_A:
        return "A"
    elif hex_key == D_Z:
        return "Z"
    elif hex_key == D_E:
        return "E"
    elif hex_key == D_R:
        return "R"
    elif hex_key == D_T:
        return "T"
    elif hex_key == D_Y:
        return "Y"
    elif hex_key == D_U:
        return "U"
    elif hex_key == D_Q:
        return "Q"
    elif hex_key == D_S:
        return "S"
    elif hex_key == D_D:
        return "D"
    elif hex_key == D_F:
        return "F"
    elif hex_key == D_LSHIFT:
        return "SHIFT"
    elif hex_key == D_W:
        return "W"
    elif hex_key == D_X:
        return "X"
    elif hex_key == D_C:
        return "C"
    elif hex_key == D_V:
        return "V"
    elif hex_key == D_CTRL:
        return "CTRL"
    elif hex_key == D_SPACE:
        return "SPACE"  


# C struct redefinitions
PUL = ctypes.POINTER(ctypes.c_ulong)
class KeyBdInput(ctypes.Structure):
    _fields_ = [("wVk", ctypes.c_ushort),
                ("wScan", ctypes.c_ushort),
                ("dwFlags", ctypes.c_ulong),
                ("time", ctypes.c_ulong),
                ("dwExtraInfo", PUL)]

class HardwareInput(ctypes.Structure):
    _fields_ = [("uMsg", ctypes.c_ulong),
                ("wParamL", ctypes.c_short),
                ("wParamH", ctypes.c_ushort)]

class MouseInput(ctypes.Structure):
    _fields_ = [("dx", ctypes.c_long),
                ("dy", ctypes.c_long),
                ("mouseData", ctypes.c_ulong),
                ("dwFlags", ctypes.c_ulong),
                ("time",ctypes.c_ulong),
                ("dwExtraInfo", PUL)]

class Input_I(ctypes.Union):
    _fields_ = [("ki", KeyBdInput),
                 ("mi", MouseInput),
                 ("hi", HardwareInput)]

class Input(ctypes.Structure):
    _fields_ = [("type", ctypes.c_ulong),
                ("ii", Input_I)]

# Actuals Functions

def PressKey(hexKeyCode):
    extra = ctypes.c_ulong(0)
    ii_ = Input_I()
    ii_.ki = KeyBdInput( 0, hexKeyCode, 0x0008, 0, ctypes.pointer(extra) )
    x = Input( ctypes.c_ulong(1), ii_ )
    ctypes.windll.user32.SendInput(1, ctypes.pointer(x), ctypes.sizeof(x))

def ReleaseKey(hexKeyCode):
    extra = ctypes.c_ulong(0)
    ii_ = Input_I()
    ii_.ki = KeyBdInput( 0, hexKeyCode, 0x0008 | 0x0002, 0, ctypes.pointer(extra) )
    x = Input( ctypes.c_ulong(1), ii_ )
    ctypes.windll.user32.SendInput(1, ctypes.pointer(x), ctypes.sizeof(x))

if __name__ == '__main__':
    PressKey(0x11)
    time.sleep(1)
    ReleaseKey(0x11)
    time.sleep(1)