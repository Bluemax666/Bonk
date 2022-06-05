import win32api as wapi
import time

keyList = ["\b"]
for char in "ABCDEFGHIJKLMNOPQRSTUVWXYZ 0123456789,.'$/":
    keyList.append(char)

def getKeys():
    keys = []
    for key in keyList:
        if wapi.GetAsyncKeyState(ord(key)):
            keys.append(key)
    return keys


def waitForKey(k='P'):
    while True:  #stall until the key is pressed
        time.sleep(0.02)
        keys = getKeys()      
        if k in keys:
            break