import ctypes
import inspect
from threading import Thread
import time
from typing import Optional


def _async_raise(tid: Optional[int], exctype):
    """raises the exception, performs cleanup if needed"""
    new_tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(new_tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(new_tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def stop_thread(thread: Thread):
    for i in range(5):
        _async_raise(thread.ident, SystemExit)


def test():
    while True:
        print('-------')
        time.sleep(1)


if __name__ == "__main__":
    thread = Thread(target=test)
    thread.start()
    time.sleep(5)
    print("main thread sleep finish")
    stop_thread(thread)
