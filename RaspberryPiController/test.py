import NetworkCommunicator as NetworkCommunicator
import threading
import time

class super_class(object):
    def __init__(self):
        print("init called")

class sub_class(super_class):
    def foo(self):
        print("foo")

super_class_obj = super_class()
sub_class_obj = sub_class()
