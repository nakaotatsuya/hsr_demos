#!/usr/bin/env pytho
# -*- coding: utf-8 -*-

import line_notify
from PIL import Image

#client = line_notify.LineNotify(token="PuanmDsKMVCqqXdhpkh7MFth9qxsbhfRPTuBpZHGUvH")
#client = line_notify.LineNotify(token="9BGMexv6R4yuV5fEv71I9oXMFqD4SXV8G7eIsmX2ga1")
client = line_notify.LineNotify(token="PuanmDsKMVCqqXdhpkh7MFth9qxsbhfRPTuBpZHGUvH")

client.notify("test test")
#pil_img = Image.open("/home/nakaotatsuya/Pictures/dot.png")
#client.notify(imgs=pil_img)
