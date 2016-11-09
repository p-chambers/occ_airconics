# -*- coding: utf-8 -*-
# @Author: p-chambers
# @Date:   2016-11-09 14:53:09
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-11-09 14:54:50
from airconics import Topology

if __name__ == "__main__":
    config = Topology()

    config.randomize()

    print(config)

    print(config._Parts)

    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()

    config.Display(display)

    for pt in config._testpoints:
        display.DisplayShape(pt)

    start_display()
