from airconics import Topology


if __name__ == '__main__':
    # Initialise the display
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()


    config2 = Topology()

    expr_string = """fuselage1(0.4, 0., 0., 1.0, 0.182,
    0.293, 0.5022, mirror2(liftingsurface0(0., 0., 0., 0.38, 0., AirlinerTP),
    liftingsurface0(0.8172, 0., 0., 0.3214, 32.5, AirlinerFin)))"""

    config2.from_string(expr_string)

    print(config2)

    print(config2._Parts)

    config2.from_string(expr_string)

    config2.Display(display)

    for pt in config2._testpoints:
        display.DisplayShape(pt)

    start_display()