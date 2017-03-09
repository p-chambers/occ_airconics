from airconics.topology import Topology_GPTools


if __name__ == '__main__':
    # Initialise the display
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()


    topo_tools = Topology_GPTools(MaxAttachments=3)

    # expr_string = """fuselage1(0., 0., 0., 1.0, 0.182,
    # 0.293, 0.5022, mirror2(liftingsurface0(0.0, 0., 0., 0.38, 0., AirlinerTP),
    # liftingsurface0(0.5, 0.0, 0.0, 0.3214, 0.3, AirlinerFin)))"""

    expr_string = """fuselage2(0.3, 0., 0., 1., 0.293, 0.183, 0.8, 
            liftingsurface0(0.55, 0., 0.6, 1., 0.45, 1.0, AirlinerFin) mirror2(
                liftingsurface0(0.7, 0., 0.5, 1.0, 0.35, 0., AirlinerTP),
                liftingsurface1( 0.15, 0., 0.05, 1., 0.8, 0., AirlinerWing,
                    engine0(0.3, 0.8, 1/1.95, 0.98, 0.35, 0.))))"""


    # liftingsurface1(0.8172, 0., 0., 0.3214, 32.5, AirlinerFin, fuselage0(1., 0., 0., 1., 0.182, 0.293, 0.5))))"""

    config = topo_tools.from_string(expr_string)

    print(config)

    print(config._Parts)

    config.Display(display)

    # Test if attachment points are where they should be
    # for pt in config._testpoints:
    #     display.DisplayShape(pt)

    start_display()