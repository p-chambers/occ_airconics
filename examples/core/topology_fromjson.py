from airconics.topology import Topology_GPTools
import os
import airconics


if __name__ == '__main__':
    # Initialise the display
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()

    topo_tools = Topology_GPTools(MaxAttachments=4)

    # Could use topo_tools.from_file instead:
    fname = os.path.join(os.path.dirname(airconics.__file__),
                         'resources/configuration_app/presets/proteus.json')

    topo = topo_tools.from_file(fname)

    topo.Display(display)



    display.FitAll()
    start_display()
