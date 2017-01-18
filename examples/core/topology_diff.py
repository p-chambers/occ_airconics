# -*- coding: utf-8 -*-
"""
Created on Fri Jan  6 16:13:29 2017

@author: pchambers
"""

from airconics.topology import Topology_GPTools
import os
import airconics
from deap import gp


if __name__ == '__main__':

    topo_tools = Topology_GPTools(MaxAttachments=4)

    fname = os.path.join(os.path.dirname(airconics.__file__),
                         'resources/configuration_app/presets/thunderbolt.json')

    topo = topo_tools.from_file(fname, loader="json")

    fname = os.path.join(os.path.dirname(airconics.__file__),
                         'resources/configuration_app/presets/airliner.json')

    # can omit the loader='json' as this is default behavior
    topo2 = topo_tools.from_file(fname)

    pset = topo_tools._pset

    components_1 = [
        node.name for node in topo._deap_tree if isinstance(node, gp.Primitive)]

    components_2 = [
        node.name for node in topo2._deap_tree if isinstance(node, gp.Primitive)]

    import difflib

    sm = difflib.SequenceMatcher(None, components_1, components_2)
    print(sm.ratio())

    print(components_1)
    print(components_2)

    for match in sm.get_matching_blocks():
        print(match)

    for code in sm.get_opcodes():
        print(code)
