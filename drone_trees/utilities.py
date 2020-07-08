#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Assorted utility functions.
"""

##############################################################################
# Imports
##############################################################################

import os
import pydot
import typing
import uuid

from py_trees import behaviour
from py_trees import blackboard
from py_trees import common
from py_trees import composites
from py_trees import console
from py_trees import decorators
from py_trees import utilities


##############################################################################
# Display Dot Trees
##############################################################################


def dot_tree(
        root: behaviour.Behaviour,
        visibility_level: common.VisibilityLevel=common.VisibilityLevel.DETAIL,
        collapse_decorators: bool=False,
        with_blackboard_variables: bool=False,
        with_qualified_names: bool=False):
    """
    Paint your tree on a pydot graph.

    .. seealso:: :py:func:`render_dot_tree`.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): the root of a tree, or subtree
        visibility_level (optional): collapse subtrees at or under this level
        collapse_decorators (optional): only show the decorator (not the child), defaults to False
        with_blackboard_variables (optional): add nodes for the blackboard variables
        with_qualified_names (optional): print the class information for each behaviour in each node, defaults to False

    Returns:
        pydot.Dot: graph

    Examples:

        .. code-block:: python

            # convert the pydot graph to a string object
            print("{}".format(py_trees.display.dot_graph(root).to_string()))
    """
    def get_node_attributes(node):
        blackbox_font_colours = {common.BlackBoxLevel.DETAIL: "white",
                                 common.BlackBoxLevel.COMPONENT: "lawngreen",
                                 common.BlackBoxLevel.BIG_PICTURE: "white"
                                 }
        if isinstance(node, composites.Chooser):
            if node.status==common.Status.SUCCESS:
                attributes = ('doubleoctagon', 'green', 'white')  # octagon
            elif node.status==common.Status.FAILURE:
                attributes = ('doubleoctagon', 'red', 'white')  # octagon
            elif node.status==common.Status.RUNNING:
                attributes = ('doubleoctagon', 'blue', 'white')  # octagon
            else:
                attributes = ('doubleoctagon', 'gray31', 'white')  # octagon
        elif isinstance(node, composites.Selector):
            if node.status==common.Status.SUCCESS:
                attributes = ('octagon', 'green', 'white')  # octagon
            elif node.status==common.Status.FAILURE:
                attributes = ('octagon', 'red', 'white')  # octagon
            elif node.status==common.Status.RUNNING:
                attributes = ('octagon', 'blue', 'white')  # octagon
            else:
                attributes = ('octagon', 'gray31', 'white')  # octagon
        elif isinstance(node, composites.Sequence):
            if node.status==common.Status.SUCCESS:
                attributes = ('record', 'green', 'white') 
            elif node.status==common.Status.FAILURE:
                attributes = ('record', 'red', 'white')
            elif node.status==common.Status.RUNNING:
                attributes = ('record', 'blue', 'white')
            else:
                attributes = ('record', 'gray31', 'white')
        elif isinstance(node, composites.Parallel):
            if node.status==common.Status.SUCCESS:
                attributes = ('parallelogram', 'green', 'white') 
            elif node.status==common.Status.FAILURE:
                attributes = ('parallelogram', 'red', 'white')
            elif node.status==common.Status.RUNNING:
                attributes = ('parallelogram', 'blue', 'white')
            else:
                attributes = ('parallelogram', 'gray31', 'white')
        elif isinstance(node, decorators.Decorator):
            attributes = ('Mdiamond', 'ghostwhite', 'black')
        else:
            if node.status==common.Status.SUCCESS:
                attributes = ('ellipse', 'green', 'white') 
            elif node.status==common.Status.FAILURE:
                attributes = ('ellipse', 'red', 'white')
            elif node.status==common.Status.RUNNING:
                attributes = ('ellipse', 'blue', 'white')
            else:
                attributes = ('ellipse', 'gray', 'black')
        try:
            if node.blackbox_level == common.BlackBoxLevel.DETAIL:
                if node.status==common.Status.SUCCESS:
                    attributes = ("ellipse", 'green', 'white')
                elif node.status==common.Status.FAILURE:
                    attributes = ("ellipse", 'red', 'white')
                elif node.status==common.Status.RUNNING:
                    attributes = ("ellipse", 'blue', 'white')
                else:
                    attributes = ("ellipse", 'gray', 'black')
        except AttributeError:
            # it's a blackboard client, not a behaviour, just pass
            pass
        return attributes

    def get_node_label(node_name, behaviour):
        """
        This extracts a more detailed string (when applicable) to append to
        that which will be used for the node name.
        """
        node_label = node_name.replace("*", "")
        if behaviour.verbose_info_string():
            node_label += "\n{}".format(behaviour.verbose_info_string())
        if with_qualified_names:
            node_label += "\n({})".format(utilities.get_fully_qualified_name(behaviour))
        return node_label

    fontsize = 9
    blackboard_colour = "blue"  # "dimgray"
    graph = pydot.Dot(graph_type='digraph', ordering="out")
    graph.set_name("pastafarianism")  # consider making this unique to the tree sometime, e.g. based on the root name
    # fonts: helvetica, times-bold, arial (times-roman is the default, but this helps some viewers, like kgraphviewer)
    graph.set_graph_defaults(fontname='times-roman')  # splines='curved' is buggy on 16.04, but would be nice to have
    graph.set_node_defaults(fontname='times-roman')
    graph.set_edge_defaults(fontname='times-roman')
    (node_shape, node_colour, node_font_colour) = get_node_attributes(root)
    node_root = pydot.Node(
        root.name,
        label=get_node_label(root.name, root),
        shape=node_shape,
        style="filled",
        fillcolor=node_colour,
        fontsize=fontsize,
        fontcolor=node_font_colour,
    )
    graph.add_node(node_root)
    behaviour_id_name_map = {root.id: root.name}

    def add_children_and_edges(root, root_node, root_dot_name, visibility_level, collapse_decorators, sub):
        if isinstance(root, decorators.Decorator) and collapse_decorators:
            return
        if visibility_level < root.blackbox_level:
            node_names = []
            for c in root.children:
                (node_shape, node_colour, node_font_colour) = get_node_attributes(c)
                node_name = c.name
                while node_name in behaviour_id_name_map.values():
                    node_name += "*"
                behaviour_id_name_map[c.id] = node_name
                # Node attributes can be found on page 5 of
                #    https://graphviz.gitlab.io/_pages/pdf/dot.1.pdf
                # Attributes that may be useful: tooltip, xlabel
                node = pydot.Node(
                    name=node_name,
                    label=get_node_label(node_name, c),
                    shape=node_shape,
                    style="filled",
                    fillcolor=node_colour,
                    fontsize=fontsize,
                    fontcolor=node_font_colour,
                )
                node_names.append(node_name)
                if root.blackbox_level == common.BlackBoxLevel.COMPONENT:
                    if sub is not None:
                        subgraph = pydot.Cluster(
                                graph_name=root_dot_name,
                                label=root_dot_name,
                                style='dotted')
                        sub.add_subgraph(subgraph)
                        subgraph.add_node(node)
                        edge = pydot.Edge(root_dot_name, node_name)
                        subgraph.add_edge(edge)
                    else:
                        subgraph = pydot.Cluster(
                                graph_name=root.name,
                                label=root.name,
                                style='dotted')
                        graph.add_subgraph(subgraph)
                        subgraph.add_node(node)
                        edge = pydot.Edge(root_dot_name, node_name)
                        subgraph.add_edge(edge)
                elif sub is not None:
                    sub.add_node(node)
                    edge = pydot.Edge(root_dot_name, node_name)
                    sub.add_edge(edge)
                    subgraph = sub
                else:
                    subgraph = None
                    graph.add_node(node)
                    edge = pydot.Edge(root_dot_name, node_name)
                    graph.add_edge(edge)
                if c.children != []:
                    add_children_and_edges(c, node, node_name, visibility_level, collapse_decorators, subgraph)

    add_children_and_edges(root, node_root, root.name, visibility_level, collapse_decorators, None)

    def create_blackboard_client_node(blackboard_client: blackboard.Blackboard):
        return pydot.Node(
            name=blackboard_client.name,
            label=blackboard_client.name,
            shape="ellipse",
            style="filled",
            color=blackboard_colour,
            fillcolor="gray",
            fontsize=fontsize - 2,
            fontcolor=blackboard_colour,
        )

    def add_blackboard_nodes(blackboard_id_name_map: typing.Dict[uuid.UUID, str]):
        data = blackboard.Blackboard.storage
        metadata = blackboard.Blackboard.metadata
        clients = blackboard.Blackboard.clients
        # add client (that are not behaviour) nodes
        subgraph = pydot.Subgraph(
            graph_name="Blackboard",
            id="Blackboard",
            label="Blackboard",
            rank="sink",
        )
        blackboard_keys = pydot.Node(
            "BlackboardKeys",
            label="Keys",
            shape='box'
        )
        root_dummy_edge = pydot.Edge(
            root.name,
            "BlackboardKeys",
            color="magenta",
            style="invis",
            constraint=True,
        )
        subgraph.add_node(blackboard_keys)
        graph.add_edge(root_dummy_edge)

        for unique_identifier, client in clients.items():
            if unique_identifier not in blackboard_id_name_map:
                subgraph.add_node(
                    create_blackboard_client_node(client)
                )
        # add key nodes
        for key in blackboard.Blackboard.keys():
            try:
                value = utilities.truncate(str(data[key]), 20)
                label = key + ": " + "{}".format(value)
            except KeyError:
                label = key + ": " + "-"
            blackboard_node = pydot.Node(
                key,
                label=label,
                shape='box',
                style="filled",
                color=blackboard_colour,
                fillcolor='white',
                fontsize=fontsize - 1,
                fontcolor=blackboard_colour,
                width=0, height=0, fixedsize=False,  # only big enough to fit text
            )
            subgraph.add_node(blackboard_node)
            for unique_identifier in metadata[key].read:
                try:
                    edge = pydot.Edge(
                        blackboard_node,
                        blackboard_id_name_map[unique_identifier],
                        color="green",
                        constraint=False,
                        weight=0,
                    )
                except KeyError:
                    edge = pydot.Edge(
                        blackboard_node,
                        clients[unique_identifier].__getattribute__("name"),
                        color="green",
                        constraint=False,
                        weight=0,
                    )
                graph.add_edge(edge)
            for unique_identifier in metadata[key].write:
                try:
                    edge = pydot.Edge(
                        blackboard_id_name_map[unique_identifier],
                        blackboard_node,
                        color=blackboard_colour,
                        constraint=False,
                        weight=0,
                    )
                except KeyError:
                    edge = pydot.Edge(
                        clients[unique_identifier].__getattribute__("name"),
                        blackboard_node,
                        color=blackboard_colour,
                        constraint=False,
                        weight=0,
                    )
                graph.add_edge(edge)
        graph.add_subgraph(subgraph)

    if with_blackboard_variables:
        blackboard_id_name_map = {}
        for b in root.iterate():
            for bb in b.blackboards:
                blackboard_id_name_map[bb.id()] = behaviour_id_name_map[b.id]
        add_blackboard_nodes(blackboard_id_name_map)

    return graph


def render_dot_tree(root: behaviour.Behaviour,
                    visibility_level: common.VisibilityLevel=common.VisibilityLevel.DETAIL,
                    collapse_decorators: bool=False,
                    name: str=None,
                    target_directory: str=os.getcwd(),
                    with_blackboard_variables: bool=False,
                    with_qualified_names: bool=False,
                    imgtype = None
                    ):
    """
    Render the dot tree to .dot, .svg, .png. files in the current
    working directory. These will be named with the root behaviour name.

    Args:
        root: the root of a tree, or subtree
        visibility_level: collapse subtrees at or under this level
        collapse_decorators: only show the decorator (not the child)
        name: name to use for the created files (defaults to the root behaviour name)
        target_directory: default is to use the current working directory, set this to redirect elsewhere
        with_blackboard_variables: add nodes for the blackboard variables
        with_qualified_names: print the class names of each behaviour in the dot node
        imgtype: choose type of image saved: .png, .svg and .dot; default will save all three

    Example:

        Render a simple tree to dot/svg/png file:

        .. graphviz:: dot/sequence.dot

        .. code-block:: python

            root = py_trees.composites.Sequence("Sequence")
            for job in ["Action 1", "Action 2", "Action 3"]:
                success_after_two = py_trees.behaviours.Count(name=job,
                                                              fail_until=0,
                                                              running_until=1,
                                                              success_until=10)
                root.add_child(success_after_two)
            py_trees.display.render_dot_tree(root)

    .. tip::

        A good practice is to provide a command line argument for optional rendering of a program so users
        can quickly visualise what tree the program will execute.
    """
    graph = dot_tree(
        root, visibility_level, collapse_decorators,
        with_blackboard_variables=with_blackboard_variables,
        with_qualified_names=with_qualified_names)
    filename_wo_extension_to_convert = root.name if name is None else name
    filename_wo_extension = utilities.get_valid_filename(filename_wo_extension_to_convert)
    filenames = {}
    if imgtype == 'svg':
        for extension, writer in {"svg": graph.write_svg}.items():
            filename = filename_wo_extension + '.' + extension
            pathname = os.path.join(target_directory, filename)
            print("Writing {}".format(pathname))
            writer(pathname)
            filenames[extension] = pathname
    elif imgtype == 'dot':
        for extension, writer in {"dot": graph.write}.items():
            filename = filename_wo_extension + '.' + extension
            pathname = os.path.join(target_directory, filename)
            print("Writing {}".format(pathname))
            writer(pathname)
            filenames[extension] = pathname
    elif imgtype == 'png':
        for extension, writer in {"png": graph.write_png}.items():
            filename = filename_wo_extension + '.' + extension
            pathname = os.path.join(target_directory, filename)
            print("Writing {}".format(pathname))
            writer(pathname)
            filenames[extension] = pathname
    else:
        for extension, writer in {"dot": graph.write, "png": graph.write_png, "svg": graph.write_svg}.items():
            filename = filename_wo_extension + '.' + extension
            pathname = os.path.join(target_directory, filename)
            print("Writing {}".format(pathname))
            writer(pathname)
            filenames[extension] = pathname

    return filenames
