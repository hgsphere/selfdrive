from cv2 import (imread as cv_imread,
                circle as cv_circle,
                arrowedLine as cv_arrowedLine)
from networkx import (  DiGraph as nx_DiGraph,
                        from_dict_of_dicts as nx_from_dict_of_dicts,
                        to_dict_of_dicts as nx_to_dict_of_dicts)
from json import (  load as json_load,
                    dump as json_dump)

img = None
G = nx_DiGraph()
G_new = nx_DiGraph()


def getPtName(x, y):
    return "{},{}".format(x, y)

def decodePtName(name):
    l = name.split(',')
    return int(l[0]), int(l[1])

def drawPt(img, x, y, color=(0, 255, 0)):
    cv_circle(img, (x, y), 2, color, 2)

def drawLine(img, pt0, pt1, color=(0, 255, 0)):
    cv_arrowedLine(img, pt0, pt1, color, 2)

def load_graph():
    global img, G, G_new
    path = "./Global.jpg"
    img = cv_imread(path)

    with open("graph.json", 'r') as jf:
        graphData = json_load(jf)
        G = nx_from_dict_of_dicts(graphData, create_using=G)
        G_new = nx_from_dict_of_dicts(graphData, create_using=G_new)

    for node in G:
        x, y = decodePtName(node)
        for nm in G.successors(node):
            x1, y1 = decodePtName(nm)
            G_new.remove_edge(node, nm)
            new_node_name = getPtName((x1+x)//2,(y1+y)//2)
            G_new.add_node(new_node_name)
            G_new.add_edge(node, new_node_name)
            G_new.add_edge(new_node_name, nm)

    with open("graph_dense.json", 'w') as jf:
        data = nx_to_dict_of_dicts(G_new)
        # G = nx_from_dict_of_dicts(data)
        json_dump(data, jf, indent=2)


def main():
    load_graph()
    # defineRoute()


if __name__ == '__main__':
    main()


