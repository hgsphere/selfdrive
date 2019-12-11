from cv2 import (EVENT_LBUTTONUP as cv_EVENT_LBUTTONUP,
                EVENT_LBUTTONDOWN as cv_EVENT_LBUTTONDOWN,
                imread as cv_imread,
                imshow as cv_imshow,
                imwrite as cv_imwrite,
                namedWindow as cv_namedWindow,
                setMouseCallback as cv_setMouseCallback,
                waitKey as cv_waitKey,
                destroyAllWindows as cv_destroyAllWindows,
                circle as cv_circle,
                arrowedLine as cv_arrowedLine)
from networkx import (  DiGraph as nx_DiGraph,
                        from_dict_of_dicts as nx_from_dict_of_dicts,
                        to_dict_of_dicts as nx_to_dict_of_dicts)
from json import (  load as json_load,
                    dump as json_dump)


refPt = None
img = None
G = nx_DiGraph()


def getPtName(x, y):
    return "{},{}".format(x, y)

def decodePtName(name):
    l = name.split(',')
    return int(l[0]), int(l[1])

def drawPt(img, x, y, color=(0, 255, 0)):
    cv_circle(img, (x, y), 2, color, 2)

def drawLine(img, pt0, pt1, color=(0, 255, 0)):
    cv_arrowedLine(img, pt0, pt1, color, 2)


def getClick(event, x, y, flags, params):
    global refPt, G, img

    if event == cv_EVENT_LBUTTONDOWN:
        refPt = (x, y)
        drawPt(img, x, y)

    elif event == cv_EVENT_LBUTTONUP:
        # names of nodes encoded
        n0 = getPtName(*refPt)
        n1 = getPtName(x, y)

        # add to graph
        G.add_node(n0)
        G.add_node(n1)
        G.add_edge(n0, n1)

        # visualize
        drawPt(img, x, y)
        drawLine(img, refPt, (x, y))


def editGraph():
    global img, G
    path = "./Global.jpg"
    img = cv_imread(path)

    # read in graph file
    with open("graph.json", 'r') as jf:
        graphData = json_load(jf)
        G = nx_from_dict_of_dicts(graphData, create_using=G)

    # draw what we already have
    RED = (0, 0, 255)
    for node, edges in graphData.items():
        x, y = decodePtName(node)
        drawPt(img, x, y, color=RED)
        for nm, attr in edges.items():
            pt1 = decodePtName(nm)
            drawLine(img, (x, y), pt1, color=RED)

    cv_imwrite("graph_overlay.jpeg", img)

    image_top = img[0:img.shape[0]//2]
    image_bottom = img[img.shape[0]//2:img.shape[0] - 1]
    cv_namedWindow("image")
    # cv_namedWindow("image_top")
    # cv_namedWindow("image_bottom")
    # cv_setMouseCallback("image", getClick)
    cv_imshow("image", img)
    # cv_imshow("image_top", image_top)
    # cv_imshow("image_bottom", image_bottom)
    cv_waitKey(0)
    return

    while True:
        cv_imshow("image", img)
        key = cv_waitKey(1) & 0xFF

        if key == ord('q'):
            break

    cv_destroyAllWindows()
    with open("graph.json", 'w') as jf:
        data = nx_to_dict_of_dicts(G)
        # G = nx_from_dict_of_dicts(data)
        json_dump(data, jf, indent=2)


routeList = []

def clickRoute(event, x, y, flags, params):
    global routeList

    if event == cv_EVENT_LBUTTONUP:
        image = params[0]
        routeList.append(getPtName(x, y))

        # where you clicked
        drawPt(image, x, y)
        cv_imshow("image", image)


def defineRoute():
    global img
    path = "./Global.jpg"
    img = cv_imread(path)

    cv_namedWindow("image")
    cv_setMouseCallback("image", clickRoute, param=(img,))
    cv_imshow("image", img)

    while True:
        key = cv_waitKey(0) & 0xFF

        if key == ord('q'):
            break

    cv_destroyAllWindows()
    # dump the route to a file
    with open("route.json", 'w') as jf:
        json_dump(routeList, jf, indent=2)


def main():
    editGraph()
    # defineRoute()


if __name__ == '__main__':
    main()
