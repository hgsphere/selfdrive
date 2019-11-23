import cv2 as cv
import networkx as nx
import json


refPt = None
img = None
G = nx.DiGraph()


def getPtName(x, y):
    return "{},{}".format(x, y)

def decodePtName(name):
    l = name.split(',')
    return int(l[0]), int(l[1])

def drawPt(img, x, y, color=(0, 255, 0)):
    cv.circle(img, (x, y), 2, color, 2)

def drawLine(img, pt0, pt1, color=(0, 255, 0)):
    cv.arrowedLine(img, pt0, pt1, color, 2)


def getClick(event, x, y, flags, params):
    global refPt, G, img

    if event == cv.EVENT_LBUTTONDOWN:
        refPt = (x, y)
        drawPt(img, x, y)

    elif event == cv.EVENT_LBUTTONUP:
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
    img = cv.imread(path)

    # read in graph file
    with open("graph.json", 'r') as jf:
        graphData = json.load(jf)
        G = nx.from_dict_of_dicts(graphData, create_using=G)

    # draw what we already have
    RED = (0, 0, 255)
    for node, edges in graphData.items():
        x, y = decodePtName(node)
        drawPt(img, x, y, color=RED)
        for nm, attr in edges.items():
            pt1 = decodePtName(nm)
            drawLine(img, (x, y), pt1, color=RED)

    cv.imwrite("graph_overlay.jpeg", img)

    image_top = img[0:img.shape[0]//2]
    image_bottom = img[img.shape[0]//2:img.shape[0] - 1]
    cv.namedWindow("image")
    # cv.namedWindow("image_top")
    # cv.namedWindow("image_bottom")
    # cv.setMouseCallback("image", getClick)
    cv.imshow("image", img)
    # cv.imshow("image_top", image_top)
    # cv.imshow("image_bottom", image_bottom)
    cv.waitKey(0)
    return

    while True:
        cv.imshow("image", img)
        key = cv.waitKey(1) & 0xFF

        if key == ord('q'):
            break

    cv.destroyAllWindows()
    with open("graph.json", 'w') as jf:
        data = nx.to_dict_of_dicts(G)
        # G = nx.from_dict_of_dicts(data)
        json.dump(data, jf, indent=2)


routeList = []

def clickRoute(event, x, y, flags, params):
    global routeList

    if event == cv.EVENT_LBUTTONUP:
        image = params[0]
        routeList.append(getPtName(x, y))

        # where you clicked
        drawPt(image, x, y)
        cv.imshow("image", image)


def defineRoute():
    global img
    path = "./Global.jpg"
    img = cv.imread(path)

    cv.namedWindow("image")
    cv.setMouseCallback("image", clickRoute, param=(img,))
    cv.imshow("image", img)

    while True:
        key = cv.waitKey(0) & 0xFF

        if key == ord('q'):
            break

    cv.destroyAllWindows()
    # dump the route to a file
    with open("route.json", 'w') as jf:
        json.dump(routeList, jf, indent=2)


def main():
    editGraph()
    # defineRoute()


if __name__ == '__main__':
    main()
