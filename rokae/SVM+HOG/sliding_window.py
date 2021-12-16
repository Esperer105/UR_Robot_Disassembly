# 满足 windowSize>stepSize
# windowSize[0]=x,windowSize[1]=y
# x为横轴,y为纵轴
def sliding_window(image, stepSize, windowSize):
    for y in range(0, image.shape[0], stepSize):
        if y == range(0, image.shape[0], stepSize)[-1]:
            for x in range(0, image.shape[1], stepSize):
                if x == range(0, image.shape[1], stepSize)[-1]:
                    yield (x, y, image[y:image.shape[0], x:image.shape[1]])
                else:
                    yield (x, y, image[y: image.shape[0], x:x + windowSize[0]])
        else:
            for x in range(0, image.shape[1], stepSize):
                if x == range(0, image.shape[1], stepSize)[-1]:
                    yield (x, y, image[y:y + windowSize[1], x:image.shape[1]])
                else:
                    yield(x,y,image[y:y+windowSize[1],x:x+windowSize[0]])

