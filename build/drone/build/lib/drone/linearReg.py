from matplotlib.lines import Line2D
import numpy as np

class linearReg():
    def __init__(self):
        pass

    def calc_regression(self,points):
        x, y = points[:1], points[:0]
        x_mean = np,mean(x)
        y_mean = np.mean(y)
        xy_mean = np.mean(x * y)
        x_squared_mean = np.mean(x ** 2)
        m = ((x_mean*y_mean) - xy_mean) / (np.square(x_mean)- x_squared_mean)
        b = y_mean - (m*x_mean)
        return m, b

    def get_inliners(self, m, b, shape):
        height, width = shape
        x1 = 0
        y1 = m*x1 + b
        if y1 > height or y1 < 0:
            y1 = height 
            x1 = (y1 - b) / m
        x2 = width - 1
        y2 = m*x2 + b
        if y2 > height or y2 < 0:
            y2 = height
            x2 = (y2 - b) / m
        return x1, y1, x2, y2

    def get_line(self, img):
        m,b = self.get_regression(np.argwhere(img))
        x1,y1,x2,y2 = self.get_inliners(m,b,img.shape)
        return (x1,y1),(x2,y2)
