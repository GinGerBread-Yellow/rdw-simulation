# perameter
from pickle import NONE
import numpy as np
from pandas import Int32Dtype
import math

from env import read_env
from randomPath import generatePath

__delta_t__ = 5
__delta_r__ = 1
__delta_c__ = 1
G_TMIN = 0.86
G_TMAX = 1.26
G_RMIN = 0.67
G_RMAX = 1.24
D = 5 # our choice
THETA_C = math.asin(D*0.045/2) # 0.1127
MAX_LENGTH = 1024


RE_CALL = 0
NO_CAND = 0

# compare 1
"Reset only"
# compare 2
"FORCE"

'''
K = 1000
N = len(path)
vertex: (int,int)
grid : bool[K][K] K
LOS: bool[K][K][K][K] 
DP: bool[K][K][K][K][N]

'''

def dist(ix,iy,jx,jy):
    return math.sqrt((jy-iy)**2 + (jx-ix)**2)

def unit_vec(vector):
    return vector/np.linalg.norm(vector)

def findAngle(ix,iy,jx,jy,kx,ky):
    v1 = [jx-ix, jy-iy]
    v2 = [kx-jx, ky-jy]
    v1_u = unit_vec(v1)
    v2_u = unit_vec(v2)
    angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    sin = np.cross(v1_u, v2_u)
    anticlockwise = 1
    # print(sin)
    if sin < 0:
        anticlockwise = -1
    elif sin == 0:
        anticlockwise = 0
    return angle, anticlockwise

def encode2DCoordinate(x, y):
    return x*MAX_LENGTH+y
def decode2DCoordinate(code):
    x,y = divmod(code, MAX_LENGTH)
    return x,y

class VirtualPath:
    def __init__(self, path):
        self.path = path
        self.n = len(path)

        self.lengthConstraint = []
        self.angleConstraint = []
        self.generateConstraint()

    
    def generateConstraint(self):
        for i in range(self.n-1):
            cx, cy = self.path[i]
            dx, dy = self.path[i+1]
            leng = dist(cx,cy,dx,dy)
            self.lengthConstraint.append((leng/G_TMAX, leng/G_TMIN))

            angle, antiClock = 0, 0
            if (i>0):
                bx, by = self.path[i-1]
                angle, antiClock = findAngle(bx,by,cx,cy,dx,dy)
            
            assert(angle >= 0)
            g_imin = angle/G_RMAX - THETA_C
            g_imax = angle/G_RMIN + THETA_C
            self.angleConstraint.append((g_imin, g_imax, antiClock))
    
    def cost(self, vx, vy, ix,iy, jx,jy, i):
        leng = dist(vx,vy,ix,iy)
        lengMin, lengMax = self.lengthConstraint[i]
        cost = 0
        if leng < lengMin or leng > lengMax:
            cost += __delta_t__
        angle, dir = findAngle(jx,jy, ix,iy, vx,vy)
        angleMin, angleMax, antiClock = self.angleConstraint[i]
        if dir*antiClock < 0:
            angle = -angle
        if angle < angleMin or angle > angleMax:
            if antiClock == 0:
                cost += __delta_c__
            else:
                cost += min(__delta_c__, __delta_r__)
        
        return cost
    def print(self):
        print("==== Path ===")
        print(f"length={self.n}")
        print(f"path={self.path}")
        print(f"lengconstraint={self.lengthConstraint}")
        print(f"angleconstraint={self.angleConstraint}")

    def compare(self, rdw_path):
        print("======== compare ========")
        assert(len(self.path) == len(rdw_path))
        for i in range(self.n-1):
            vx1, vy1 = self.path[i]
            vx2, vy2 = self.path[i+1]
            px1, py1 = rdw_path[i]
            px2, py2 = rdw_path[i+1]
            print(f"virtual length: {dist(vx1,vy1,vx2,vy2)};\t physical: {dist(px1,py1,px2,py2)}")
            if i>0:
                vx3, vy3 = self.path[i-1]
                px3, py3 = rdw_path[i-1]
                va, vc = findAngle(vx1,vy1,vx2,vy2,vx3,vy3)
                pa, pc = findAngle(px1,py1,px2,py2,px3,py3)
                print(f"virtual angle: {va,vc},\t physical angle {pa, pc}")


                

            

class Solution:

    def __init__(self, width, height, grid, LOS):
        self.grid = grid
        self.LOS = LOS
        self.width = width
        self.height = height
        self.rdwPath = []
    
    def find_path(self, ix,iy, jx,jy, virtual_path):
        self.n = virtual_path.n
        self.virtual_path = virtual_path
        self.dp = np.ones([self.width, self.height, self.width, self.height, self.n], dtype=Int32Dtype)
        self.dp = -self.dp # init with -1
        self.back_track = np.ones([self.width, self.height, self.width, self.height, self.n], dtype=Int32Dtype)
        self.back_track = -self.back_track # init with -1
        # print(f"{ix},{iy},{jx},{jy}")
        ans = self.top_down(ix,iy, jx,jy, 0)
        return ans
        
    
    def top_down(self, ix, iy, jx, jy, i):
        
        if (i == self.n-1):
            return 0
        if self.dp[ix,iy,jx,jy,i] != -1:
            # RE_CALL += 1
            return self.dp[ix,iy,jx,jy,i]
        
        # if i <= 1:
        #     print(f"==== top down{ix},{iy},{jx},{jy}, {i} === ")
        candidate_set = []
        for x in range(0,self.width):
            for y in range(0, self.height):
                if ix != x and iy != y and self.LOS[x,y,ix,iy]:
                    candidate_set.append((x,y))
        # print(f"cand size = {len(candidate_set)}")
        # if len(candidate_set)==0:
        #     # NO_CAND += 1
        #     self.dp[ix,iy,jx,jy,i] = -2
        #     self.back_track[ix,iy,jx,jy,i] = -1
        #     return -2

        minCost = -2
        back = -1
        for (vx,vy) in candidate_set:
            dpCost = self.top_down(vx,vy,ix,iy,i+1)
            if dpCost < 0:
                continue
            dpCost += self.virtual_path.cost(vx,vy,ix,iy,jx,jy,i)
            
            if minCost < 0 or dpCost < minCost:
                minCost = dpCost
                back = encode2DCoordinate(vx,vy)
                if minCost == 0:
                    break # early return
        # if i<=1:
        #     print(f"min = {minCost} back = {back}")
        self.dp[ix,iy,jx,jy,i] = minCost
        self.back_track[ix,iy,jx,jy,i] = back
        return minCost


    def BACK_TRACK(self,ix,iy,jx,jy,i):
        self.rdwPath.append((ix,iy))
        if i==self.n-1 or self.dp[ix,iy,jx,jy,i]==-2:
            return
        print(f"dp[{ix},{iy},{jx},{jy},{i}] = {self.dp[ix,iy,jx,jy,i]}")
        backCode = self.back_track[ix,iy,jx,jy,i]
        if backCode == -1:
            print(f"strange")
            return
        nx, ny = decode2DCoordinate(backCode)
        self.BACK_TRACK(nx,ny,ix,iy,i+1)
        return


def bresenham(x1,y1,x2,y2, grids=NONE):
    miny = min(y1,y2)
    maxy = max(y1,y2)

    if x1 == x2: 
        for y in range(miny, maxy+1):
            if grids is not NONE:
                if not grids[x1,y]:
                    return False
        return True
    elif x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
    
    m_new = 2 * (y2 - y1)
    slope_error_new = m_new - (x2 - x1)
    
    y=y1
    for x in range(x1,x2+1):
        # print("(",x ,",",y ,")")
        y = min(maxy, max(miny, y))
        if grids is not NONE:
            if not grids[x,y]:
                return False

        # Add slope to increment angle formed
        slope_error_new =slope_error_new + m_new
        # Slope error reached limit, time to
        # increment y and update slope error.
        if (slope_error_new >= 0):
            y=y+1
            slope_error_new = slope_error_new - 2 * (x2 - x1)
    return True
    
def generateLOS(width, height, LOS, grids):

    for x1 in range(width-1):
        for y1 in range(height-1):
            for x2 in range(x1+1, width):
                for y2 in range(y1+1, height):
                    # print(f"{x1},{y1}<->{x2},{y2}")
                    los = bresenham(x1,y1,x2,y2,grids)
                    LOS[x1,y1,x2,y2] = los
                    LOS[x2,y2,x1,y1] = los
                    

def simulate(width, height, ix, iy, jx, jy, grids):
    # width = 10
    # height = 10
    # grids = np.ones([width, height], dtype=bool)
    # print(f"grids={grids}")
    # # generate blockages
    # for i in range(2, 5):
    #     for j in range(4, 9):
    #         grids[i,j] = 0
    print("====== init ========")
    LOS = np.ones([width, height, width, height], dtype=bool)
    generateLOS(width, height, LOS, grids)

    paths = []
    for i in range(10):
        path = generatePath(10, 5)
        paths.append(path)

    for path in paths:
        virtual_path = VirtualPath(path) 
        virtual_path.print()
        print("======== dp ==========")
        sol = Solution(width, height, grid=grids, LOS=LOS)
        ans = sol.find_path(ix,iy,jx,jy,virtual_path)
        print(f"minimum cost = {ans}")
        print(f"recall = {RE_CALL}")
        print(f"no cand = {NO_CAND}")
            
        sol.BACK_TRACK(ix,iy,jx,jy, 0)
        print(f"rdw = {sol.rdwPath}")
        virtual_path.compare(sol.rdwPath)

if __name__ == "__main__":
    print(THETA_C)
    # for i in range(1,3):
    #     env_file = "./env/env_"+str(i)+".txt"
    #     w,h,ix,iy,jx,jy,grids=read_env(env_file)
    #     simulate(w,h,ix,iy,jx,jy,grids)
    


'''
5 (physical configuration + initial pos + initial orientation)
100 virtual path (can using rng)
'''