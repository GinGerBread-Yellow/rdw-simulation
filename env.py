import numpy as np

def read_env(path):

    with open(path, 'r') as f:
        line = f.readline()
        width = int(line)
        line = f.readline()
        height = int(line)

        ret = np.ones([width, height], dtype=bool)

        line = f.readline()
        while(line != "//\n"):
            nums = line.split(' ')
            assert(len(nums)==4)
            x1 = int(nums[0])
            y1 = int(nums[1])
            x2 = int(nums[2])
            y2 = int(nums[3])
            print(f"{x1},{x2},{y1},{y2}")
            ret[x1:x2, y1:y2] = 0 # np.zeros([x2-x1, y2-y1])
            line = f.readline()
        
        line = f.readline()
        init = line.split(' ')
        x1 = int(init[0])
        y1 = int(init[1])
        x2 = int(init[2])
        y2 = int(init[3])
        assert(ret[x1,y1])

    return width, height,x1,y1,x2,y2, ret
                

if __name__ == "__main__":
    print('===== list all env ======')
    for k in range(1,6):

        test_path = "./env/env_" + str(k) + ".txt"
        print(f"===== env {k} ======")
        width, height,x1,y1,x2,y2,ret = read_env(test_path)
        print(width, height,x1,y1,x2,y2)   
        m,n=ret.shape
        for i in range(m):
            for j in range(n):
                if (ret[i,j]):
                    print("*",end='')
                else :
                    print(" ",end='')
            print('')
        
                   



