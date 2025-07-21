import cv2
import numpy as np
import matplotlib.pyplot as plt
import yaml

def process_map(map: np.array, times: int = 3):
    NN_binary = np.zeros_like(map)
    for x in range(map.shape[0]):
        for y in range(map.shape[1]):
            try:
                if(np.sum(map[x-1:x+2,y-1:y+2])>=5):
                    NN_binary[x,y]=1
            except IndexError:
                None #Can assume border to be black :)

    print(f"Sum of NN_binary after iteration {times}: {np.sum(NN_binary)}")  # Debug print

    if(times>1):
        NN_binary = process_map(map=NN_binary, times=times-1)
    return NN_binary

def flood(map: np.array, pose=None):
    if not pose:
        print(f"No pose given")
        return 0

    change = True
    flooded_map = np.zeros_like(map)
    flooded_map[pose] = 1
    while change:
        change = False
        for x in range(map.shape[0]):
            for y in range(map.shape[1]):
                if(map[x,y]==0): continue
                
                if(np.sum(flooded_map[x-1:x+2,y-1:y+2])>=1):
                    if(flooded_map[x,y]!=1): change = True
                    flooded_map[x,y]=1
    return flooded_map

def plotmap(map: np.array, pose=None):
    plt.imshow(map, cmap='gray')
    if(pose):plt.scatter(pose[0],pose[1])
    plt.colorbar()
    plt.show()

def main(map_path, pose=None, verbose=0):
    map = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    if(verbose>=1):plotmap(map)
    if(verbose>=1):print(f"Different values in beginning: {np.unique(map)}")

    map[map < np.max(map)] = 0
    if(verbose>=1):plotmap(map)
    if(verbose>=1):print(f"Different values after beginning: {np.unique(map)}")
    binary_map = (map == np.max(map))

    processed_binary = process_map(map=binary_map, times=3)

    if pose is not None and not(processed_binary[pose]):
        print(f"Please select a pixel which is white and on main track {pose}")
        plotmap(processed_binary, pose)
        return 1
    
    flooded = flood(processed_binary, pose)

    plotmap(flooded)

    flooded_map = (flooded * 255).astype(np.uint8)
    
    plotmap(flooded_map)
    
    cv2.imwrite(f"{map_path[:-4]}_new.png", flooded_map)

    return flooded_map

if __name__ == "__main__":
    main()