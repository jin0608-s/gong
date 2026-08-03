from pathlib import Path

import cv2
import numpy as np
from matplotlib import pyplot as plt

def main():
    file_path = Path(__file__).parent
    img = cv2.imread(str(file_path / "data/robot.jpeg"))
    cv2.imshow("robot", img)        #검정 화면 뛰우기
    plt.axis("off")  # 좌표축 제거
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # BGR -> RGB
    img.resize((400, 400, 3))
    plt.imshow(imgRGB)
    plt.show()
    cv2.waitKey()   # 블럭 함수
    
    

if __name__=="__main__":
    main()