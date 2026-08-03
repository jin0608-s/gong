from pathlib import Path

import cv2
import numpy as np

def main():
    file_path = Path(__file__).parent
    black_img = np.array((300, 300, 1), dtype=np.uint8)  # 검정 화면
    cv2.imshow("black", black_img)        #검정 화면 뛰우기
    cv2.waitKey()   # 블럭 함수
    

if __name__=="__main__":
    main()