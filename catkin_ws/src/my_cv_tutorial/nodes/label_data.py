#!/usr/bin/env python
"""
Created on Sat Nov 17 10:36:07 2018

@author: jon
"""

import os
import cv2 as cv
import numpy as np
import shutil
import random
import easygui
from pathlib import Path
import sys
import argparse


def main():
    
    parser = argparse.ArgumentParser(description="Label images YOLO style")
    parser.add_argument("inputFolder", help="Path of folder containing images to label", type=str)
    parser.add_argument("outputFolder", help="Path of folder where labeled images will be sent and label files will be saved", type=str)

    args = parser.parse_args()

    data_folder = Path(args.inputFolder)
    final_folder = Path(args.outputFolder)

    if not data_folder.is_dir():
        print("{} is not a directory".format(data_folder))
        sys.exit(1)

    if not final_folder.is_dir():
        print("{} is not a directory".format(final_folder))
        sys.exit(1)

    # Recognize jpg or jpeg images
    images = list(data_folder.glob('*.jpg'))
    images.extend(list(data_folder.glob('*.jpeg')))
    random.shuffle(images)
    
    for file in images:
        print('Processing image {}'.format(file))

        img = cv.imread(str(file))

        boolDone = False

        fid = open(final_folder.joinpath(file.stem + '.txt'), 'w+')

        while not boolDone:
            cv.imshow('image', img)
            roi = cv.selectROI('image', img, False)

            if np.max(roi) > 0:
                print(roi)
                x = np.round((float(roi[0])+(float(roi[2]))/2)/img.shape[1], 2)
                y = np.round((float(roi[1])+(float(roi[3])/2))/img.shape[0], 2)
                width = np.round(float(roi[2])/img.shape[1], 2)
                height = np.round(float(roi[3])/img.shape[0], 2)
            
                annClass = easygui.integerbox('What class? class 0 to quit', 'Class box', lowerbound=0, upperbound=100)
            
                if annClass == 0:
                    boolDone = True
                else:
                    fid.write(str(annClass-1)+' '+str(x)+' '+str(y)+' '+str(width)+' '+str(height)+'\n')
                    if annClass == 1:
                        cv.rectangle(img, (roi[0], roi[1]), (roi[0]+roi[2], roi[1]+roi[3]), (0,255,0), 2)
                    elif annClass == 2:
                        cv.rectangle(img, (roi[0], roi[1]), (roi[0]+roi[2], roi[1]+roi[3]), (0,255,255), 2)
                    elif annClass == 3:
                        cv.rectangle(img, (roi[0], roi[1]), (roi[0]+roi[2], roi[1]+roi[3]), (0,0,255), 2)
                    elif annClass == 4:
                        cv.rectangle(img, (roi[0], roi[1]), (roi[0]+roi[2], roi[1]+roi[3]), (255,255,0), 2)
            else:
                boolDone = True

        fid.close()
        shutil.move(data_folder.joinpath(file), final_folder.joinpath(file.stem + '.jpg'))


if __name__ == "__main__":
    main()
