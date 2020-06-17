# -*- coding: utf-8 -*-
"""
Created on Sat Nov 17 13:14:47 2018

@author: jon
"""
import cv2 as cv
import pathlib
import argparse
from pathlib import Path
import sys


def checkAnnotation(img,anClass,x,y,w,h):

    coor_1=[int((x-w*0.5)*img.shape[1]),int((y-h*0.5)*img.shape[0])]
    coor_2=[int((x+w*0.5)*img.shape[1]),int((y+h*0.5)*img.shape[0])]

    if anClass==0:
        cv.rectangle(img, (coor_1[0], coor_1[1]), (coor_2[0], coor_2[1]),(0,255,0), 2)
    elif anClass==1:
        cv.rectangle(img, (coor_1[0], coor_1[1]), (coor_2[0], coor_2[1]),(0,255,255), 2)
    elif anClass==2:
        cv.rectangle(img, (coor_1[0], coor_1[1]), (coor_2[0], coor_2[1]),(0,0,255), 2)
    elif anClass==3:
        cv.rectangle(img, (coor_1[0], coor_1[1]), (coor_2[0], coor_2[1]),(255,255,0), 2)
    return img


def main():
    parser = argparse.ArgumentParser(description="Visually check label of images")
    parser.add_argument("inputImage", help="Path of image to check", type=str)
    parser.add_argument("inputLabel", help="Path of label to check", type=str)

    args = parser.parse_args()

    img_path = Path(args.inputImage)
    txt_file = Path(args.inputLabel)

    if not img_path.is_file():
        print("{} is not a file".format(img_path))
        sys.exit(1)

    if not txt_file.is_file():
        print("{} is not a file".format(txt_file))
        sys.exit(1)    
    
    img = cv.imread(str(img_path))

    with open(txt_file, "r") as f:
        lines = f.readlines()
            
    for line in lines:
        line = line.rstrip()
        tokens = line.split()
        img = checkAnnotation(img,float(tokens[0]),float(tokens[1]),float(tokens[2]),float(tokens[3]),float(tokens[4]))
    
    cv.imshow('image', img)
    cv.waitKey(0)
    cv.destroyAllWindows()
          
  
if __name__ == "__main__":
    main()
