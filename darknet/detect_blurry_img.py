# -*- coding: utf-8 -*-
"""
Created on Sat Nov 17 09:55:11 2018

@author: jon
"""
import cv2 as cv
import os
import argparse
from pathlib import Path
import sys


# detect if image is blurry
def main():
    
    parser = argparse.ArgumentParser(description="Eliminate blurry pictures")
    parser.add_argument("inputFolder", help="Path of folder containing images to classify", type=str)
    parser.add_argument("blurryFolder", help="Path of folder where blurry images will be sent", type=str)
    parser.add_argument("notBlurryFolder", help="Path of folder where non blurry images will be sent", type=str)
    parser.add_argument("--threshold", help="Threshold for blurry detection, default is 200", type=int, default=200)

    args = parser.parse_args()

    data_folder = Path(args.inputFolder)
    blurry_folder = Path(args.blurryFolder)
    good_folder = Path(args.notBlurryFolder)
    threshold = args.threshold

    if not data_folder.is_dir():
        print("{} is not a directory".format(data_folder))
        sys.exit(1)

    if not blurry_folder.is_dir():
        print("{} is not a directory".format(blurry_folder))
        sys.exit(1)

    if not good_folder.is_dir():
        print("{} is not a directory".format(good_folder))
        sys.exit(1)

    # Recognize jpg or jpeg images
    images = list(data_folder.glob('*.jpg'))
    images.extend(list(data_folder.glob('*.jpeg')))
    
    # Go through all images in data folder
    for imageFile in images:
        print('Processing image {}'.format(imageFile))

        image = cv.imread(str(imageFile))
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        fm = cv.Laplacian(gray, cv.CV_64F).var()

        # blurry
        if fm < threshold:
            cv.imwrite(str(blurry_folder.joinpath(imageFile.name)), image)
        # not blurry
        else:
            cv.imwrite(str(good_folder.joinpath(imageFile.name)), image)


if __name__ == "__main__":
    main()
