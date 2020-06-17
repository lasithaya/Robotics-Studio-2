#  Train an object detector 

## Skills learned

- Label images from Duckietown logs to perform object detection
- Prepare data to train a neural network that performs object detection: YOLO
- Train YOLO on labelled images
- Run the object detector on images or videos

## Motivation

YOLO is a real-time object detection system with a small footprint, if using tiny YOLO, which is a version of the YOLO architecture that has fewer convolutional layers. The size of the trained weights of tiny YOLO is less than 50 Mb.

An object detection system can form the basis of a more complex pipeline, for instance, when doing SLAM. Below, we provide instructions on how to train tiny YOLO on 4 classes of objects:

1. Duckiebots
2. Duckies
3. Stop signs
4. Road signs

We hope to make it easier for anyone to train a Duckietown object detection system on more classes and with more data (to improve accuracy).

## Instructions

Clone the Github repository forked from YOLO's repository:

     $ git clone https://github.com/lasithaya/darknet.git

This repository contains the YOLO files, a dataset that we created and tools to train an object detection system.

### Creating a dataset
We include a small dataset of 420 images `data_4_classes.tar.gz` and provide tools to expand this dataset or create a new one with the classes of objects you want to detect.

First, if you are using the Duckietown logs, you will see that many of the images are blurry. Some of them are too blurry for anything useful to be learned. The script `detect_blurry_img.py` uses a Laplacien filter to determine if an image is blurry.

     $ python detect_blurry_img.py ![input_folder] ![blurry_folder] ![non_blurry_folder] ![threshold]

The arguments are:

* `input_folder`: directory of all images you want to classify as blurry. We grab all files ending with `.jpg` or `.jpeg`
* `blurry_folder`: directory where blurry images will be copied to
* `non_blurry_folder`: directory where non blurry images will be copied to
* `threshold`: default is 200, the higher the threshold, the stricter the classifier is on blurry images

After executing this script the directory `non_blurry_folder` will contain the non-blurry images that we can train on.

### Labeling the image
To train the object detector first we need to create a dataset with label

We will be using the Label Img tool to label the data. Clone this repository and install it and run.

```bash
git clone https://github.com/tzutalin/labelImg.git

``` 
More information about the installation can be found in the following link. Depending on your python version install correct dependencies. 

https://github.com/tzutalin/labelImg

create a textile with the following names and add them as an input to the labelImg class name file (`PRE-DEFINED CLASS FILE`). Add more classes to the bottom if you have more classes.
```bash
bot
duckie
stop_sign
road_sign
```

Go to the labelImg folder and run the labelImg as follows. `PRE-DEFINED CLASS FILE` is the file that you created above.IMAGE_PATH is the folder where you have all the images that you want to label.
```diff
- **Remember to select the YOLO from the menu when you start labeling. The default is Pascal/VOC.
-Otherwise, labels would not save in the correct format**.
```

```bash
python labelImg.py [IMAGE_PATH] [PRE-DEFINED CLASS FILE]
```

You can see a video below. That shows how to use the labeling software

[Click here for Video](https://www.youtube.com/watch?v=_FC6mr7k694)


![](https://github.com/lasithaya/Robotics-Studio-2/blob/master/labelimg.jpg)


### Preparing the dataset
To proceed, you need to have a directory with two sub-directories `frames` and `labels`, the first containing the images and the other containing the corresponding labels which you created by using the labelImg software above. From there we want to create directories that we will use to train YOLO. We provide a script named `create_datasets.py` that creates the required directories:

    $ python create_datasets.py ![raw_data_folder] ![data_folder] ![percentage_training]

The arguments are:

* `raw_data_folder`: directory containing two sub-directories `frames` and `labels`, the former containing the images (`.jpg` files) and the latter has the labels (`.txt` files)
* `data_folder`: directory where we want to create three directories `trainset`, `validset` and `testset`
* `percentage_training`: how much of the data to use for training, between 80 and 90. The validation set and the test set will contain the rest of the examples, equally divided.

After execution, you need to create two other files that will be required during training
```bash
     $ ls ![data_folder]/trainset/*.jpg > ![data_folder]/train.txt
     $ ls ![data_folder]/validset/*.jpg > ![data_folder]/valid.txt
```
We provide a sample bash script that performs all the steps to prepare the data. It is called `prepdata`.


### Prepare the YOLO config files
Now that the datasets have been created, we need to create the YOLO configuration files. These are:

* File containing the class names (`classname_file`)
* File specifying where the training/validation sets are as well as where to save the weights during training (`data_file`)
* File specifying the neural network architecture to use (`architecture_file`)

To start the training, we need to call: ( **We will run this in google colab not on your laptop.See the  training section below**)

   $ ./darknet detector train ![data_file] ![architecture_file] ![pretrained_weights]

The `data_file` is important as this specifies the data to use while training. you can save the data file in the ``darknet/cfg`` folder.  Our file, ``duckie-multi.data`` in the above repository has the following lines and it follows the YOLO convention:

```
classes= 4
train  = duckiestuff/train.txt
valid  = duckiestuff/valid.txt
names = data/duckie-multi.names
backup = duckie_backup
```
 The file reference `backup` defines the folder that weight files are saved. In this example, `duckie_backup` is that folder.
You can look at the repository to get an idea of what the files `train.txt` and `valid.txt` look like: lists of image paths. The file referenced by `names` is the `classname_file`, which in our case looks like the following since we are training on 4 classes:
```
bot
duckie
stop_sign
road_sign
```

These are the 4 classes of objects we are training on. An example label file is `duckiestuff/trainset/100_000151.txt` with a corresponding image file `duckiestuff/trainset/100_000151.jpg`. Note that they have to be in the same directory. The label for this image is specified as follows:

```
2 0.4 0.22 0.02 0.1
3 0.75 0.22 0.03 0.1
1 0.99 0.44 0.03 0.07
```

Each line of this file refers to an object. There are 5 elements in the YOLO label, each separated by a space. Note that all values are normalized by the image width and height to be between 0 and 1.0.

1. Class ID (in the same order as in the file containing the class names). Class 0 is bot in our case, class 1 is duckie and so on
2. x coordinate for the center of the object in the image
3. y coordinate for the center of the object in the image
4. object width
5. object height

Lastly, there is the `architecture_file`, which specifies the number of convolutional layers to use and has to match the number of classes we are trying to detect. We simply copy `cfg/yolov3-tiny.cfg` and make the following modifications:

1. Change lines 127 and 171 to filters=27 as we have 4 classes. The formula is filters=(classes + 5)\*3
2. Change lines 135 and 177 to classes=4 as we are training 4 classes

### Training

For the training steps, refer to the following [Google Colab](https://drive.google.com/open?id=17pV9CtC8MFi38z1gz8CqE0gD6DthDNMV), which you can copy and modify. This Colab starts by cloning the same repository as above. Change this to your repository

### Testing
Once trained you can download the weight files from your google drive and run the YOLO ROS node that we learned in last week to test the object detector's performances
