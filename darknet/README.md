# Training YOLO on Duckietown images #
We forked [Darknet](http://pjreddie.com/darknet) to train YOLO on images from Duckietown. The folder `duckiestuff` contains training images and labels in `trainset`. There also is a validation set in `validset` and a test set `testset`.

All the required configuration files have been added to `cfg`. We prefer to train with tiny YOLO as its accuracy is good enough and it has better chances of running on the duckiebot as it is smaller.

The first important configuration file is `cfg/duckie-multi.data`. Its contents are:

```classes= 4
train  = duckiestuff/train.txt
valid  = duckiestuff/valid.txt
names = data/duckie-multi.names
backup = duckie_backup
```

This means that the file `duckiestuff/train.txt` lists all images that will be used as the training set and the file `duckiestuff/valid.txt` contains those that are part of the validation set. It is unclear how those in the validation set are used.

Also, the file `data/duckie-multi.names` contains the class names. The contents of `data/duckie-multi.names` are:
```
bot
duckie
stop_sign
road_sign
```

These are the 4 classes of objects we are training on. An example label file is `duckiestuff/trainset/100_000151.txt` with a corresponding image file `duckiestuff/trainset/100_000151.jpg`. Note that they have to be in the same directory. The label for this image is specified as follows:

```2 0.16 0.23 0.07 0.19
2 0.4 0.22 0.02 0.1
3 0.75 0.22 0.03 0.1
1 0.99 0.44 0.03 0.07
```

Each line of this file refers to an object. There are 5 elements in the YOLO label, each separated by a space:
1. Class ID (in the same order as in the file containing the class names). Class 0 is `bot` in our case, class 1 is `duckie` and so on
2. x coordinate for the center of the object in the image
3. y coordinate for the center of the object in the image
4. object width (normalized by the width of the image)
5. object height (normalized by the height of the image)

Lastly, the file `cfg/yolov3-tiny-duckie-multi.cfg` specifies the architecture. We simply copied `cfg/yolov3-tiny.cfg` and made the following modifications
1. Change lines 127 and 171 to `filters=27` as we have 4 classes. The formula is `filters=(classes + 5)*3`
2. Change lines 135 and 177 to `classes=4` as we are training 4 classes

Having these files, we simply run `./darknet detector train cfg/duckie-multi.data cfg/yolov3-tiny-duckie-multi.cfg darknet53.conv.74` as we want to start training from pretrained weights.

To run inference on a duckietown image:

`./darknet detector test cfg/duckie-multi.data cfg/yolov3-tiny-duckie-multi.cfg cfg/yolov3-tiny-duckie-multi.weights duckiestuff/testset/102_000059.jpg`

To run inference on a video (from Duckietown logs):

`/darknet detector demo cfg/duckie-multi.data cfg/yolov3-tiny-duckie-multi.cfg cfg/yolov3-tiny-duckie-multi.weights 20171020150812_bluck.video.mp4`
