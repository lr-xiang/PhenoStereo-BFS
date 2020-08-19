# Stereo Sensor Module #

This is a program to control multiple FLIR BLACK FLY S cameras simutanously with a Nvidia Jetson TX2, in a stereo camera module.   
It needs to communicate with an upper level computer by socket (wired or wireless), to initialize, sync, start acquisition, and return images to review on the upper level computer.  
All images saved as .jpeg file, BayerRG8 encoded

## Dependency:
* boost (modules: system, filesystem, thread. Got from apt-get)
* libjsoncpp ( >> sudo apt-get install libjsoncpp-dev )
* opencv ( any version, I used the one got from apt-get )
* gtest ([gtest tutorial](https://www.eriksmistad.no/getting-started-with-google-test-on-ubuntu/))
* spinnaker SDK ARM64( [Official website](https://www.ptgrey.com/support/downloads) [Using usb camera on ARM](https://www.ptgrey.com/KB/11145))
* libb64 for base64 image encoding ( >> sudo apt-get install libb64-dev )

## Socket message defination: ##
This applies to messages from both sides, all strings in lower case.  
One message is compose of a header and a body.
* Header: 8 bytes integer number string, the size (in bytes) of the body.
* Body: String in json form, length defined in header. The body contains possible fields:
  * "command" : The task as string. 
  * "data" : The parameters / returned information.

### Details:

| command           | data                      | Note                      | Direction (from)  |
| ------------------| --------------------------| ------------------------- | ----------------- |
| connect           | N/A                       | Connect all the cameras   | Host              |
| sync              | yyyy.mm.dd-HH.MM.SS.fff   | UTC time shared           | Host              |
| set_paras         | (Json) paras              | If data empty, use default| Host              |
| set_exposure      | (str as float), in us     |                           | Host              |
| start_capturing   | N/A                       |                           | Host              |
| single_trigger    | N/A                       |                           | Host              |
| cont_trigger      | (str as int),frames to take| If -1 keep taking        | Host              |
| stop_trigger      | N/A                       |                           | Host              |
| disconnect        | N/A                       |                           | Host              |
| ****              | (str) "OK,"+info / Error code| Response of each commands | server         |
| images            | (Json) List of images (see below)| Base64 encoded images | server         |

For the message "images", the fields are: 
* "n_imgs": number of images in this message, should be equal to # of cameras
* "imgs": List of json objects of each image. The image json objects include fields:
  * "width": No need to explain
  * "height":
  * "type": `int(img.type())` If 0, 8UC1. Refer to [this table](http://ninghang.blogspot.com/2012/11/list-of-mat-type-in-opencv.html)
  * "size": `int(img.total() * img.elemSize())` Total bytes of data
  * "data": Base64 encoded image.data part

### Some notes about using boost::asio:

1. Good website to learn: [link](https://theboostcpplibraries.com/boost.asio)
2. `io_service` handles the async operation. The "async**" function returns imediatly. The "handles" will not be exec-ed without `io_service.run()`. `io_service.run()` is blocking. Use a new thead to make it non-blocking. Examples shown blow.
3. `io_service` can handle multiple tasks concurrently (e.g. one handle run on thread1, one on thread2). Because this program needs async read and write concurrently, I used two threads. Code:
```c++
        // As class members. Two threads ensure two handles can be executed concurretly
        boost::thread worker_thread1_;
        boost::thread worker_thread2_;
```
```c++
        // In the main entry, start 
        do_accept(); // Has reading & running, two async processes
        worker_thread1_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
        worker_thread2_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
```

4. For the shared queues, use "shared_ptr" 
5. For writting, this program needs to monitor the shared "send" msg queue. I used a writter thread to handle this. ..
.. ** Update: ** No need to use a seperate thread for the writting. Since `do_write()` is called in a handle, even it is in blocking style.


## Black fly s with Spinnaker

* Bug: when starting acquisition, the program failed to set "PixelFormat" node.  
Reason: Disconnect doesn't stop acquisition.   
Solution: So call DeInit() before calling Init() to a camera.   
** Update ** This didn't fully fix the problem. It was finally solved by calling `pCam->AcquisitionStop()` before starting acquisition. Calling `EndAcquisition()` will raise error.

## GPIO related:

* In the external sync mode, the Jetson will use GPIO to control the cameras by sending pulses signals to Camera GPIO. 
* The Jetson GPIO (dev kit board) can output maximum 1mA / 20uA, the camera's Opt-input (Pin 2, Line 0) needs minimum 3.5mA. So a transistor was used to amplify the current.
* With the GPIO code from [1], the Jetson can control one pin to output pulse at ~37kHz maximum
* On the camera side, used Pin 2, Line 0 as trigger input. 

### Connection:
* When using Jetson dev kit board, I used J21 Pin1(3.3v), Pin13(GPIO-GEN2, 397) and Pin14(GND)
* On Orbitty, I will use Pin1(3.3v), Pin7(GPIO-0, 388) and Pin19(GND)

### Useful links:
* [J21 Header on developer kit board pinout](https://www.jetsonhacks.com/nvidia-jetson-tx2-j21-header-pinout/)
* [Detailed specification for J21 pinout](https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/390/JetsonTX1_5F00_TX2_5F00_Developer_5F00_Kit_5F00_Carrier_5F00_Board_5F00_Specification1.pdf) ( P24, S3.4 )
* [Orbitty carrier pinout](http://connecttech.com/pdf/CTIM-ASG003_Manual.pdf)
* [GPIO access permission for non-root user (ubuntu) on TX1](https://jkjung-avt.github.io/gpio-non-root/).
* [A tutorial of how to use GPIO signals](https://developer.ridgerun.com/wiki/index.php/How_to_use_GPIO_signals)[1]
* [Black Fly S installation Guide](https://www.ptgrey.com/support/downloads/10610)
* [libb64 from cv Mat example](https://stackoverflow.com/questions/28003981/opencv-cvmat-to-stdifstream-for-base64-encoding/28014307)

## Performance:
Speed:
* Software trigger 2 cams, Save to Jetson internal drive, With seperate thread saving: 45 fps max (Oct 10)
* Software trigger 2 cams, Save to Jetson internal drive, Without seperate thread saving: 12.25 fps (Oct 9)
* Hardware trigger 1 cam, Save to Jetson internal drive, with seperate thread saving; 22 fps
Size:
* Hardware trigger 2 cams, Save jpeg to SD card, seperate thread saving: 17.3 fps

3 channel images, using cv::imwrite:
* jpeg, quality_75 :105.5k
* jpeg, quality_80 :122.9k
* jpeg, quality_85 :148.5k
* jpeg, quality_90 :197.7k
* jpeg, quality_95 :307.8k
* jpeg, quality_100 :796.7k

Continuous acquisition time:
* HW trigger, 2 cams, jpeg to SD card, 17.3 fps: 120s
* HW trigger, 2 cams, jpeg to SSD, 17 fps: 660s
* HW trigger, 2 cams, pgm to SSD, disk full save empty files
* HW trigger, 2 cams, pgm to SD, 21 fps: 263s
## TODO:
2. Test GPIO mode (Done)
3. Slave mode (Done, without testing)
4. Speed up frame rate by using buffer for saving.   ** Update ** 12.25-> 45 fps (Oct 10)
5. Save to video using GStream? (Not that important)
6. Disable the Auto Gain (Done)
7. Migrate to Orbitty

## BUG and fix:
* [BUG] when using boost 1.66, `/usr/local/include/boost/asio/detail/consuming_buffers.hpp:105:50: error: parse error in template argument list
     while (next != end && max_size > 0 && result.count < result.max_buffers)`  
  Solved by edit the header to `while (next != end && max_size > 0 && (result.count) < result.max_buffers)`


