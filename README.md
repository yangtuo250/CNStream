# Cambricon CNStream #
CNStream is a streaming framework with plug-ins. It is used to connect other modules, includes basic functionalities, libraries,
and essential elements.

CNStream provides the following built-in modules:

- DataSource: Support RTSP, video file,  images and elementary stream in memory （H.264, H.265, and JPEG decoding）.
- Inferencer: MLU-based inference accelerator for detection and classification.
- Inferencer2: Based on infer server to run inference, preprocessing and postprocessing.
- Osd (On-screen display): Module for highlighting objects and text overlay.
- Encode: Encode videos or images.
- Displayer: Display the video on screen.
- Tracker: Multi-object tracking.
- RtspSink：Push RTSP stream to internet.

### Getting started ###

  To start using CNStream, please refer to the chapter of ***quick start*** in the document of [Cambricon-CNStream-User-Guild-CN.pdf](./docs/release_document/latest/Cambricon-CNStream-User-Guide-CN-vlatest.pdf) .
## Samples ##

|                        Classification                        |               Object Detection                |
| :----------------------------------------------------------: | :-------------------------------------------: |
| <img src="./data/gifs/image_classification.gif" alt="Classification" style="height=350px" /> | <img src="./data/gifs/object_detection_yolov3.gif" alt="Object Detection" style="height=350px" /> |

|               Object Tracking               |               License plate recognization               |
| :-----------------------------------------: | :-----------------------------------------------------: |
| <img src="./data/gifs/object_tracking.gif" alt="Object Tracking" style="height=350px" /> | <img src="./data/gifs/lpr.gif" alt="License plate recognization" style="height=350px" /> |

|                           Body Pose                          |               Vehicle Detection               |
| :----------------------------------------------------------: | :-----------------------------------------: |
| <img src="./data/gifs/body_pose.gif" alt="Body Pose" style="height=350px" /> | <img src="./data/gifs/vehicle_cts.gif" alt="Vehicle Detection" style="height=350px" /> |


## Best Practices ##

### **How to build a classic classification or detection application based on CNStream?** ###

You should find a sample from ``samples/simple_run_pipeline/simple_run_pipeline.cpp`` that helps developers easily understand how to develop a classic classification or detection application based on CNStream pipeline.

This sample supports typical classification and detection networks like vgg resnet ssd fasterrcnn yolo-vx and so on.

This sample supports images or video file as input.

### **How to change the input video file?** ##

Modify the `files.list_video` file, which is under the `samples` directory, to replace the video path. Each line represents one stream. It is recommended to use an absolute path or use a relative path relative to the executor path.


## Documentation ##
[Cambricon Forum Docs](https://www.cambricon.com/docs/cnstream/user_guide_html/index.html)

Check out the Examples page for tutorials on how to use CNStream. Concepts page for basic definitions.

## Community forum ##
[Discuss](http://forum.cambricon.com/list-47-1.html) - General community discussion around CNStream.
