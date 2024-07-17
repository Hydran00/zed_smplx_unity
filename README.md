# SMPL-X body tracking with Zed camera

This is the Unity Project for using the body tracking module of the Stereolabs Zed Unity module for the animation of a SMPL-X model.
It supports just body joint angles tracking since the Zed Body Tracking module generates just the skeleton. The accuracy of the depth of the model is higher than using SOTA RGB methods like [HybriIK](https://github.com/Jeff-sjtu/HybrIK) since the body tracking module leverage on the stereo vision of the Zed camera.

https://github.com/user-attachments/assets/8a3227f0-13d5-4d31-9468-902b1a2b70d3

## Requirements
- [CUDA](https://developer.nvidia.com/cuda-downloads)
- [ZED SDK](https://www.stereolabs.com/developers/release)
## Quick start
### Unity Side (client)
1. Create a new 3D project in Unity
2. Download the SMPL-X Unity plugin from [here](https://download.is.tue.mpg.de/download.php?domain=smplx&sfile=SMPLX_UnityProject_20210617.zip) (You need to sign up)
3. Unzip the content and import the content of `SMPLX-Unity` in the `Assets` folder of your project
4. Download the Zed Livelink Unity package from [here](https://github.com/stereolabs/zed-unity-livelink/releases/)
5. Import the package in your project from `Assets -> Import Package -> Custom Package`
6. If you have compilation errors related to the missing of the newtonsoft package do the following as explained [here](https://github.com/stereolabs/zed-unity-livelink):  
  - Open the Package Manager (Window -> Package Manager)
  - Click on the "+" button and select "Add package from name"
  - Enter `com.unity.nuget.newtonsoft-json` and click on `Add`

7. Go to `SMPLX-Unity/Assets/SMPLX/Models` and click on `smplx-male.fbx`. On the inspector window set the following options as described [here](https://files.is.tue.mpg.de/nmahmood/smpl_website/How-to_SMPLinUnity.pdf):  
  - Meshes:
    - Mesh Compression: Off
    - Read/Write Enabled: On  
  - Rig:
    - Optimize Mesh: Off
    - Import Blendshapes: On
    - Keep Quads: Off
    - Weld Vertices: Off
    - Animation Type: Humanoid  
  - Normals & Tangents:
    - Normals: Calculate
    - Smoothing Angle: recommended value=180 (higher numbers help with faster and
    smoother rendering)
    - Tangents: None  
  - Materials:  
    - Material creation mode: None  
And click `Apply`
8. Go to `SMPLX-Unity/Assets/SMPLX/Prefabs` and in `smplx-male.prefab` (or `smplx-female.prefab`) under `Animation` select `BodyTrackingAnimationController` as the controller
9. Also add the component `ZED Skeleton Animator` script for the prefab 
9. Open the scene in `Assets/ZEDFusion/Scenes/Body Tracking Fusion.unity`
10. Select `FusionManager` from the hierarchy and in the inspector expand `Avatar Controls` and delete the already existing avatars.
11. Drag and drop the `smplx-male.prefab` (or `smplx-female.prefab`) prefab in `SMPLX-Unity/Assets/SMPLX/Prefabs` in the `Avatars` list in the `FusionManager` inspector

### Your pc (server)
1. Clone `zed-unity-livelink` repo:
```
git clone https://github.com/stereolabs/zed-unity-livelink.git
```
2. Build ZED Unity livelink Mono (If you would like to use multiple cameras you can try with zed-unity-livelink-fusion)
```bash
cd zed-unity-livelink/zed-unity-livelink-mono
mkdir build
cd build
cmake ..
make
```
3. Start the ZED Unity Livelink server
```bash
./ZED_LiveLink_Mono
```
4. You will see the ip address and port used for multicast communication. Annotate them and set them in the `FusionManager` inspector in Unity in the `ZED Streaming Client` section

## Retrieving the SMPL-X model using Python
Inside the `utils` folder you can find the C# program called `SendParams.cs` that I use in Unity to send the parameters of the SMPL-X model to the `smplx_tracking.py` script.  
I used ROS2 and [Unity TCP Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Connector) but you can rewrite it using a simple TCP client-server architecture.  


https://github.com/user-attachments/assets/2f34e494-09b7-45c2-9365-01a61793e382


### Python dependecies
- smplx
- open3d

You can install them using pip:
```bash
pip install smplx open3d
```

The C# program works if change the visibility of the following variables in the `SMPLX.cs` script under `Assets/SMPLX-Unity/Assets/SMPLX/Scripts` from `private` to `public`:
```
_bodyJointNames
_transformFromName
```

Also you need to define a constant name for the prefab instantiated by the ZED Unity plugin.  
You need to add the following line in the `SkeletonHandler.cs` script under `Assets/ZEDFusion/Scripts`:
```csharp
humanoid.name = "Humanoid";
```
In this way the `SendParams.cs` program can retrieve the gameobject correctly at run-time.

### Run the script
I avoided creating a ROS2 package since it is a simple script. You can run it using the following command:
```bash
python3 smplx_tracking smplx_tracking.py
```
Then press play in Unity and finally run the ROS TCP Endpoint to create the connection. 
### Dump SMPL-X parameters
I created a ROS2 service in the `smplx_tracking.py` script that dumps the SMPL-X parameters in a file. 
```
ros2 service call /dump_smplx_parameters std_srvs/srv/Empty "{}"
```
