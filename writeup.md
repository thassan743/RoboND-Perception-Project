## Project: Perception Pick & Place
### Taariq Hassan (thassan743)

---

[//]: # (Image References)

[image1]: ./misc/composite.png
[image2]: ./misc/5_clusters.png
[image3]: ./misc/confusion_matrix.png
[image4]: ./misc/world3_obj.png
[image5]: ./misc/world1.png
[image6]: ./misc/world2.png
[image7]: ./misc/world3.png

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
The code begins in the `pcl_callback` function of `project_template.py` found [here](https://github.com/thassan743/RoboND-Perception-Project/blob/master/pr2_robot/scripts/project_template.py). The first step is to convert the ROS message to a PCL compatible point cloud so that we can make use of the powerful Point Cloud Library. My final filtering and plane fitting pipeline is very similar to that implemented in Exercise 1, with the exception of an additional PassThrough filter and the addition of the Statistical Outlier filter. I will briefly discuss each step of the pipeline in the order of implementation.

The first step was to do **Voxel Grid Downsampling Filter** on the input point cloud. Downsampling reduces the size of the point cloud by averaging points within a "voxel". The aim is that the resulting point cloud is significantly smaller (fewer points) but still provides a good representation of the original scene. The only parameter to tune here is the **leaf size** which defines the dimensions (in meters) of the cube that the 3d-space is partitioned into. It was found that a leaf size of **0.005** provided a good trade-off between size and detail. The resulting point cloud can be seen in figure 1 of the image below.

Step 2 was to apply a **PassThrough Filter** in the **z-axis** (vertical) to remove unnecessary data from the point cloud. The region of interest is the table and all objects on it. Therefore, we need to define the **minimum and maximum z-axis** values for the region we wish to keep. These were chosen to be **0.6 and 2.0** respectively, and were determined using a raw point cloud capture and `pcl_viewer`. The result of applying this filter can be seen in figure 2a of the image below. It can clearly be seen that all the data representing the floor and the base of the table were removed. Additionally a second PassThrough Filter was added in the **x-axis** to remove parts of the two dropboxes from the scene. Without removing the dropboxes, they will be identified as objects later in the pipeline. The parameters for the x-axis filter were a **minimum and maximum of 0.33 and 0.9** respectively, once again determined using a raw point cloud capture and `pcl_viewer`. The result of applying this filter can be seen in figure 2b of the image below. We can see that the corners of the dropboxes that were visible are now gone.

Step 3 was to do **RANSAC Plane Segmentation** on the scene with the aim of separating the table and the objects into separate point clouds. Since we know that there is a table with objects on it, we can use RANSAC to fit a plane to the tabletop, thus separating the objects from the table. The only parameter to tune here is the **distance threshold** which defines the distance from the plane a point needs to be in order to be classified as part of the table or part of the objects. A good value was found to be **0.005**. The result of applying RANSAC is two separate point clouds, one for the table and one for the objects. These point clouds can be seen in figure 3 of the image below. The upper half of the image shows the segmented table and the lower half shows the objects.

Step 4 was to apply a **Statistical Outlier filter** on the objects point cloud. Since the RGB-D camera in the project environment simulates a real camera, the point cloud it returns includes noise. Therefore, the purpose of the Statistical Outlier filter is to remove most of this noise. It was recommended in the lecture notes that this filter be applied first in the pipeline, however, I found better results when applying it at the end. Looking at figure 3 in the image we can see that the objects output of RANSAC still has some points which should belong to the table. This can be regarded as noise in the objects point cloud and removed using this Outlier filter. For this reason, the outlier filter was applied last. This filter has two parameters, **k** which is the number of neighbouring points to analyse and **standard deviation multiplier** which defines the threshold distance for a point to remain in the cloud or be considered noise and removed. I found values of **k = 10 and std_dev_multiplier = 0.1** worked well. The result of applying this filter can be seen in figure 4 of the image below. We can see that most of the noise that was present in the original point cloud is now gone.

The image below shows the output of each of the steps described above. The original images can be found in the [misc](https://github.com/thassan743/RoboND-Perception-Project/tree/master/misc) folder.

![alt text][image1]

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  
In this section, the objects point cloud returned from the Statistical Outlier filter above is further segmented into individual objects using the clustering technique. We perform Euclidean Clustering on a "white" point cloud which is the object point cloud without any colour data since clustering relies only on the pixel locations in 3d-space. The parameters for Euclidean Clustering are **ClusterTolerance, MinClusterSize and MaxClusterSize**. The Min and Max parameters define the size range of possible clusters. These were set to **50** and **3000** respectively since the objects vary in size. **ClusterTolerance** defines how close a point needs to be to a cluster to be considered as part of that cluster. A value of **0.03** was found to work well. Once the individual clusters have been extracted from the "white" point cloud, each cluster is given a unique colour.

The result of the Euclidean Clustering step can be seen in the image below. We can see that each object has been given a unique colour which means that each object is a separate cluster.

![alt text][image2]

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
To generate object features, the `capture_features.py` script from the `sensor_stick` project was used. The script was modified to generate training sets with varying lengths of features and different colour spaces which allowed me to experiment with different models. In the end, I found the best performance from a training set with **100 features** per object or **800 features** in total, using the **HSV Colour Space**. The SVM model was then trainied on this feature set using an **RBF kernel**. This gave me an accuracy score of **0.88875**. It is worth noting here that I generated training sets with 5, 50, 100, 500 and 1000 features per object, for both the RGB and HSV colour spaces and trained a model for each of these using both the Linear and RBF kernel. And while the model I chose did not have the highest accuracy score, it performed better than most in the prediction step of the pipeline. This is most likely attributed to some models overfitting the data. The figure below shows the Confusion matrices for my selected model.

![alt text][image3]

The trained model is then used to identify and label each object that was returned from the Euclidean Clustering step. The image below shows the successful classification of the objects in the `test3.world` environment of the project.

![alt text][image4]

Finally, the necessary point clouds are converted and published to ROS.

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.



![alt text][image5]
![alt text][image6]
![alt text][image7]

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



