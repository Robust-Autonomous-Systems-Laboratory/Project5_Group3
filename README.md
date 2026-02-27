# Project 5: Laser Range Finder Intrinsic Parameter Estimation

**Anders Smitterberg & Progress Munoriarwa**  
EE5531 Intro to Robotics

---

## 1. Introduction and Methodology

### Beam Model Overview

Thrun's beam model is introduced as a method of charactizing LiDAR performance. Thrun characterizes a lidar using several parameters p hit pshort p max and prand. These four parameters respond to individual probability distributions what when combined form the probability model for the entire LiDAR. it is in theory possible to determine all of these parameters experimentally, and then develop a good emasure for the LiDar. A Good LiDAR in my mind should have a very narrow tall gaussian distribition around the nominally correct measurement value, it should dominate the measurement, and is lilkey sufficient for characterizing the performance of the model, and is likley the only thing we will be able to measure using our measuring setup.

### Calibration Approach

- Place robot perpendicular to a flat wall at a known distance
- Record /scan data to a ROS2 bag at multiple distances
- Offline: extract front-facing beams, fit Gaussian, estimate sigma_hit
- Compare distributions against beam model predictions

### Data Collection Procedure

| Distance (m) | Bag file | Duration (s) | Approx. samples |
|:---:|---|:---:|:---:|
| 0.5 | `data/rosbag_0_5m` | ~ 10 seconds | 100 samples |
| 1.0 | `data/rosbag_1m`   | ~ 10 seconds | 105 samples |
| 2.0 | `data/rosbag_2m`   | ~ 10 seconds | 121 samples |


Using two meter sticks and several two by four pieces of lumber we attempted to position the front of the LDS as close to the nominal distance as possible. using the rostopic echo command we then fine tuned the distance until the first measurement of the bag was as close as possible to the nominal distance. We ensured the robot was perpendicular to the wall by sight. The scan rate was approximately 5hz. By scanning for approximately ten seconds for each bag we got a reasonable amount of data. In hindsight a much longer time would have been preferred.

### Challenges and How They Were Addressed

There were quite a few challenges when setting up the measurements, first of all it was difficult to know precicely where the reference of the LiDAR was, and it was difficult to align it exactly and get it perpendicular to the wall. we set up two meter sticks on a few two by fours to elevate the meter stick to the correct heigth to get us as close as possible. To get as close to the nominal measurement value as possible we rostopic echoed the /scan command once and looked at the first value corresponding to 0.0 degrees, which, in theory was dead ahead. We then tried to move the robot precicely to the nominal measurement value as possible. As you can see however from our results there were still some substantial systemic bias issues in the measurements.  



---

## 2. Histogram Analysis


### 0.5 m

![Histogram 0.5 m](analysis/figures/hist_0_5m.png)

The distribution's mode is approximately at 501mm. the distribution I would not call gaussian. there are visible tails and clusters, but the total range of the measurements is still quite small at about 5mm. Interestingly there are very few short measurements. 

### 1.0 m

![Histogram 1.0 m](analysis/figures/hist_1_0m.png)

The measured distribution reads long, the mean is at 1007 mm, which is substantially longer than anticipated, this is likley measurement error. The distribution has much less variance than the 0.5m distribution, but also has more clearly defined tails.

### 2.0 m

![Histogram 2.0 m](analysis/figures/hist_2_0m.png)

It appears as though the standard deviation increases greatly in this distribution. the mean is slighyly higher than the nominal measurement value, and the distribution appears much more gaussian. 

### Distribution Shape Discussion

Across all the measurements it appears although the p_hit dominates. There do not appear to be any short measurements, other than potentially a tail on the 1.0m distribution. Additionally there didn't appear to be any "long" measurements on the sensor. if there were they were already likley clipped by the firmware. If I had to characteize these distribution I would have preferred longer rosbags, and I suspect they would have converged to a gaussian appearing distribution. I believe that p_hit appears to dominate, and for simplicity's sake I believe is sufficient to characterize this sensor's performance.

---

## 3. Parameter Estimation and Results

### Estimated Parameters per Distance

| z\* (m) | N | Mean (m) | Bias (m) | σ_hit (m) | Outlier rate | Short rate |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| 0.5 | 392 | 0.5015 | 0.0015 | 0.0013 | 0.00 | 0.00 |
| 1.0 | 416 | 1.0068 | 0.0068| 0.0015 | 0.00 | 0.00 |
| 2.0 | 481 | 2.0034 | 0.0034 | 0.0036 | 0.00 | 0.00 |

### σ_hit vs Distance

![sigma vs distance](analysis/figures/sigma_vs_distance.png)


### Proposed Uncertainty Model
The Beam Model represents the probability of a LiDAR measurement using several components, but in this implementation I focused primarily on the p_hit
component. The  \(\p_{hit}\) term models accurate range measurements as a Gaussian distribution centered at the expected distance z_star
, with standard deviation signma_hit. The parameter \(\sigma_{hit}\) represents the sensor’s measurement noise and determines how spread out the measurements are around the true distance. For calibration, we placed the LiDAR at a known target distance and collected range measurements within a small angular window around the target angle. We then used Welford’s algorithm to compute the running mean and variance of the measurements. From the running variance, we calculated  \(\sigma_{hit}\)
,which quantifies the random noise in the sensor readings. By continuously updating the running statistics, We were able to estimate how much the measurements fluctuated around the true distance. A small \(\sigma_{hit}\) indicates that the sensor is precise with low noise, while a larger  \(\sigma_{hit}\) indicates greater measurement uncertainty. We also monitored the difference between the running mean and the known target distance to observe any systematic bias, but the primary calibration parameter used in the beam model was \(\sigma_{hit}\) Overall, the calibration process allowed me to estimate the Gaussian noise parameter  \(\sigma_{hit}\) which is essential for accurately modeling the likelihood of LiDAR measurements in the beam model.

### Outlier Rates and Discussion


I evaluated the LiDAR performance at three known target distances: 0.5 m, 1.0 m, and 2.0 m. For each case, I analyzed the number of samples collected, the estimated \(\sigma_{hit}\), the measurement error, and the number of detected outliers (defined as measurements exceeding 3 standard deviations from the running mean).

*** 0.5 m Results ***

- **Mean measurement:** 0.50077 m  
- **Measurement error:** 0.00077 m  
- **\(\sigma_{hit}\):** 0.00042 m  
- **Samples:** 173  
- **Outliers:** 0  

At 0.5 m, the sensor showed extremely small measurement error and very low noise. The estimated \(\sigma_{hit}\) was the smallest among all tested distances, indicating very high precision at short range. No outliers were detected, suggesting stable and consistent readings.

---

*** 1.0 m Results ***

- **Mean measurement:** 1.00674 m  
- **Measurement error:** 0.00674 m  
- **\(\sigma_{hit}\):** 0.00085 m  
- **Samples:** 184  
- **Outliers:** 3  

At 1.0 m, a small systematic bias was observed, with the sensor slightly overestimating the distance by about 6.7 mm. The noise level increased compared to 0.5 m, as shown by the larger \(\sigma_{hit}\). Three outliers were detected, which represents:

\[
\frac{3}{184} \approx 1.63\%
\]

This is still a very low outlier rate, indicating generally stable performance.


*** 2.0 m Results ***

- **Mean measurement:** 2.00061 m  
- **Measurement error:** 0.00061 m  
- **\(\sigma_{hit}\):** 0.00125 m  
- **Samples:** 239  
- **Outliers:** 0  

At 2.0 m, the systematic bias was nearly zero. However, \(\sigma_{hit}\) increased further, indicating that measurement noise grows with distance. Despite the increased variance, no outliers were detected, showing consistent performance.

The results show that:

- Measurement noise (\(\sigma_{hit}\)) increases as distance increases.  
- Systematic bias is very small at all tested distances.  
- Outlier rates are extremely low (0 - 1.63%).  
- The sensor performs very consistently within the tested range.  

The increase in \(\sigma_{hit}\) with distance is expected because LiDAR measurement uncertainty typically grows with range due to beam divergence and signal attenuation.
But Overall, the LiDAR demonstrates high precision, minimal systematic bias, and a very low outlier rate, indicating reliable performance for use in the Gaussian \(p_{hit}\) component of the beam model.


## 4. Analysis Questions

**Q1. Does the measurement distribution match the Gaussian assumption of p_hit?**

Yes with some caveats. It appears as thought the gaussian distribution appears with larger measurement distances. I believe with more measurements over a longer time we would be more likley to see this emerge. I think that a gaussian centered on the mean is a reasonable assumption for the distributions. The only real outlier in my mind is 1m which has some distinct looking tails. 

**Q2. How does measurement uncertainty vary with distance?**

In general it appears that as distance increases so does measurement uncertainty. This is a good thing as it means our model is working, and is what we would expect. The return power would be less for a device that is further away, and you would expect the uncersainty to increase. this is clearly visible in our model. It appears through our limited samples that at least for small changes like this then a linear model for uncaertainty is appropriate. 


**Q3. Were there systematic biases? How would you correct for them?**

Yes, for our measurement setups some of them were not set up correctly, introducting systemic bias into our measurements, additionally it was difficult to determine exactly where the reference of the LiDAR was, and as such it was difficult to precicely or accurately position the robot. we attempted to echo the /scan topic to position the robot at exactly the reference distance, but even with that it was extremely difficult to move the robot precisely.

---

## 5. Usage Instructions

### Build the ROS2 Package

```bash
cd ~/proj5_ws
colcon build --packages-select lidar_calibration
source install/setup.bash
```

### Run the Calibration Node

Run with default parameters (target distance = 1.0 m, target angle = 0 rad, angle window = 0.1 rad):

```bash
ros2 run lidar_calibration calibration_node
```

Run with a specific target distance (e.g. 0.5 m):

```bash
ros2 run lidar_calibration calibration_node --ros-args -p target_distance:=0.5
```

Run with all parameters specified:

```bash
ros2 run lidar_calibration calibration_node --ros-args \
  -p target_distance:=2.0 \
  -p target_angle:=0.0 \
  -p angle_window:=0.1
```

### Play Back a Recorded Bag

```bash
ros2 bag play data/rosbag_1m
```

Then in a second terminal, run the node with the matching target distance:

```bash
ros2 run lidar_calibration calibration_node --ros-args -p target_distance:=1.0
```

### Monitor Published Topics

```bash
# Current measurement error (z - z*)
ros2 topic echo /calibration/range_error

# Running sigma_hit estimate
ros2 topic echo /calibration/statistics
```

---

*Generative artificial intelligence was used by Anders Smitterberg to assist with developing and debugging the `calibration_node.py` and `analyze_lidar_bag.py` scripts, and for formatting this README.*
