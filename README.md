# Project 5: Laser Range Finder Intrinsic Parameter Estimation

**Anders Smitterberg · Progress Munoriarwa**  
EE5531 Intro to Robotics

---

## 1. Introduction and Methodology

### Beam Model Overview

Thrun's beam model is introduced as a method of characterizing LiDAR performance. Thrun characterizes a LiDAR using several parameters: $p_{hit}$, $p_{short}$, $p_{max}$, and $p_{rand}$. These four parameters correspond to individual probability distributions that, when combined, form the probability model for the entire LiDAR. It is, in theory, possible to determine all of these parameters experimentally to develop a robust model for the LiDAR. A good LiDAR should have a very narrow, tall Gaussian distribution around the nominally correct measurement value; this distribution should dominate the measurement model and is likely the primary characteristic we can measure using our current setup.

### Calibration Approach

- Place robot perpendicular to a flat wall at a known distance.
- Record `/scan` data to a ROS 2 bag at multiple distances.
- Offline: extract front-facing beams, fit Gaussian, and estimate $\sigma_{hit}$.
- Compare distributions against beam model predictions.

### Data Collection Procedure

| Distance (m) | Bag file | Duration (s) | Approx. samples |
|:---:|---|:---:|:---:|
| 0.5 | `data/rosbag_0_5m` | ~10 seconds | 199 samples |
| 1.0 | `data/rosbag_1m`   | ~10 seconds | 207 samples |
| 2.0 | `data/rosbag_2m`   | ~10 seconds | 239 samples |

Using two meter sticks and several 2x4 pieces of lumber, we attempted to position the front of the LDS as close to the nominal distance as possible. Using the `ros2 topic echo` command, we fine-tuned the distance until the first measurement of the bag was as close as possible to the nominal distance. We ensured the robot was perpendicular to the wall by sight. The scan rate was approximately 5 Hz. By scanning for approximately ten seconds for each bag, we obtained a reasonable amount of data, though in hindsight a longer duration would have been preferred.

### Challenges and How They Were Addressed

There were several challenges when setting up the measurements. First, it was difficult to identify the precise reference point of the LiDAR, making it hard to align it exactly perpendicular to the wall. We used meter sticks elevated on 2x4s to match the LiDAR's height for better accuracy. To get as close to the nominal measurement value as possible, we used `ros2 topic echo /scan` and monitored the value corresponding to 0.0 degrees (dead ahead). We then manually adjusted the robot's position. Despite these efforts, some systematic bias remained in the measurements, as seen in our results.



---

## 2. Histogram Analysis


### 0.5 m

![Histogram 0.5 m](analysis/figures/hist_0_5m.png)

The distribution's mode is approximately at 501 mm. We would not call this distribution Gaussian; there are visible tails and clusters, but the total range of the measurements is still quite small at about 5 mm. Interestingly, there are very few short measurements. 

### 1.0 m

![Histogram 1.0 m](analysis/figures/hist_1_0m.png)

The measured distribution reads long; the mean is at 1007 mm, which is substantially longer than anticipated—this is likely measurement error. The distribution has much less variance than the 0.5 m distribution but also has more clearly defined tails.

### 2.0 m

![Histogram 2.0 m](analysis/figures/hist_2_0m.png)

It appears as though the standard deviation increases significantly in this distribution. The mean is slightly higher than the nominal measurement value, and the distribution appears much more Gaussian. 

### Distribution Shape Discussion

Across all the measurements, it appears as though $p_{hit}$ dominates. There do not appear to be any short measurements, other than potentially a tail on the 1.0 m distribution. Additionally, there didn't appear to be any "long" measurements on the sensor; if there were, they were likely already clipped by the firmware. If we had to characterize these distributions, we would have preferred longer ROS bags, and we suspect they would have eventually converged to a Gaussian-like distribution. We believe that $p_{hit}$ dominates, and for simplicity's sake, it is sufficient to characterize this sensor's performance.

---

## 3. Parameter Estimation and Results

### Estimated Parameters per Distance

| $z^*$ (m) | $N$ | Mean (m) | Bias (m) | $\sigma_{hit}$ (m) | Outlier rate | Short rate |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| 0.5 | 199 | 0.5008 | 0.0008 | 0.0004 | 0.00% | 0.00% |
| 1.0 | 207 | 1.0067 | 0.0067 | 0.0009 | 9.18% | 0.00% |
| 2.0 | 239 | 2.0006 | 0.0006 | 0.0012 | 0.00% | 0.00% |

### $\sigma_{hit}$ vs. Distance

![sigma vs distance](analysis/figures/sigma_vs_distance.png)


### Proposed Uncertainty Model

The beam model represents the probability of a LiDAR measurement using several components, but in our implementation, we focused primarily on the $p_{hit}$ component. This term models accurate range measurements as a Gaussian distribution centered at the expected distance $z^*$, with a standard deviation $\sigma_{hit}$ that represents the measurement noise.

Through our offline analysis of the ROS bags, we observed a clear relationship between the target distance and the resulting measurement noise. By fitting a linear model to our estimated $\sigma_{hit}$ values across the 0.5 m to 2.0 m range, we determined the following uncertainty model:

$$\sigma_{hit}(z^*) = 0.00023 + 0.00053 \cdot z^*$$

This linear model fits our data quite well for the tested range, capturing the expected trend where measurement uncertainty grows as the distance increases. This growth is typically attributed to beam divergence and a reduction in returned signal power at greater distances. By adopting this distance-dependent $\sigma_{hit}$, we can more accurately characterize the likelihood of LiDAR measurements using standard deviation.

### Outlier Rates and Discussion


We evaluated the LiDAR performance at three known target distances: 0.5 m, 1.0 m, and 2.0 m. For each case, we analyzed the number of samples collected, the estimated $\sigma_{hit}$, the measurement error, and the number of detected outliers (defined as measurements exceeding 3 standard deviations from the running mean).

##### 0.5 m Results

- **Mean measurement:** 0.5008 m  
- **Measurement error:** 0.0008 m  
- **$\sigma_{hit}$:** 0.0004 m  
- **Samples:** 199  
- **Outliers:** 0  

At 0.5 m, the sensor showed extremely small measurement error and very low noise. The estimated $\sigma_{hit}$ was the smallest among all tested distances, indicating very high precision at short range. No outliers were detected, suggesting stable and consistent readings.


##### 1.0 m Results

- **Mean measurement:** 1.0067 m  
- **Measurement error:** 0.0067 m  
- **$\sigma_{hit}$:** 0.0009 m  
- **Samples:** 207  
- **Outliers:** 19  

At 1.0 m, a small systematic bias was observed, with the sensor slightly overestimating the distance by about 6.7 mm, almost certainly due to our shoddy initial setup. The noise level increased compared to 0.5 m, as shown by the larger $\sigma_{hit}$. Nineteen outliers were detected, which represents:

$$\frac{19}{207} = 9.18\%$$

While higher than at other distances, this is still a relatively low outlier rate, though it highlights the distinct tails observed in the 1.0 m distribution.


##### 2.0 m Results

- **Mean measurement:** 2.0006 m  
- **Measurement error:** 0.0006 m  
- **$\sigma_{hit}$:** 0.0012 m  
- **Samples:** 239  
- **Outliers:** 0  

At 2.0 m, the systematic bias was nearly zero. However, $\sigma_{hit}$ increased further, indicating that measurement noise grows with distance. Despite the increased variance, no outliers were detected, showing consistent performance.

The results show that:

- Measurement noise $\sigma_{hit}$ increases as distance increases.  
- Systematic bias exists, but is small at all tested distances.  
- Outlier rates are low (0 - 9.18%).  
- The sensor performs well within the tested range.  

The increase in $\sigma_{hit}$ with distance is expected because LiDAR measurement uncertainty typically grows with range due to beam divergence and signal attenuation. Overall, the LiDAR demonstrates high precision, minimal systematic bias, and a low outlier rate, indicating reliable performance for use in the Gaussian $p_{hit}$ component of the beam model.

---

## 4. Analysis Questions

**Q1. Does the measurement distribution match the Gaussian assumption of $p_{hit}$?**

Yes, with some caveats. It appears as though the Gaussian distribution becomes more evident at larger measurement distances. We believe that with more measurements over a longer time, we would be more likely to see this emerge. We think that a Gaussian centered on the mean is a reasonable assumption for the distributions. The only real outlier in our view is the 1.0 m case, which has some distinct looking tails. 

**Q2. How does measurement uncertainty vary with distance?**

In general, it appears that as distance increases, so does measurement uncertainty. This is expected and suggests our model is accurate. The return power would be less for an object that is further away, and you would expect the uncertainty to increase; this is clearly visible in our results. It appears through our limited samples that, at least for small changes like this, a linear model for uncertainty is appropriate. 


**Q3. Were there systematic biases? How would you correct for them?**

Yes, some of our measurement setups introduced systematic bias into our measurements. Additionally, it was difficult to determine exactly where the reference point of the LiDAR was, and as such, it was difficult to accurately position the robot. We attempted to echo the `/scan` topic to position the robot at exactly the reference distance, but even with that, it was extremely difficult to move the robot with high precision.

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
