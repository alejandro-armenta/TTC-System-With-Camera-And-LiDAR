FP.1

I looked at each DMatch, and looked for previous Keypoint and current key point. Then I got bounding boxes for each keypoint. and added its IDs into a multimap.

Then I created a map of frecuencies, where I stored the second ID and its frecuency matched to the first ID.

Finally I just took the second ID with the biggest frecuency.

FP.2

I computed the median of the x distances for each the previous pointcloud and the current point cloud.
Then using these medians I computed TTC.

FP.3

I clustered keypoint matches based on the euclidean distance between current keypoint and previous keypoint. So to find a rigid transformation that does not change too much between frames.

FP.4

Then based on those matches i computed distance ratios between current and previos keypoints, then i picked median distance ratio to represent the change in scale of the object in front of the camera, and used it to compute TTC.

FP.5

The car is deaccelerating in the video, therefore the time to collision should be increasing. So that is the reason why sometimes with the constant velocity model, lidar's TTC estimation goes up. But of course, a better representation should be the constant acceleration model, which fits best with the video sequence.

FP.6

The Chart and spreadsheet are located in detector_ttc_comparison.gnumeric

There is a moment in the ORB FREAK estimation where the Camera's TTC is negative. Reason: So the distances between keypoints are shrinking, so that means that the algorithm thinks that it is separating itself from the preceding car, instead of reaching into it.

There were also some examples where Camera's TTC is too big. Reason: So the distance ratios are very close to 1, that means that there is no aparent change in distances. Then, the algorithm thinks it is not reaching the preceding car at all.




