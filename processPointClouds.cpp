// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

//Acuerdate que estas en el tiempo del procesador, por eso no se ha hecho la expanción
//Es hey aguantate no se ha hecho la expansión! mantén el template y expándelo!

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;

    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloudRegion);

    //Remove roof points

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers { new pcl::PointIndices };
    for(int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr inlierCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr otherCloud(new pcl::PointCloud<PointT>());

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    //This will extract the inliers indices!
    extract.setNegative(false);
    extract.filter(*inlierCloud);

    extract.setNegative(true);
    extract.filter(*otherCloud);

    //Plane and obstacles.
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(inlierCloud, otherCloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneAlex(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    int bestSize = 0;

	std::unordered_set<int> inliersResult;

	srand(time(NULL));

	// For max iterations 

	for(int it = 0; it < maxIterations; ++it)
	{
		std::unordered_set<int> line;

		// Randomly sample subset and fit line

		while(line.size() < 3)
		{
			line.insert(rand() % (cloud->points.size()));
		}

		auto iter = line.begin();

		PointT p0 = cloud->points[*iter];

		++iter;
		
		PointT p1 = cloud->points[*iter];

		++iter;

		PointT p2 = cloud->points[*iter];


		//NOTE: This is the line.

		float a1 = p1.x - p0.x;
		float a2 = p1.y - p0.y;
		float a3 = p1.z - p0.z;

		float b1 = p2.x - p0.x;
		float b2 = p2.y - p0.y;
		float b3 = p2.z - p0.z;

		float i = a2*b3 - a3*b2;
		float j = a3*b1 - a1*b3;
		float k = a1*b2 - a2*b1;

		float A = i;
		float B = j;
		float C = k;
		float D = -(i*p0.x + j*p0.y + k*p0.z);

		for(int j = 0; j < cloud->points.size(); ++j)
		{
			//auto lineIt = line.find(i);
			if(line.count(j) > 0)
			{
				continue;
			}

			PointT p = cloud->points[j];
 
			// Measure distance between every point and fitted line
			float d = fabs(A*p.x + B*p.y + C * p.z + D) / sqrt(A*A + B*B + C*C);

			// If distance is smaller than threshold count it as inlier

			if(d < distanceThreshold)
			{
				line.insert(j);
			}
		}

		if(line.size() > inliersResult.size())
		{
			inliersResult = line;
		}
	}

    pcl::PointIndices::Ptr inliers { new pcl::PointIndices };

    for(int index : inliersResult)
    {
        inliers->indices.push_back(index);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

	// Return indicies of inliers from fitted line with most inliers
	
	return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    //NOTE: Esto los crea en el heap!
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());


    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    
    //Segment the largest planar component from the remaining cloud

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size()==0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(std::vector<point_processed>& processedPoints, int index, std::vector<int>& cluster, KdTree* tree, float distanceTol)
{
	processedPoints[index].processed = true;
	cluster.push_back(index);

	std::vector<int> nearbyPoints = tree->search(processedPoints[index].point, distanceTol);

	for(int i = 0; i < nearbyPoints.size(); ++i)
	{
		if(processedPoints[nearbyPoints[i]].processed == false)
		{
			Proximity(processedPoints, nearbyPoints[i], cluster, tree, distanceTol);
		}
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringAlex(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters_clouds;

    std::vector<std::vector<int>> clusters;
	std::vector<point_processed> processedPoints(cloud->points.size());

    KdTree* tree = new KdTree;
  
    for (int i=0; i< cloud->points.size(); i++)
    {
        std::vector<float> point { cloud->points[i].x, cloud->points[i].y, cloud->points[i].z };
    	tree->insert(point,i);

		processedPoints[i].point = point;
		processedPoints[i].processed = false;
    }

	for(int index = 0; index < cloud->points.size(); ++index)
	{
		if(processedPoints[index].processed == false)
		{
			std::vector<int> cluster;
			Proximity(processedPoints, index, cluster, tree, clusterTolerance);
            if(cluster.size() >= minSize && cluster.size() <= maxSize)
			    clusters.push_back(cluster);
		}
	}

    for(std::vector<int>& cluster : clusters)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);

        for(int index : cluster)
        {
            cluster_cloud->points.push_back(cloud->points[index]);
        }

        cluster_cloud->width = cluster.size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;

        clusters_clouds.push_back(cluster_cloud);
    }

	return clusters_clouds;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    ec.extract(cluster_indices);


    for(int i = 0; i < cluster_indices.size(); ++i)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

        for(const auto& j : cluster_indices[i].indices)
        {
            cluster->points.push_back(cloud->points[j]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}