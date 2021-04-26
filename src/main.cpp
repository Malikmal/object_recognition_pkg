#include <iostream>
#include <vector>
#include <algorithm>
#include <string>

#include "fann.h"
#include "floatfann.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/recognition/linemod/line_rgbd.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>


// argument 1 => file ex : scene_mug_table.pcd 

struct info{
  int no;
  std::string category;
  float score;
  std::vector<float> histogram;
} ;
typedef std::pair<info, pcl::PointCloud<pcl::PointXYZ>::Ptr> modelsDetail;

// typedef std::pair<info, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> modelRaw;

int BoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr, pcl::visualization::PCLVisualizer &viewer)
{
    // // ref : https://stackoverflow.com/questions/40522181/to-extract-a-set-of-feature-and-cluster-data-from-the-given-point-cloud-data
    // // compute principal direction
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*point_cloud_ptr, centroid);
    std::cout <<  "centroid : " << centroid << std::endl;
    // Eigen::Matrix3f covariance;
    // computeCovarianceMatrixNormalized(*point_cloud_ptr, centroid,covariance);
    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance,  Eigen::ComputeEigenvectors);
    // Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    // eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // // move the points to the that reference frame
    // Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    // p2w.block<3,3>(0,0) = eigDx.transpose();
    // p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
    // pcl::PointCloud<pcl::PointXYZ> cPoints;
    // pcl::transformPointCloud(*point_cloud_ptr, cPoints, p2w);

    // pcl::PointXYZ min_pt, max_pt;
    // pcl::getMinMax3D(cPoints, min_pt, max_pt);
    // const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // // final transform
    // const Eigen::Quaternionf qfinal(eigDx);
    // const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

    // // draw the cloud and the box
    // // pcl::visualization::PCLVisualizer viewer;
    // // viewer.addPointCloud(point_cloud_ptr);
    // viewer.addCube(-tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z);//, "AABB");
    // // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
    // // viewer.spin();



    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*point_cloud_ptr, minPt, maxPt);
    // std::cout << "Min x: " << minPt.x << std::endl;
    // std::cout << "Max x: " << maxPt.x << std::endl;
    // std::cout << "Min y: " << minPt.y << std::endl;
    // std::cout << "Max y: " << maxPt.y << std::endl;
    // std::cout << "Min z: " << minPt.z << std::endl;
    // std::cout << "Max z: " << maxPt.z << std::endl;
    std::string bboxId = "BBOX" + std::to_string(rand() % 100);
    std::string textId = "TEXT" + std::to_string(rand() % 100);
    // std::cout << "bboxId : " << bboxId << std::endl;
    viewer.addCube(minPt.x, maxPt.x,  -maxPt.y , -minPt.y, -maxPt.z, -minPt.z, 1.0, 1.0, 1.0, bboxId, 0 );
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, bboxId);
    // viewer.addText("asdasdasdasd asdasdasdasd adasdas das d", maxPt.x, maxPt.y, textId);
    viewer.addText3D(textId, pcl::PointXYZ(maxPt.x, -maxPt.y, -minPt.z), 0.05, 255.0, 1.0, 1.0,  textId);

    // viewer.addPointCloud(*point_cloud_ptr);
    return(1);
}

int 
main (int argc, char** argv)
{

    /** percobaan **
    // std::vector<std::string> listDataSet;
    // std::ifstream listDataSetFile("listDataSetv2.1.txt");
    // for (std::string line ; getline(listDataSetFile, line);)
    // {
    //     listDataSet.push_back(line);
    // }
    // // std::cout << listDataSet.size();
    // fann_type result[listDataSet.size()] = {0, 0, 0.2, 0.4, 0.9, 0.1, 0, 0, 0, 0, 0, 0, 0, 0};

    // std::vector< std::pair<std::string, std::vector<float> > > resultDataSet; 
    // for (int i = 0; i < listDataSet.size(); i++)
    // {
    //     // float tmp = (float);
    //     // result[i] = (2/i);
    //     // resultDataSet 
    //     std::pair<std::string, std::vector<float> > hist;
    //     std::vector<float> histBin;
    //     for(int j = 0; j < listDataSet.size(); j++)
    //     {
    //         histBin.push_back(result[j]);
    //     }
    //     hist.first = listDataSet[i];
    //     hist.second = histBin;
    //     resultDataSet.push_back(hist);
    // }

    // for ( auto it : resultDataSet)
    // {
    //     std::vector<std::string> resultsss;
    //     boost::algorithm::split(resultsss, it.first, boost::is_any_of("/"));
    //     std::cout << resultsss[1] << std::endl;
    //     it.first = resultsss[1];
    //     for(auto it2 : it.second)
    //     {
    //         std::cout << it2 << " " ;
    //     }
    //     std::cout << std::endl;
    //     std::cout << std::distance(it.second.begin(), std::max_element(it.second.begin(), it.second.end())) << std::endl;
    // }
*/
    pcl::visualization::PCLVisualizer viewer;

    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudRGB (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nan (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read (argv[1], *cloudRGB);
    reader.read (argv[1], *cloud);
    // pcl::copyPointC
    
    
    // copyPointCloud(cloudRGB, cloud);
    std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*


    //transform scene
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.rotate(Eigen::AngleAxisf(45, Eigen::Vector3f::UnitX()));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::transformPointCloud (*cloudRGB, *transformed_cloud, transform_2);

    // add original pointcloud to viewer
    viewer.addPointCloud(transformed_cloud);
    // viewer.addPointCloud(cloud);
    viewer.addCoordinateSystem (1.0);
    viewer.initCameraParameters ();
    viewer.setCameraPosition(0, 30, 0,    0, 0, 0,   0, 0, 1);
    viewer.setCameraFieldOfView(0.523599);
    viewer.setCameraClipDistances(0.00522511, 50);

    // Filter Removing NaN data Pointcloud.
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_nan, mapping);
    std::cerr << "Pointcloud after remove NaN : " << cloud->size() << " data points." << std::endl;

    // Filter object crop by z coordinate
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_nan);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 2.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_passthrough);
    std::cerr << "Pointcloud after cropped : " << cloud_passthrough->size() << " data points." << std::endl;
    //save passthrough filter
    pcl::io::savePCDFile("src/object_recognition_pkg/output/filter/cropping.pcd", *cloud_passthrough);


    // Filter downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud_passthrough);
    vg.setLeafSize (0.01f, 0.01f, 0.01f); // 1cm
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*
    //save the result of downsample voxel grid
    pcl::io::savePCDFile("src/object_recognition_pkg/output/filter/voxel_grid.pcd", *cloud_filtered);
    

    //trasnform cloud 45deg
    pcl::transformPointCloud (*cloud_filtered, *cloud_filtered, transform_2);


    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_filtered->size ();
    while (cloud_filtered->size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;


        //save planar [optional]
        std::stringstream ssPlanar;
        ssPlanar << "src/object_recognition_pkg/output/segmentation/planar" << i <<".pcd";
        writer.write<pcl::PointXYZ> (ssPlanar.str (), *cloud_plane, false); //*

        //save planar removed[optional]
        std::stringstream ssPlanarRemoved;
        ssPlanarRemoved << "src/object_recognition_pkg/output/segmentation/planar_removed" << i <<".pcd";
        writer.write<pcl::PointXYZ> (ssPlanarRemoved.str (), *cloud_f, false); //*

        i++;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    //// get the cluster models
    i = 0;
    std::vector<modelsDetail> dataClustered;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*

        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;

        modelsDetail modelClustered;
        modelClustered.second = cloud_cluster;
        modelClustered.first.no = i; 
        dataClustered.push_back (modelClustered);

        //save file of clustered object
        std::stringstream ss;
        ss << "src/object_recognition_pkg/output/cluster/cloud_cluster_" << i << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        i++;


        // BoundingBox(cloud_cluster, viewer);
        // viewer.addPointCloud(cloud_cluster);
    }


    //// VFH DESCRIPTOR  
    std::vector<modelsDetail> dataAddVFH;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    typedef pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> VFHEstimationType;
    VFHEstimationType vfhEstimation;
    i = 0;
    for(auto it : dataClustered )
    {
        //  Compute the normals
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        normalEstimation.setInputCloud (it.second);
        normalEstimation.setSearchMethod (tree);
        normalEstimation.setRadiusSearch (0.03);
        pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);
        normalEstimation.compute (*cloudWithNormals);
        // std::cout << "Computed " << cloudWithNormals->points.size() << " normals." << std::endl;
        
        // Setup the feature computation
        // Provide the original point cloud (without normals)
        vfhEstimation.setInputCloud (it.second);    // Provide the point cloud with normals
        vfhEstimation.setInputNormals(cloudWithNormals);    // Use the same KdTree from the normal estimation
        vfhEstimation.setSearchMethod (tree);
        // vfhEstimation.setRadiusSearch (0.2); // With this, error: "Both radius (.2) and K (1) defined! Set one of them to zero first and then re-run compute()"
        // Actually compute the VFH features
        pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhFeatures(new pcl::PointCloud<pcl::VFHSignature308>);
        vfhEstimation.compute (*vfhFeatures);
        // std::cout << "output points.size (): " << vfhFeatures->points.size () << std::endl; // This outputs 1 - should be 397!

        // Display and retrieve the shape context descriptor vector for the 0th point.
        // pcl::VFHSignature308 descriptor = vfhFeatures->points[0];
        // VFHEstimationType::PointCloudOut::PointType descriptor2 = vfhFeatures->points[0];
        // std::cout << descriptor << std::endl;

        // save histogram VFH of clustered object
        std::stringstream ss;
        ss << "src/object_recognition_pkg/output/vfh/vfh_" << i << ".pcd";
        writer.write<pcl::VFHSignature308> (ss.str (), *vfhFeatures, false); //*
        i++;

        //push to vector 
        std::vector<float> VFHValue(
            vfhFeatures->points[0].histogram,
            vfhFeatures->points[0].histogram + 308 
        );

        it.first.histogram = VFHValue;
        dataAddVFH.push_back(it);

        /** backup code before
        // modelsVFH.push_back (vfhFeatures);

        // VFHValues.push_back(vfhFeatures->points[0].histogram);
        // std::vector<float> VFHValue;
        // float vfh[308] = vfhFeatures->points[0].histogram;
        // std::cout << "size historgam : " << it.first.histogram.size() << std::endl;

        // for(auto it2 : it.first.histogram)
        // {
        //     std::cout << it2 << " ";
        //     // it.first.histogram;
        // }
        // std::cout << std::endl;

            // std::end(vfhFeatures->points[0].histogram) - std::end(vfhFeatures->points[0].histogram)
        // for(auto it : vfhFeatures->points[0].histogram)
        // {
        //     // std::cout << it << std::endl;
        //     VFHValue.push_back(it);
        // }
        // VFHValues.push_back(VFHValue);
        */
    }
    // std::cout << "jumlah vfh : " << VFHValues.size() << std::endl;


    // READ label/class trainned (dataset)
    std::vector<std::string> listDataSet;
    std::ifstream listDataSetFile("listDataSetv2.1.txt");
    for (std::string line ; getline(listDataSetFile, line);)
    {

        std::vector<std::string> stringLabelParsed;
        boost::algorithm::split(stringLabelParsed, line, boost::is_any_of("/"));
        listDataSet.push_back(stringLabelParsed[1]);
    }


    // ARTIFICIAL NEURAL NETOWRK

    std::vector<modelsDetail> dataCompleted;
    struct fann *ann = fann_create_from_file("bbbbbbb.net"); // generated from training
    fann_type *calc_out;
    fann_type input[308]; //length of VFH Descriptor
    for(auto it : dataAddVFH)
    {
        std::copy(it.first.histogram.begin(), it.first.histogram.end(), input);

        calc_out = fann_run(ann, input);

        //find max score 
        int maxIndex = 0, maxValue = 0;
        for(int i = 0 ; i < ann->num_output ; i++)
        {
            // std::cout << calc_out[i]  << " " ;//<< std::endl;
            //  std::cout << (calc_out[i] * 100) << " % " 
            //     << "   " << listDataSet[i] 
            //     << std::endl;

            if(calc_out[i] > maxValue){
                maxIndex = i;
            }

        }
        std::cout << "max : " << calc_out[maxIndex] * 100 << "% " << listDataSet[maxIndex] << std::endl;
        it.first.category = listDataSet[maxIndex];
        it.first.score = calc_out[maxIndex]*100.0;
        dataCompleted.push_back(it);
        
    }
    fann_destroy(ann);




    //visualize
    for(auto it:dataCompleted)
    {
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D (*it.second, minPt, maxPt);
        // std::cout << "Min x: " << minPt.x << std::endl;
        // std::cout << "Max x: " << maxPt.x << std::endl;
        // std::cout << "Min y: " << minPt.y << std::endl;
        // std::cout << "Max y: " << maxPt.y << std::endl;
        // std::cout << "Min z: " << minPt.z << std::endl;
        // std::cout << "Max z: " << maxPt.z << std::endl;
        std::string bboxId = "BBOX" + std::to_string(rand() % 100);
        std::string textId = "TEXT" + std::to_string(rand() % 100);
        std::stringstream stream;
        stream << std::fixed << std::setprecision(2) << it.first.score;
        std::string textLabel = (it.first.category + " " + stream.str() + "%"); 
        // std::cout << "bboxId : " << bboxId << std::endl;
        viewer.addCube(minPt.x, maxPt.x,  -maxPt.y , -minPt.y, -maxPt.z, -minPt.z, 1.0, 1.0, 1.0, bboxId, 0 );
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, bboxId);
        // viewer.addText("asdasdasdasd asdasdasdasd adasdas das d", maxPt.x, maxPt.y, textId);

        viewer.addText3D(textLabel, pcl::PointXYZ(maxPt.x, -maxPt.y, -minPt.z), 0.05, 255.0, 1.0, 1.0,  textId);

    }
    
    
    // pcl::visualization::PCLVisualizer viewer("Test visualize");
    // viewer.addPointCloud( cloud, "cloud"); 
    viewer.spin();
    // BoundingBox(cloud);
    // BoundingBox(cloud_cluster, viewer);
    

    // while(!viewer.wasStopped())
    // {
    //     // cloud->boundingBox(viewer, 1);
    //     // Viewer.addCube(Min.x, Max.x, Min.y, Max.y, Min.z, Max.z, 1, 0, 0, "AABB");
    //     viewer.addCube(0, 0, 0 , 1, 1, 1, 1, 0, 0, "AABB");
    //     viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

    //     /***
    //      * 
    //     Use this :
    //     viewer->addCube(bboxTransform, bboxQuaternion, maxP.x - minP.x, maxP.y -
    //     minP.y, maxP.z - minP.z, "bbox");

    //     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
    //     0.7, 0.7, 0, "bbox");
    //     viewer->setRepresentationToWireframeForAllActors();
    //      */


    //     // pcl::PointXYZRGB min_point,max_point;
    //     // Eigen::Vector4f pcaCentroid;
    //     // pcl::compute3DCentroid(*cloud,pcaCentroid);
    //     // Eigen::Matrix3f covariance;
    //     // pcl::computeCovarianceMatrixNormalized(*cloud,pcaCentroid,covariance);
    //     // //Compute eigenvectors and eigenvalues of covariance matrix using Eigen
    //     // Eigen::SelfAdjointEigenSolverEigen::Matrix3f eigen_solver(covariance, Eigen::ComputeEigenvectors);
    //     // //Eigen vectors
    //     // Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    //     // // Extract the eigenvalues and eigenvectors using PCL
    //     // Eigen::Vector3f eigen_values;
    //     // Eigen::Matrix3f eigen_vectors;
    //     // pcl::eigen33 (covariance, eigen_vectors, eigen_values);

    //     // /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
    //     // /// the signs are different and the box doesn't get correctly oriented in some cases.
    //     // eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    //     // // Transform the original cloud to the origin point where the principal components correspond to the axes.
    //     // Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    //     // projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    //     // projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());

    //     // pcl::PointCloudpcl::PointXYZRGB::Ptr cloudPointsProjected (new pcl::PointCloudpcl::PointXYZRGB);
    //     // pcl::transformPointCloud(*cloud,*cloudPointsProjected,projectionTransform);
    //     // // Get the minimum and maximum points of the transformed cloud.
    //     // pcl::getMinMax3D(cloudPointsProjected, min_point, max_point);
    //     // const Eigen::Vector3f meanDiagonal = 0.5f(max_point.getVector3fMap() + min_point.getVector3fMap());
    //     // //Finally, the quaternion is calculated using the eigenvectors (which determines how the final box gets rotated),
    //     // //and the transform to put the box in correct location is calculated.
    //     // //The minimum and maximum points are used to determine the box width, height, and depth.
    //     // const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations
    //     // //translation
    //     // const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    //     // //Add Cube
    //     // viewer_->addCube(bboxTransform, bboxQuaternion, max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z);

    //     viewer.spinOnce();
    // }


    return (0);
}