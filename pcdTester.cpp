#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <string.h>

using namespace pcl;
using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointT> XYZCloud;
typedef pcl::PointCloud<PointNormalT> XYZNormalCloud;

class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation ()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const PointNormalT &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

void pre_process_cloud(XYZCloud::Ptr &pcd,XYZNormalCloud::Ptr &out, bool downsample, float *ds_scale = NULL, int vicinity = 30) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr src;
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    src = pcd;
    if(!ds_scale) {
        ds_scale = new float[3];
        for(int i = 0; i < 3; ds_scale[i++] = 0.1);
    }
    //Downsampling...

    std::vector<int> idx;
    src->is_dense = false;
    pcl::removeNaNFromPointCloud(*src,*src,idx);
    if(downsample) {
        src.reset(new pcl::PointCloud<pcl::PointXYZ>);
        grid.setLeafSize (ds_scale[0], ds_scale[1], ds_scale[2]);
        grid.setInputCloud (pcd);
        grid.filter(*src);
    }


    //Normal Calculation...

    pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ,pcl::PointNormal> ne;
    ne.setInputCloud(src);
    ne.setSearchMethod(tree);
    ne.setKSearch(vicinity);
    ne.compute(*normals);

    pcl::concatenateFields(*src,*normals,*out);
    pcl::concatenateFields(*src,*out,*out);
}

void calculate_statistics(std::vector<float> &curvatures,float &mean,float &var) {
    mean = 0.;
    var = 0.;
    for(int i = 0; i < curvatures.size();i++) {
        mean += curvatures.at(i);
        var +=curvatures.at(i) * curvatures.at(i);
    }
    mean = mean/curvatures.size(); //E[X]
    var = var/curvatures.size(); //E[X²]
    var = var - (mean*mean); //var = E[X²] - (E[X])²
}

bool isInStableArea(std::vector<XYZNormalCloud::Ptr> &clouds,
                    std::vector< KdTreeFLANN<PointNormalT>::Ptr > &trees,
                    PointNormalT searchPoint,
                    std::vector<int> neighborhood_scale)
{


    std::cout << "sup bitches!!: Point: " << searchPoint.x << "," << searchPoint.y << "," << searchPoint.z << endl;


    if(!(clouds.size() == trees.size() && trees.size() == neighborhood_scale.size())) {
        std::cout << "The sizes of the vectors does not agree." << endl;
        return false;
    }
    std::vector<float> curvature_variances;
    std::vector<float> curvature_means;

    for(int i = 0; i < clouds.size();i++) {
        //Looks for (neighborhhod_scale[i]) neighbours of pointSearch at point cloud i
        KdTreeFLANN<PointNormalT> kdtree = *(trees.at(i));
        vector<int> pointIdxNKNSearch(neighborhood_scale[i]);
        vector<float> pointNKNSquaredDistance(neighborhood_scale[i]);

        std::cout << "fetching neighbours..." << endl;
        int neighbours = kdtree.nearestKSearch (searchPoint, neighborhood_scale[i], pointIdxNKNSearch, pointNKNSquaredDistance);
        std::cout << "got them" << endl;

        if(neighbours > 0) { //If there is a vicinity....
            //warns if the nbeighbourhood is smaller than expected
            if(neighbours < neighborhood_scale[i])
                std::cout << "Less points then expected..." << std::endl;


            std::cout << "let's search!" << endl;


            //copy the curvature values of each neighbour point and calculates its mean and variance..
            std::vector<float> local_curvatures(neighbours);
            for(int j = 0; j < pointIdxNKNSearch.size();j++) {
                int pointIdx = pointIdxNKNSearch.at(j);
                local_curvatures.push_back(clouds.at(i)->at(pointIdx).curvature);
            }


            cout << "search done succesfully" << endl;


            float mean,var;
            calculate_statistics(local_curvatures,mean,var);
            curvature_means.push_back(mean); curvature_variances.push_back(var);
        }
        else {
            cout << "No neighbours, what now?" << endl;
        }
    }


    std::cout << "sup bitches!! the eend!" << endl;


    float mean,var;
    calculate_statistics(curvature_variances,mean,var);
    std::cout << "var of vars: " << var << endl;

    return false;
}

void searchForStablePoints(XYZNormalCloud::Ptr cloud,vector<int> &vicinity) {
    vector<XYZNormalCloud::Ptr> clouds;

    /** **************************************** Uncomment for visualization... *************************************
    visualization::PCLVisualizer *v = new visualization::PCLVisualizer("Visualizaiton testset");
    v->setBackgroundColor(0,0,0);
    int vp_idx[vicinity.size()];
    float vpsize = 1.f/(float)vicinity.size();
    float vpLast = 0.;
    for(int i =0; i < vicinity.size();i++) {
        v->createViewPort(0,vpLast,1,vpLast+vpsize,vp_idx[i]);
        vpLast += vpsize;
    }
    */

    //Calculates different normal estimations, varying the neighbourhood size.
    //and storing at clouds vector.
    pcl::NormalEstimation<PointNormalT,PointNormalT> ne;
    pcl::search::KdTree<PointNormalT>::Ptr tree(new pcl::search::KdTree<PointNormalT>);
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);

    for(int i = 0; i < vicinity.size(); i++) {
        XYZNormalCloud::Ptr newCloud(new XYZNormalCloud);
        newCloud->resize(cloud->size());
        pcl::concatenateFields(*cloud,*newCloud,*newCloud);
        ne.setKSearch(vicinity.at(i));
        ne.compute(*newCloud);
        clouds.push_back(newCloud);
        /**
        char n[10];
        snprintf(n,10,"%d",i);
        visualization::PointCloudColorHandlerGenericField<PointNormalT> c(newCloud,"curvature");
        v->addPointCloud(newCloud,c,n,vp_idx[i]);
        */
    }
    /** v->spin(); */

    //Build kdTrees for each cloud... (only necessary to have a different kdtree for each cloud if the sampling factor varies)
    std::vector< KdTreeFLANN<PointNormalT>::Ptr > trees;
    MyPointRepresentation point_rep;
    float alpha[4] = {1.0,1.0,1.0,1.0};
    point_rep.setRescaleValues(alpha);
    for(int i = 0; i < clouds.size();i++) {
        KdTreeFLANN<PointNormalT>::Ptr t(new KdTreeFLANN<PointNormalT>);
        t->setInputCloud(clouds.at(i));
        t->setPointRepresentation(boost::make_shared<const MyPointRepresentation> (point_rep));
        trees.push_back(t);
    }
    for(int i = 0; i < cloud->size(); i++)
        isInStableArea(clouds,trees,cloud->at(i),vicinity);
}

int main() {
    vector<string> filenames;
    vector<XYZCloud::Ptr> clouds;

    filenames.push_back("regCloud_AfterStairBag11.pcd");
    filenames.push_back("regCloud_SingleStalactiteBag8.pcd");
    filenames.push_back("regCloud_StairSceneBag11.pcd");

    for(int i = 0; i < filenames.size(); i++) {
        XYZCloud::Ptr pcd(new XYZCloud);
        io::loadPCDFile(filenames.at(i),*pcd);
        clouds.push_back(pcd);
    }
    XYZNormalCloud::Ptr processedPcd1(new XYZNormalCloud);
    pre_process_cloud(clouds.at(2),processedPcd1,1,NULL,5);

    vector<int> vicinity;
    vicinity.push_back(5);vicinity.push_back(10);vicinity.push_back(15);
    searchForStablePoints(processedPcd1,vicinity);

    return 0;
}
