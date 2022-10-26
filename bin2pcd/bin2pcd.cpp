#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
 
#include <iostream>
#include <fstream>
#include <sys/types.h>  
#include <dirent.h> 
#include <vector>
 
using namespace pcl;
using namespace std;
 
namespace po = boost::program_options;
 void GetFileNames(string& path, vector<string>& filenames)
    {
        DIR *pDir;
        struct dirent* ptr;
        if(!(pDir = opendir(path.c_str())))
            return;
        while((ptr = readdir(pDir))!=0) {
            if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
                filenames.push_back(ptr->d_name);
        }
        closedir(pDir);
    }
int main(int argc, char **argv){

	string path1 = "/home/seuiv/Projects/jinshuai/PoinTr/data/KITTI/data/bins/";
	string path2 = "/home/seuiv/Projects/jinshuai/PoinTr/data/KITTI/data/pcds/";
	vector<string> filenames;
	GetFileNames(path1,filenames);
	for(int j=0;j<filenames.size();j++)
	{
			///The file to read from.
		string infile = path1 + filenames[j];
	
		///The file to output to.
		string outfile = path2 + filenames[j].substr(0,filenames[j].rfind(".")) + ".pcd";
	
		fstream input(infile.c_str(), ios::in | ios::binary);
		// cout<<infile.c_str()<<endl;

		if(!input.good()){
			cerr << "Could not read file: " << infile << endl;
			exit(EXIT_FAILURE);
		}
		input.seekg(0, ios::beg);
	
		pcl::PointCloud<PointXYZI>::Ptr points (new pcl::PointCloud<PointXYZI>);
	
		int i;
		for (i=0; input.good() && !input.eof(); i++) {
			PointXYZI point;
			input.read((char *) &point.x, 3*sizeof(float));
			input.read((char *) &point.intensity, sizeof(float));
			points->push_back(point);
		}
		input.close();
	
		cout << "Read KTTI point cloud with " << i << " points, writing to " << outfile << endl;
	
		pcl::PCDWriter writer;
	
		// Save DoN features
		writer.write<PointXYZI> (outfile, *points, false);
	}
	
}

