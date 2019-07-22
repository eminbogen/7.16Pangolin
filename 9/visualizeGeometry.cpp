#include <iostream>
#include <iomanip>
#include <ctime>
#include <boost/format.hpp> 
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

#include <pangolin/pangolin.h>
//多线程的库
#include <thread>
//OPENCV库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

vector<vector<double>> pose_of_world;
void get_map();
int main( int argc, char** argv )
{
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;         // 相机位姿
    cv::Mat image;
    ifstream fin("../5.0/pose.txt");//
    if (!fin)
    {
        cerr<<"请在有pose.txt的目录下运行此程序"<<endl;
        return 1;
    }
    
    for ( int i=0; i<5; i++ )
    {
        boost::format fmt( "../5.0/%s/%d.%s" ); //图像文件格式
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
	cout<<(fmt%"color"%(i+1)%"png").str()<<endl;
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // 使用-1读取原始图像
        
        double data[7] = {0};
        for ( auto& d:data )
            fin>>d;
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d T(q);
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
        poses.push_back( T );
    }
    
    // 计算点云并拼接
    // 相机内参 
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    
    cout<<"正在将图像转换为pangolin..."<<endl;
    
    for ( int i=0; i<5; i++ )
    {
        cout<<"转换图像中: "<<i+1<<endl; 
        cv::Mat color = colorImgs[i]; 
	cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];	
        for ( int v=0; v<color.rows; v++ )
	{
	    uchar* data = color.ptr<uchar>(v);
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                if ( d==0 ) continue; // 为0表示没有测量到
                Eigen::Vector3d point; vector<double> point_pose;
                point[2] = double(d)/depthScale; 
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 
                Eigen::Vector3d pointWorld = T*point;
		point_pose.push_back(pointWorld[0]);
		point_pose.push_back(pointWorld[1]);
		point_pose.push_back(pointWorld[2]);
		point_pose.push_back(double(data[3*u+0]));
		point_pose.push_back(double(data[3*u+1]));
		point_pose.push_back(double(data[3*u+2]));
		pose_of_world.push_back(point_pose);
		point_pose.clear();
	    }
	  
	}
    } 
    get_map();
    return 0;
}

void get_map()
{
    const float w=0.2;
    const float h=w*0.75;
    const float z=w*0.6;
    //生成一个gui界面，定义大小
    pangolin::CreateWindowAndBind("Main",1800,1280);
    //进行深度测试，保证某视角下像素只有一种颜色，不混杂
    glEnable(GL_DEPTH_TEST);

    //放置一个相机
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1800,1200,420,420,900,640,0.2,2000),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );

    //创建视角窗口
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

 
    while( !pangolin::ShouldQuit() )
    {
        clock_t time_stt = clock();
	//清除颜色缓冲和深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
	
	//背景先弄成白色的吧，我觉得白色比较好看
	glClearColor(1.0f,1.0f,1.0f,1.0f);
	
	//画点
	glPointSize(2);
	glBegin(GL_POINTS);
	for(int p=0;p<pose_of_world.size();p++)
	{
	    glColor3d(pose_of_world[p][5]/255,pose_of_world[p][4]/255,pose_of_world[p][3]/255);
	    glVertex3d(pose_of_world[p][0],pose_of_world[p][1],pose_of_world[p][2]);
	}
	    glEnd();

        //交换帧和并推进事件
        pangolin::FinishFrame();
	cout <<"time use is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms"<< endl;
    }
}