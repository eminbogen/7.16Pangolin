#include <iostream>
#include <iomanip>
#include <ctime>
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
//定义一个全局变量，用于保存生成的位姿
vector<vector<float>> pose_fin;
vector<string> v_rgb;vector<string> v_depth;
//给定初始速度和初始状态（运行）
int slam_speed=1;int command_go=1;

//首先需要辅助输出的结构体和子函数。为后面设计文本输出做准备
struct RotationMatrix
{
    Matrix3d matrix = Matrix3d::Identity();
};

ostream& operator << ( ostream& out, const RotationMatrix& r ) 
{
    out.setf(ios::fixed);
    Matrix3d matrix = r.matrix;
    out<<'=';
    out<<"["<<setprecision(2)<<matrix(0,0)<<","<<matrix(0,1)<<","<<matrix(0,2)<<"],"
    << "["<<matrix(1,0)<<","<<matrix(1,1)<<","<<matrix(1,2)<<"],"
    << "["<<matrix(2,0)<<","<<matrix(2,1)<<","<<matrix(2,2)<<"]";
    return out;
}

istream& operator >> (istream& in, RotationMatrix& r )
{
    return in;
}

struct TranslationVector
{
    Vector3d trans = Vector3d(0,0,0);
};

ostream& operator << (ostream& out, const TranslationVector& t)
{
    out<<"=["<<t.trans(0)<<','<<t.trans(1)<<','<<t.trans(2)<<"]";
    return out;
}

istream& operator >> ( istream& in, TranslationVector& t)
{
    return in;
}

//从groundtruth文件获取位姿信息
int get_pose(string path_to_data,vector<vector<float>> &pose,int index,int interval,vector<string> &v_rgb,vector<string> &v_depth);

//生成地图
void get_map();

int main()
{
    //给予文件地址
    string path_to_data = "/home/eminbogen/NEW MXZ/SLAM/SLAM-14/SLAM_dataset/room_data";
    //定义暂时读取位姿信息存储的vector，读取总帧数和读取间隔
    vector<vector<float>> pose;int index=100;int interval =80;
    //取得pose
    get_pose(path_to_data,pose,index,interval,v_rgb,v_depth);
    //定义线程
    std::thread render_loop;
    //开启线程
    render_loop = std::thread(get_map);
    //转换原groundtruth下的数据格式（四元数）到适合pangolin的数据格式（旋转矩阵）
    for (int i=0;i<index;i++)
    {
	  //存储四元数
	  Eigen::Quaterniond quaternion(pose[i][3],pose[i][4],pose[i][5],pose[i][6]);
	  //存储旋转矩阵
	  Eigen::Matrix3d rotation_matrix;
	  //四元数转化旋转矩阵
	  rotation_matrix=quaternion.matrix();
	  //定义一个暂时的pose_temp存储12个位姿数据，9个旋转矩阵的元素，3各位置元素
	  vector<float> pose_temp;
	  //旋转矩阵元素
	  pose_temp.push_back(rotation_matrix(0,0));	pose_temp.push_back(rotation_matrix(1,0));	pose_temp.push_back(rotation_matrix(2,0));
	  pose_temp.push_back(rotation_matrix(0,1));	pose_temp.push_back(rotation_matrix(1,1));	pose_temp.push_back(rotation_matrix(2,1));
	  pose_temp.push_back(rotation_matrix(0,2));	pose_temp.push_back(rotation_matrix(1,2));	pose_temp.push_back(rotation_matrix(2,2));
	  //位置元素
	  pose_temp.push_back(pose[i][0]);			pose_temp.push_back(pose[i][1]);				pose_temp.push_back(pose[i][2]);
	  //将pose_temp存入全局变量pose用于构图，也就是每一行的pose都是一个pose_temp，12个数，最后会有index行
	  pose_fin.push_back(pose_temp);
	  //清空pose_temp内存
	  pose_temp.clear();
	  //暂定
	  //利用序列更新时间长短变化改变更新地图速度
	  while(!command_go){}
	  usleep(1000000/slam_speed);
    }
    //清空pose内存
    pose.clear();
    //收束线程
    render_loop.join();
    return 0;
}

int get_pose(string path_to_data,vector<vector<float>> &pose,int index,int interval,vector<string> &v_rgb,vector<string> &v_depth)
{
    //定义文件路径
    string path_to_pose = path_to_data + "/groundtruth.txt";
    string path_to_imagepath = path_to_data + "/associate.txt";
    
    //检测文件是否存在，1为位姿，2为图片
    ifstream fin1( path_to_pose );
    if ( !fin1 ) 
    {
        cerr<<"I cann't find groundtruth.txt!"<<endl;
	return 1;
    }
    
    ifstream fin2( path_to_imagepath );
    if ( !fin2 ) 
    {
        cerr<<"I cann't find associate.txt!"<<endl;
	return 1;
    }
    
    //定义用于存储fin2中时间与路径的变量
    string rgb_path,depth_path;
    double rgb_time,depth_time;
    
    for ( int i=0; i<index; i++ )
    {
	double temp_pose;float temp[8];vector< float>pose_temp;int interval_temp = interval;
	while(interval_temp--) fin1>>temp_pose>>temp[1]>>temp[2]>>temp[3]>>temp[4]>>temp[5]>>temp[6]>>temp[7];
	pose_temp.push_back(temp[1]);pose_temp.push_back(temp[2]);pose_temp.push_back(temp[3]);
	pose_temp.push_back(temp[7]);pose_temp.push_back(temp[4]);pose_temp.push_back(temp[5]);pose_temp.push_back(temp[6]);
	pose.push_back(pose_temp);
	pose_temp.clear();
	
	//这是一个标志
	int flag_num_match = 1;
	while(flag_num_match)
	{
	       fin2>>rgb_time>>rgb_path>>depth_time>>depth_path;
	       //ta用于检测fin1读取位姿的时间和fin2读取的时间差距是否小于0.1s，小于则跳出死循环
	       if(abs(temp_pose-rgb_time)<0.1) flag_num_match=0;
	}
	//并把该时刻下的图片路径记录
	v_rgb.push_back(path_to_data+"/"+rgb_path);
	v_depth.push_back(path_to_data+"/"+depth_path);
    }
    return 1;
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

    //设计显示面板
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,0.3);
    //第一个参数为按钮的名字，第二个为默认状态，第三个为最低值，第四个为最高值
    pangolin::Var<int> a_int("menu.slam_speed",2,1,10);
    //第一个参数为按钮的名字，第二个为默认状态，第三个为是否有选择框
    pangolin::Var<bool> menu("menu.lines",true,true);
    pangolin::Var<bool> goon("menu.go_on",true,true);
    //设计文本输出于面板
    //前面使用了结构体为变量，后面使用了函数作输出，中间定义的变量供后续使用。
    //结构体：RotationMatrix
    //中间量：rotation_matrix
    //子函数：RotationMatrix
    pangolin::Var<RotationMatrix> rotation_matrix("menu.r", RotationMatrix());
    pangolin::Var<TranslationVector> translation_vector("menu.t", TranslationVector());
    // 设置按钮
    pangolin::Var<bool> save_window("menu.Save_Window",false,false);
    pangolin::Var<bool> save_cube("menu.Save_Cube",false,false);
    pangolin::Var<std::function<void(void)> > reset("menu.ESC");

    //定义图片面板
    pangolin::View& rgb_image = pangolin::Display("rgb")
    .SetBounds(0,0.3,0.3,0.65,1024.0f/768.0f)
    .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    pangolin::View& depth_image = pangolin::Display("depth")
    .SetBounds(0,0.3,0.65,1,1024.0f/768.0f)
    .SetLock(pangolin::LockLeft, pangolin::LockBottom);
    
    //初始化
    pangolin::GlTexture imageTexture(640,480,GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);
    while( !pangolin::ShouldQuit() )
    {
        clock_t time_stt = clock();
	//清除颜色缓冲和深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
	
	//背景先弄成白色的吧，我觉得白色比较好看
	glClearColor(1.0f,1.0f,1.0f,1.0f);
	
	//使用变换矩阵画图
	int k=0;
	for(k =0;k<pose_fin.size();k++)
	{
	    //使用位置变换矩阵
	    glPushMatrix();
	    //变换如该矩阵，注意这个变换矩阵是转置的
	    std::vector<GLfloat> Twc ={ pose_fin[k][0],pose_fin[k][1],pose_fin[k][2],0,
							      pose_fin[k][3],pose_fin[k][4],pose_fin[k][5],0,
							      pose_fin[k][6],pose_fin[k][7],pose_fin[k][8],0,
							      pose_fin[k][9],pose_fin[k][10],pose_fin[k][11],1 };
	    //变换
	    glMultMatrixf(Twc.data());
	    //每次变换后绘制相机
	    glLineWidth(2);
	    glBegin(GL_LINES);
	    glColor3f(0.0f,0.0f,1.0f);
	    glVertex3f(0,0,0);		glVertex3f(w,h,z);
	    glVertex3f(0,0,0);		glVertex3f(w,-h,z);
	    glVertex3f(0,0,0);		glVertex3f(-w,-h,z);
	    glVertex3f(0,0,0);		glVertex3f(-w,h,z);
	    glVertex3f(w,h,z);		glVertex3f(w,-h,z);
	    glVertex3f(-w,h,z);		glVertex3f(-w,-h,z);
	    glVertex3f(-w,h,z);		glVertex3f(w,h,z);
	    glVertex3f(-w,-h,z);		glVertex3f(w,-h,z);
	    
	    glEnd();
	    glPopMatrix();
	}
	cout<<"k="<<k<<endl;
	
	//运行R，t输出
	if(pose_fin.size()>0)
	{
	    RotationMatrix R; 
	    for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
		    R.matrix(j,i) = double(pose_fin[k-1][3*j+i]);
	   rotation_matrix = R;
	   
	   TranslationVector t;
	   t.trans = Vector3d(pose_fin[k-1][9],pose_fin[k-1][10],pose_fin[k-1][11]);
	   t.trans = -R.matrix*t.trans;
	   translation_vector = t;
	}
	
	//速度变更
	slam_speed =a_int;
	//运行状态变更
	if(goon) command_go = 1;if(!goon) command_go = 0;
	//使用按钮
	if( pangolin::Pushed(save_window) )
        pangolin::SaveWindowOnRender("window");
	if( pangolin::Pushed(save_cube) )
        d_cam.SaveOnRender("cube");
	
	//绘制连接的绿色线，并根据选项决定是否绘制绿线
	if(menu)
	{
	    glLineWidth(2);
	    glBegin ( GL_LINES );
	    glColor3f ( 0.0f,1.f,0.f );
	    for(int i=0;i<pose_fin.size()-1;i++)
	    {
		glVertex3f( pose_fin[i][9],pose_fin[i][10],pose_fin[i][11]);
		glVertex3f( pose_fin[i+1][9],pose_fin[i+1][10],pose_fin[i+1][11] );
	    }
	    glEnd();
	}

	//图像读取，显示
	cv::Mat rgb;
        rgb = cv::imread (v_rgb.at(k-1));
        imageTexture.Upload(rgb.data,GL_BGR,GL_UNSIGNED_BYTE);
        rgb_image.Activate();
        glColor3f(1.0,1.0,1.0);
        imageTexture.RenderToViewportFlipY();
	
	depth_image.Activate();
	cv::Mat depth;
        depth = cv::imread (v_depth.at(k-1));
        imageTexture.Upload(depth.data,GL_BGR,GL_UNSIGNED_BYTE);
	glColor3f(1.0,1.0,1.0);
        imageTexture.RenderToViewportFlipY();
	
        //交换帧和并推进事件
        pangolin::FinishFrame();
	cout <<"time use is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms"<< endl;
    }
}
