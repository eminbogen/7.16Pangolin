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

//定义一个全局变量，用于保存生成的位姿
vector<vector<float>> pose_fin;
//给定初始速度和初始状态（运行）
int slam_speed=1;int command_go=1;

//从groundtruth文件获取位姿信息
int get_pose(string path_to_dataset,vector<vector<float>> &pose,int index,int interval);

//生成地图
void get_map();

int main()
{
    //给予文件地址
    string path_to_dataset = "../groundtruth.txt";
    //定义暂时读取位姿信息存储的vector，读取总帧数和读取间隔
    vector<vector<float>> pose;int index=100;int interval =80;
    //取得pose
    get_pose(path_to_dataset,pose,index,interval);
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

int get_pose(string path_to_dataset,vector<vector<float>> &pose,int index,int interval)
{
    //检测文件是否存在
    ifstream fin( path_to_dataset );
    if ( !fin ) 
    {
        cerr<<"I cann't find txt!"<<endl;
	return 1;
    }
    //循环取值给pose，取帧数量为index
    for ( int i=0; i<index; i++ )
    {
	//定义暂时量用于读取操作，定义pose_temp用于向pose添加数据，设定选取间隔为interval
	float temp[8];vector< float>pose_temp;int interval_temp = interval;
	//循环读取文件每行数据，直到满足interval行
	while(interval_temp--) fin>>temp[0]>>temp[1]>>temp[2]>>temp[3]>>temp[4]>>temp[5]>>temp[6]>>temp[7];
	//先把7个数给pose_temp，3个位置元素，还有四元数，注意我先加入的temp[7]也就是四元数的实部
	pose_temp.push_back(temp[1]);pose_temp.push_back(temp[2]);pose_temp.push_back(temp[3]);
	pose_temp.push_back(temp[7]);pose_temp.push_back(temp[4]);pose_temp.push_back(temp[5]);pose_temp.push_back(temp[6]);
	//把pose_temp堆入pose
	pose.push_back(pose_temp);
	//清空pose_temp内存
	pose_temp.clear();
    }
    return 1;
}

void get_map()
{
    const float w=0.2;
    const float h=w*0.75;
    const float z=w*0.6;
    //生成一个gui界面，定义大小
    pangolin::CreateWindowAndBind("Main",1280,960);
    //进行深度测试，保证某视角下像素只有一种颜色，不混杂
    glEnable(GL_DEPTH_TEST);

    //放置一个相机
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1280,960,420,420,640,480,0.2,2000),
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
    // 设置按钮
    pangolin::Var<bool> save_window("menu.Save_Window",false,false);
    pangolin::Var<bool> save_cube("menu.Save_Cube",false,false);
    pangolin::Var<std::function<void(void)> > reset("menu.ESC");
    while( !pangolin::ShouldQuit() )
    {
        //清除颜色缓冲和深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
	
	//背景先弄成白色的吧，我觉得白色比较好看
	glClearColor(1.0f,1.0f,1.0f,1.0f);
	
	//使用变换矩阵画图
	for(int i=0;i<pose_fin.size();i++)
	{
	    //使用位置变换矩阵
	    glPushMatrix();
	    //变换如该矩阵，注意这个变换矩阵是转置的
	    std::vector<GLfloat> Twc ={ pose_fin[i][0],pose_fin[i][1],pose_fin[i][2],0,
							      pose_fin[i][3],pose_fin[i][4],pose_fin[i][5],0,
							      pose_fin[i][6],pose_fin[i][7],pose_fin[i][8],0,
							      pose_fin[i][9],pose_fin[i][10],pose_fin[i][11],1 };
	    
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

        //交换帧和并推进事件
        pangolin::FinishFrame();
    }
}
