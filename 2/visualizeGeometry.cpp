#include <pangolin/pangolin.h>

int main(  )
{
    //生成一个gui界面，定义大小
    pangolin::CreateWindowAndBind("Main",640,480);
    //进行深度测试，保证某视角下像素只有一种颜色，不混杂
    glEnable(GL_DEPTH_TEST);

    //放置一个相机
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );

    //创建视角窗口
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    while( !pangolin::ShouldQuit() )
    {
        //清除颜色缓冲和深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        //绘制立方体
        pangolin::glDrawColouredCube();
	
	//绘制坐标系  线宽 启动 颜色 起点 终点 停止 
	glLineWidth(3);
	glBegin ( GL_LINES );
	glColor3f ( 0.8f,0.f,0.f );
	glVertex3f( -1,-1,-1 );
	glVertex3f( 0,-1,-1 );
	glColor3f( 0.f,0.8f,0.f);
	glVertex3f( -1,-1,-1 );
	glVertex3f( -1,0,-1 );
	glColor3f( 0.2f,0.2f,1.f);
	glVertex3f( -1,-1,-1 );
	glVertex3f( -1,-1,0 );
	glEnd();

        //交换帧和并推进事件
        pangolin::FinishFrame();
    }
    
    return 0;
}
