#include <iostream>
#include <Eigen/Dense>
#include "robot.h"

int main()
{
    robot puma560(6);

    puma560.m_links[0].transformation(0, 0, 0, 0);
    puma560.m_links[1].transformation(-M_PI_2, 0, 0, 0);
    puma560.m_links[2].transformation(0, 0.432, 0.149, 0);
    puma560.m_links[3].transformation(-M_PI_2, 0.02, 0.433, 0);
    puma560.m_links[4].transformation(M_PI_2, 0, 0, 0);
    puma560.m_links[5].transformation(-M_PI_2, 0, 0, 0);

    cout<< "T60齐次变换矩阵 = " << puma560.SerialLink() <<endl;

    Vector3f point;
    //某一关节上的某一点
    point << 1, 12, 12;

    cout<<"第5关节上点在基坐标系下的坐标："<<endl<<puma560.fk_point(5,point)<<endl;

    // Vector4f P;
    // Matrix4f
    
}