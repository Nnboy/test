
#include <vector>
using namespace std;

#include <Eigen/Dense>
using namespace Eigen;

enum JointType
{
	JointType_Revolute,
	JointType_Translate
};

class DHParameter
{
public:
	DHParameter()
		: m_jointtype(JointType_Revolute)
	{
	}
	~DHParameter() {}

private:
	double a;
	double alpha;
	double d;
	double theta;

	JointType m_jointtype;

public:
	Matrix4f Trans;
	void transformation(float alpha, float a, float d, float theta)
	{
		Matrix4f m;
		m << cos(theta), -sin(theta), 0, a,
			sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d,
			sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), cos(alpha) * d,
			0, 0, 0, 1;
		Trans = m;
	}
};

class robot
{
public:
	robot(int a) : joints(a) {}
	~robot() {}

	DHParameter m_links[6];

private:
	int joints;

public:

	Vector3f fk_point(unsigned int link_index, Vector3f ref_position)
	{
		Matrix4f T;
		Vector4f position;
		position << ref_position.head(3), 1;
		T.setIdentity(4, 4);
		for (size_t i = 0; i < link_index; i++)
		{
			T *= m_links[i].Trans;
		}
		return (T * position).head(3);
	};

	Matrix4f SerialLink()
	{
		Matrix4f T;
		T.setIdentity(4, 4);
		for (size_t i = 0; i < joints; i++)
		{
			T *= m_links[i].Trans;
		}
		return T;
	}
};
